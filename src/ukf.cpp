#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
using namespace Eigen;
using namespace std;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = false;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd::Zero(5);
  x_aug_=VectorXd(7);

  // initial covariance matrix
  P_ = MatrixXd::Identity(5,5);
  P_aug_=MatrixXd::Zero(7,7);
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2.5;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  //Initializing the Sigma Point Matrices
  Xsig_pred_=MatrixXd::Ones(5,15);
  Xsig_aug_=MatrixXd::Ones(7,15);
  Zsig_upd_=MatrixXd::Ones(3,15);

//Initializing the R and H matrices for the Laser and Radar
  R_radar_=MatrixXd(3,3);
  R_laser_=MatrixXd(2,2);
  H_=MatrixXd(2,5);

  H_<<1,0,0,0,0,
      0,1,0,0,0;

  R_laser_<<std_laspx_*std_laspx_,0,
            0,std_laspy_*std_laspy_;
  R_radar_<<std_radr_*std_radr_,0,0,
            0,std_radphi_*std_radphi_,0,
            0,0,std_radrd_*std_radrd_;

  //Initializing other Variables
  is_initialized_=false;

  n_x_=5;

  n_aug_=7;

  lambda_=3-n_aug_;

  //Weights Initialization
  weights_=VectorXd::Zero(15);
  weights_(0)=(lambda_/(lambda_+n_aug_));
  weights_.tail(2*n_aug_) = Eigen::VectorXd::Constant(2*n_aug_,1/(2*(lambda_+n_aug_)));
   time_us_=0; 
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
    
if (is_initialized_==false)
{
 
  cout << "UKF: " << endl;
	time_us_=meas_package.timestamp_;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    x_(0)=meas_package.raw_measurements_(0)*cos(meas_package.raw_measurements_(1));
    x_(1)=meas_package.raw_measurements_(0)*sin(meas_package.raw_measurements_(1));
    x_(2)=meas_package.raw_measurements_(2);
    P_(0,0)=std_laspx_*std_laspx_;
    P_(1,1)=std_laspy_*std_laspy_;
    //P_(3,3)=200;
    //P_(4,4)=200;   // Initializing the Co-variances of the yaw and yawdd as a higher value than the measurements since Radar measures velocity
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
	//do nothing as we need the radar measurement to update the velocities. 	
    x_(0)=meas_package.raw_measurements_(0);
    x_(1)=meas_package.raw_measurements_(1);
    //x_(2)=0.5;
    P_(0,0)=std_laspx_*std_laspx_;
    P_(1,1)=std_laspy_*std_laspy_;
    //P_(2,2)=200;
    //P_(3,3)=200;
    //P_(4,4)=200; // Initializing the Co-variances of the velocity, yaw and yawdd as a higher value than the measurements since Lidar does not measure velocity
    }
    // done initializing, no need to predict or update
    is_initialized_ = true; 
}
else
{
double delta_T=(meas_package.timestamp_-time_us_)/(double)1000000.;

Prediction(delta_T);

if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
{
  UpdateRadar(meas_package);
}
else if (meas_package.sensor_type_==MeasurementPackage::LASER)
{
  UpdateLidar(meas_package);
}
}
time_us_=meas_package.timestamp_;
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  MatrixXd A_(7,7);
  VectorXd F_(5);
  VectorXd v_(5);
  VectorXd x_diff_=VectorXd::Zero(5);
   x_aug_<<x_,
          0,
          0;
    
  P_aug_.block<5,5>(0,0) = P_;
  P_aug_(5,5)=std_a_*std_a_;
  P_aug_(6,6)=std_yawdd_*std_yawdd_;    //Adding Process Noise to the Covariance Matrix
  
  //Generate Sigma points
  A_=P_aug_.llt().matrixL();
  Xsig_aug_<<x_aug_,(1.732*(A_)).colwise()+x_aug_,(-1.732*(A_)).colwise()+x_aug_;

  for (int i=0; i<=2*n_aug_;i++)
  {
    double p_x = Xsig_aug_(0,i);
    double p_y = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

// Calculate the State Transition Vector
    if(yawd<0.001)
  {
    F_(0)=v*delta_t*cos(yaw);
    F_(1)=v*delta_t*sin(yaw); 
  } 
    else
  {
    F_(0)=(v/yawd)*(sin(yaw+yawd*delta_t)-sin(yaw));
    F_(1)=(v/yawd)*(-cos(yaw+yawd*delta_t)+cos(yaw));
  }
  F_(2)=0;
  F_(3)=yawd*delta_t;
  F_(4)=0;

// Calculate Process Noise Vector
  v_(0)=(1/2)*delta_t*delta_t*cos(yaw)*nu_a;
  v_(1)=(1/2)*delta_t*delta_t*sin(yaw)*nu_a;
  v_(2)=nu_a*delta_t;
  v_(3)=(1/2)*delta_t*delta_t*nu_yawdd;
  v_(4)=delta_t*nu_yawdd;
  
  Xsig_pred_.col(i)=Xsig_aug_.col(i).head(5)+F_+v_; //Adding the State Transition, F and the Process Noise Vector
  }

  //Calculate the mean state and covariance

  P_=MatrixXd::Zero(5,5);
  x_=VectorXd::Zero(5);

  for (int i=0;i<=Xsig_pred_.cols()-1;i++)
  {
    x_=x_+Xsig_pred_.col(i)*weights_(i);
  }
  
    for (int i=0;i<=Xsig_pred_.cols()-1;i++)
  {
    x_diff_=Xsig_pred_.col(i)-x_;
      while (x_diff_(3)>M_PI)
    {
      x_diff_(3)-=2*M_PI;
    }
    while (x_diff_(3)<-M_PI)
    {
      x_diff_(3)+=2*M_PI;
    }                           // Normalizing the angle between pi and -pi.
    P_= P_+ (weights_(i)*((x_diff_)*(x_diff_.transpose())));
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
    VectorXd y(2);
  MatrixXd S=MatrixXd(2,2);
  MatrixXd K=MatrixXd(5,2);
  MatrixXd I=MatrixXd::Identity(5,5);
y=meas_package.raw_measurements_-H_*x_;       // Lidar Kalman Filter Equations
S=H_*P_*H_.transpose()+R_laser_;
K= P_*H_.transpose()*S.inverse();
x_=x_+K*y;
P_=(I-(K*H_))*P_;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
VectorXd z_=VectorXd::Zero(3);
VectorXd z_diff_=VectorXd::Zero(3);
VectorXd x_diff_=VectorXd::Zero(5);
  MatrixXd S=MatrixXd::Zero(3,3);
  MatrixXd K=MatrixXd::Zero(4,3);
  MatrixXd T=MatrixXd::Zero(n_x_,3);
  Zsig_upd_=PredictMeasurement(Xsig_pred_);   //Predict measurement for the state sigma points

    for (int i=0;i<=Zsig_upd_.cols()-1;i++)
  {
    z_=z_ + Zsig_upd_.col(i)*weights_(i);     //Calculating the mean of the sigma points
  }

  for (int i=0;i<=Zsig_upd_.cols()-1;i++)
  {
    x_diff_=Xsig_pred_.col(i)-x_;
    z_diff_=Zsig_upd_.col(i)-z_;
    while (z_diff_(1)>M_PI)
    {
      z_diff_(1)-=2*M_PI;
    }
    while (z_diff_(1)<-M_PI)
    {
      z_diff_(1)+=2*M_PI;
    }
    while (x_diff_(3)>M_PI)
    {
      x_diff_(3)-=2*M_PI;
    }
    while (x_diff_(3)<-M_PI)
    {
      x_diff_(3)+=2*M_PI;
    }                                       //Normalizing Angles
    
  S= S + (weights_(i)*(z_diff_*(z_diff_.transpose())));   
  T= T + (weights_(i)*(x_diff_*(z_diff_.transpose())));   
  }
  S= S+R_radar_;

  K=T*S.inverse();                          //UKF Equations

  z_diff_=meas_package.raw_measurements_-z_;

    while (z_diff_(1)>M_PI)
    {
      z_diff_(1)-=2*M_PI;
    }
    while (z_diff_(1)<-M_PI)
    {
      z_diff_(1)+=2*M_PI;
    }                                       //Normalizing Angles

  x_=x_+K*(z_diff_);
  P_=P_-K*S*K.transpose();                  //UKF Equations
}
Eigen::MatrixXd UKF::PredictMeasurement(MatrixXd Xsig_) {
  MatrixXd Zsig_(Zsig_upd_.rows(),Zsig_upd_.cols());      

  for (int i=0; i<=Xsig_.cols()-1;i++)
  {
  Zsig_(0,i)=sqrt((Xsig_(0,i)*Xsig_(0,i))+(Xsig_(1,i)*Xsig_(1,i))); 
  Zsig_(1,i)=atan2(Xsig_(1,i),Xsig_(0,i));
  Zsig_(2,i)=((Xsig_(0,i)*cos(Xsig_(3,i))*Xsig_(2,i))+(Xsig_(1,i)*sin(Xsig_(3,i))*Xsig_(2,i)))/sqrt((Xsig_(0,i)*Xsig_(0,i))+(Xsig_(1,i)*Xsig_(1,i)));
  }
  return (Zsig_);
}
