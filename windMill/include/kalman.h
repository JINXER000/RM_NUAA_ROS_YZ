#ifndef KALMAN_H
#define KALMAN_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include "MarkerSensor.h"
using namespace cv;
class Kalman_Filter {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  Kalman_Filter(
      double dt,
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P
  );

  /**
  * Create a blank estimator.
  */
  Kalman_Filter();

  /**
  * Initialize the filter with initial states as zero.
  */
  void init();

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(double t0, const Eigen::VectorXd& x0);

  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void update(const Eigen::VectorXd& y);

  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

  /**
  * Return the current state and time.
  */
  Eigen::VectorXd state() { return x_hat; };
  double time() { return t; };
  int resetCounter;
  void lostAndReset(int maxLost);
private:

  // Matrices for computation
  Eigen::MatrixXd A, C, Q, R, P, K, P0;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
};

// the following class use opencv build-in lib
class cvKalmanFilter
{
public:
	cvKalmanFilter(int x, int y):
		KF_(4, 2)
		/*
		KalmanFilter( int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F )
		"dynamParams = 4": 4*1 vector of state (x, y, delta x, delta y)
		"measureParams = 2": 2*1 vector of measurement (x, y)
		*/
        {
            measurement_ = Mat::zeros(2, 1, CV_32F);// (x, y)
            KF_.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0,//**Latter 1: Larger, faster regression
                                                         0, 1, 0, 1,//**Latter 1: Larger, faster regression
                                                         0, 0, 1, 0,
                                                         0, 0, 0, 1);
            setIdentity(KF_.measurementMatrix, Scalar::all(1));
            setIdentity(KF_.processNoiseCov, Scalar::all(1e-3));//**3: Larger, slower regression
            setIdentity(KF_.measurementNoiseCov, Scalar::all(1e-1));//1: Larger, quicker regression
            setIdentity(KF_.errorCovPost, Scalar::all(1));
 
            KF_.statePost = (Mat_<float>(4, 1) << x, y, 0, 0);//Ensure beginner is default value
        }
 
	Point2f run(float x, float y)
	{
		Mat prediction = KF_.predict();
		Point2f predict_pt = Point2f(prediction.at<float>(0),prediction.at<float>(1));
 
		measurement_.at<float>(0, 0) = x;
		measurement_.at<float>(1, 0) = y;
 
		KF_.correct(measurement_);
 
		return predict_pt;
	}
private:
	Mat measurement_;
	cv::KalmanFilter KF_;//Differ from Kalman_example::KalmanFilter
};



#endif