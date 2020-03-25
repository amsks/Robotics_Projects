#include "cv_control.h"
#include "opencv2/opencv.hpp"


using namespace cv;
using namespace std;

// Parameters for Visual Servoing
float f;					// Focal length in pixel
float img_width;			// width of image sensor in pixel
float img_height;			// height of image sensor in pixel
float diameter_real;		// real diameter of the circle
float diameter_desired_px;	// desired diameter of the circle in pixels
float dt;					// Time step
float t0;                  // how fast the velocity controller should converge
int   hcd_min_distance;
double reachable_radius = 0.85; //the maximum reachable radius for the robot, to avoid singularity -- around the base frame
PrVector3 desired_s_opencvf;


void initVisualServoing(float _f, float _img_width, float _img_height, float _diameter_real, float _diameter_desired_px, float _dt, PrVector3 _desired_s_opencvf)
{
    f = _f;
    img_width = _img_width;
    img_height = _img_height;
    diameter_real = _diameter_real;
    diameter_desired_px = _diameter_desired_px;
    dt = _dt;
    desired_s_opencvf = _desired_s_opencvf;
    t0 = 0.3;   //time constant for controller convergence. Smaller values result in higher velocities. Transforms an error into a velocity
    hcd_min_distance = 2000;
}

/*****************************************************************************************************************/
/* YOUR WORK STARTS HERE!!! */

/** 
* findCircleFeature
* Find circles in the image using the OpenCV Hough Circle detector
*
* Input parameters:
*  img: the camera image, you can print text in it with 
* 	    putText(img,"Hello World",cvPoint(0,12),FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(0,0,255))
*	    see http://opencv.willowgarage.com/documentation/cpp/drawing_functions.html#cv-puttext
*
*  backproject: grayscale image with high values where the color of the image is like the selected color.
*
* Output:
*  crcl: as a result of this function you should write the center and radius of the detected circle into crcl
*/
bool findCircleFeature(Mat& img, Mat &backproject, Circle& crcl)
{
    //IMPLEMENT THIS
    // @Aditya: We need to tweak the 200, and 100 parameters for thresholds

    vector<Vec3f> circles;

    try {

        cv::HoughCircles(backproject, circles, CV_HOUGH_GRADIENT,1 , backproject.rows/8, 200, 100, 0, 0);
        double r = 0;
        for(int i=0; i < circles.size(); i++){
            if(circles[i][2] > r){
                r = circles[i][2];
                crcl.center = {circles[i][0], circles[i][1]};
                crcl.radius= r;
            }
        }
        return true ;
    } catch (...){

        return false ;
    }

    // @Aditya : The crcl variable stores the number of circles. Hence, if we find one circle then it should ret0urn true, otherwise false

}

/**
* getImageJacobianCFToFF
* Compute the Image Jacobian to map from 
* camera velocity in Camera Frame to feature velocities in Feature Frame
* 
* You should use getImageJacobianFFToCF in controlRobot
*
* Input parameters:
*  u and v: the current center of the circle in feature frame [pixels]
*  z: the estimated depth of the circle [meters]
*  f: the focal length [pixels]
*  diameter: the real diameter of the circle [meters]
*
* Output:
*  Jv: assign your image 3x3 Jacobian.
*/
void getImageJacobianCFToFF(PrMatrix3 &Jv, float u, float v, float z, float f, float diameter)
{

  //For the implementation we use diameter instead of radius everywhere, es the Hugh-transform outputs that value

  //IMPLEMENT THIS
  PrMatrix3 Im_Jacobian;

  Im_Jacobian[0][0] = - f / z ;
  Im_Jacobian[0][1] = Im_Jacobian[1][0] = Im_Jacobian[2][0] = Im_Jacobian[2][1] = 0.;
  Im_Jacobian[0][2] = u / z;
  Im_Jacobian[1][1] = -f / z;
  Im_Jacobian[1][2] = v / z;
  Im_Jacobian[2][2] = diameter / z; //

  Jv = Im_Jacobian;

}

/**
* estimateCircleDepth
* Estimates and returns the depth of the circle
*
* Input parameters:
*  f: the focal length [pixels]
*  diameter: the real diameter of the circle [meters]
*  crcl: the parameters of the detected circle in the image
*
* Output return:
*  depth of the circle wrt the camera [meters]
*/
float estimateCircleDepth(float f, float diameter, Circle &crcl)
{
    //IMPLEMENT THIS
    /* @Aditya :-
      f - focal length
      crcl - circle structure
      depth - depth
    */

    float depth;

    depth = (diameter / 2) * f / crcl.radius;  // @Aditya : z = X_x *f / S_u from optics

    return depth;
}

/**
* transformFromOpenCVFToFF
* Transform a feature vector from openCV frame (origin in upper left corner of the image) to feature frame (origin at the center of the image)
*
* Input parameter:
*  vector_opencvf: feature vector defined in opencv frame
*
* Output:
*  vector_ff: feature vector defined in feature frame
*/
void transformFromOpenCVFToFF(PrVector3 vector_opencvf, PrVector3& vector_ff) 
{

    //IMPLEMENT THIS
    // @Aditya: Convert the Image features obtained in the Find cricle function to feature frame
    vector_ff = desired_s_opencvf - vector_opencvf;
}

/**
* transformVelocityFromCFToEEF
* Transform the desired velocity vector from camera frame to end-effector frame
* You can hard code this transformation according to the fixed transformation between the camera and the end effector
* (see the sketch in your assignment)
*
* Input parameter:
*  vector_cf: velocity vector defined in camera frame
*
* Output:
*  vector_eef: velocity vector defined in end-effector frame
*/
void transformVelocityFromCFToEEF(PrVector3 vector_cf, PrVector3& vector_eef)
{
  //IMPLEMENT THIS
  //Here we transform te velocities from the camera to the end effector space, by rotating the frames by -90 degrees around Z axis
  //as seen on the assigment setup figure

  PrMatrix3 rot_mat;
  double angle = -90 * M_PI / 180;
  rot_mat[0][0] = cos(angle);
  rot_mat[0][1] = -sin(angle);
  rot_mat[0][2] = 0;
  rot_mat[1][0] = sin(angle);
  rot_mat[1][1] = cos(angle);
  rot_mat[1][2] = 0;
  rot_mat[2][0] = 0;
  rot_mat[2][1] = 0;
  rot_mat[2][2] = 1;

  vector_eef = rot_mat * vector_cf;

}

/**
* transformVelocityFromEEFToBF
* Transform the desired velocity vector from end-effector frame to base frame
* You cannot hard code this transformation because it depends of the current orientation of the end-effector wrt the base
* Make use of the current state of the robot x (the pose of the end-effector in base frame coordinates)
*
* Input parameters:
*  x_current_bf: current state of the robot - pose of the end-effector in base frame coordinates
*  vector_eef: velocity vector defined in end-effector frame
*
* Output:
*  vector_bf: velocity vector defined in base frame
*/
void transformVelocityFromEEFToBF(PrVector x_current_bf, PrVector3 vector_eef, PrVector3& vector_bf)
{
  //IMPLEMENT THIS
    /*The current pose is represented by the PrVector x_current_bf, where the first 3 elements are the translations,
    4. element is the quaternion angle, and the last 3 elements are the quaternion axes. OpenCv doesn't support quaternions, but PrQuaternion is implemented in the headers.
     will initialize it with -  PrQuaternion( Float scalar, Float vector0, Float vector1, Float vector2 );
     for rotating the vector we convert the quaternion to a rotation matrix and then multiply the velocity vector with it. Suggested by:
     https://www.astro.rug.nl/software/kapteyn-beta/_downloads/attitude.pdf - pg.14

     V2 - There is a function in PRQuaternion that converts automatically
    */

    PrQuaternion state_quat(x_current_bf[3], x_current_bf[4],x_current_bf[5],x_current_bf[6]);

    //First we need to normalize the quaternion to be sure the inverse calculation can be made
    state_quat.normalize();

    //We need to inverse the quaternion as we would like to go from the end effector to the Base frame and not the other way around.
    //We could also inverse the rotation matrix, but these are basically the same
    PrQuaternion inverse_quat;
    state_quat.inverse(inverse_quat);
    PrMatrix3 state_rotation = inverse_quat.matrix();

    vector_bf = state_rotation * vector_eef;
}

/*
* controlRobot
* This function computes the command to be send to the robot using Visual Servoing so that the robot tracks the circle
*
* Here you should:
* - compute the error in feature frame
* - compute the circle depth
* - compute the image jacobian from feature frame in camera frame
* - compute the desired ee velocity in feature frame
* - compute the desired ee velocity in camera frame
* - compute the desired ee velocity in ee frame
* - compute the desired ee velocity in base frame
* - compute the step in the direction of the desired ee velocity in base frame
* - form the comand to be sent to the robot (previous pose + computed step)
*
* The function will only be called if findCircleFeature returns true (if a circle is detected in the image)
*
* Input parameters:
*  crcl: the parameters of the detected circle in the image
*  x:	current robot configuration in operational space (7 dof: 3 first values are position, 4 last values is orientation quaternion)
*  img: the camera image for drawing debug text
*
* Output:
*  cmdbuf: should contain the command for the robot controler, for example: 
*			"goto 0.0 0.0 90.0 0.0 0.0 0.0"
*/

void controlRobot(Circle& crcl, PrVector &x, Mat& img, char *cmdbuf)
{
    if (crcl.radius == 0) {
        sprintf(cmdbuf,"float");
        return;
    }

    PrVector3 current_s_opencvf;
    current_s_opencvf[0] = crcl.center.x;
    current_s_opencvf[1] = crcl.center.y;
    current_s_opencvf[2] = 2 * crcl.radius;

    PrVector3 desired_s_ff;
    transformFromOpenCVFToFF(desired_s_opencvf, desired_s_ff);
    PrVector3 current_s_ff;
    transformFromOpenCVFToFF(current_s_opencvf, current_s_ff);

    PrVector3 error_s_ff = desired_s_ff - current_s_ff;

    float z = estimateCircleDepth(f, diameter_real, crcl);

    PrMatrix3 Jv;
    getImageJacobianCFToFF(Jv, current_s_ff[0], current_s_ff[1], z, f, diameter_real);

    PrMatrix3 Jv_inv;
    Jv.pseudoInverse(Jv_inv);

    //Compute the desired velocity of the feature in feature frame
    PrVector3 vel_f_ff = error_s_ff / t0;

    //Compute the desired velocity of the end effector in camera frame
    PrVector3 vel_ee_cf = Jv_inv*vel_f_ff;

    PrVector3 vel_ee_eef;
    transformVelocityFromCFToEEF(vel_ee_cf, vel_ee_eef);

    PrVector3 vel_ee_bf;
    transformVelocityFromEEFToBF(x, vel_ee_eef, vel_ee_bf);

    // compute the next EE position for the next timestep given the desired EE velocity:
    PrVector3 step_ee_bf = vel_ee_bf * dt;

    PrVector desired_ee_pose_bf = x;
    desired_ee_pose_bf[0] += step_ee_bf[0];
    desired_ee_pose_bf[1] += step_ee_bf[1];
    desired_ee_pose_bf[2] += step_ee_bf[2];

    //Command the robot to go to the new desired position if its inside the radius
    if(desired_ee_pose_bf.abs() < reachable_radius){
        sprintf(cmdbuf,"goto %.4f %.4f %.4f %.4f %.4f %.4f %.4f", desired_ee_pose_bf[0], desired_ee_pose_bf[1], desired_ee_pose_bf[2], 0.50, 0.50, -0.50, 0.50);
    } else {
        //We reduce the summed up desired pose to the previous value, this way for the next iteration it could move again
        desired_ee_pose_bf[0] -= step_ee_bf[0];
        desired_ee_pose_bf[1] -= step_ee_bf[1];
        desired_ee_pose_bf[2] -= step_ee_bf[2];
    }


    putText(img,cmdbuf, cv::Point(5,50), FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,255,0), 1.2);
}

