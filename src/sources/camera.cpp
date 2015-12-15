#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include "camera.h"


Camera::Camera() {
	//window properties
	winWidth = 1200;
	winHeight = 500;
	centerX = winWidth * 0.5; //new center of the window
	centerY = winHeight * 0.5; //new center of the window


	//frustum properties
	xrot = 0.0; //pitch
	yrot = 0.0; //yaw
	zrot = 0.0; //roll
	lastXrot = xrot;
	lastYrot = yrot;

	//camera behaviour
	speedFactor = 2.0f;

	//coordinate system with standard OpenGL values:
	positionVector = (cv::Mat_<double>(3,1) <<  0.0, 0.0, 0.0);
	viewVector = (cv::Mat_<double>(3,1) <<  0.0, 0.0, -1.0);
	rightVector = (cv::Mat_<double>(3,1) <<  1.0, 0.0, 0.0);
	upVector =(cv::Mat_<double>(3,1) <<  0.0, 1.0, 0.0);
}


void Camera::move(cv::Mat& direction) {
	positionVector = positionVector + direction;
}

/**
 * @brief Pitch - rotate around rightVector (look up/down)
 */
void Camera::rotateX (double angle) {
	//X = rightVector, Y = upVector, Z = viewVector
	//X' = X
	//Z' = -sinθ.Y + cosθ.Z
	//Y'= X' ^ Z'

	//compute new view-vector
	viewVector = viewVector * cos(angle*PIdiv180) + upVector * sin(angle*PIdiv180);
	normalize(viewVector);
}

/**
 * Yaw - rotate around upVector (look left/right)
 */
void Camera::rotateY(double angle) {
	//X = rightVector, Y = upVector, Z = viewVector
    //Y' = Y
    //Z' = sinθ.X+cosθ.Z
    //X' = Z' ^ Y'

	//compute new view-vector
	viewVector = viewVector*cos(angle*PIdiv180) - rightVector*sin(angle*PIdiv180);
	normalize(viewVector);

	//compute the new right-vector by cross product
	rightVector = viewVector.cross(upVector);
}


/**
 * Roll (bend left/right)
 */
void Camera::rotateZ (double angle) {
	//Rotate viewVector around the right vector:
	rightVector = rightVector*cos(angle*PIdiv180) + upVector*sin(angle*PIdiv180);
	normalize(rightVector);
}

void Camera::setup() {

	//the positionVector the camera aims at
	targetPosition = positionVector + viewVector;

	//TODO: Problem when upVector is collinear to viewVector
	gluLookAt(positionVector.at<double>(0,0), positionVector.at<double>(1,0), positionVector.at<double>(2,0),
			  targetPosition.at<double>(0,0), targetPosition.at<double>(1,0), targetPosition.at<double>(2,0),
			  upVector.at<double>(0,0), upVector.at<double>(1,0), upVector.at<double>(2,0)); //(0,1,0) should work too


//	printf("positionVector: %f, %f, %f\n", positionVector.at<double>(0,0), positionVector.at<double>(1,0), positionVector.at<double>(2,0));
//	printf("upVector: %f, %f, %f\n", upVector.at<double>(0,0), upVector.at<double>(1,0), upVector.at<double>(2,0));
//	printf("viewVector: %f, %f, %f\n", viewVector.at<double>(0,0), viewVector.at<double>(1,0), viewVector.at<double>(2,0));
//	printf("rightVector: %f, %f, %f\n\n", rightVector.at<double>(0,0), rightVector.at<double>(1,0), rightVector.at<double>(2,0));

}


void Camera::moveX(double distance) {
	positionVector = positionVector + (rightVector *(distance * speedFactor));
}

void Camera::moveY(double distance) {
	positionVector = positionVector + (upVector * (distance * speedFactor));
}

void Camera::moveZ(double distance) {
	positionVector = positionVector + (viewVector * (distance * speedFactor));
}

void Camera::setPosition(cv::Mat pos) {
	positionVector = pos;
}


//normalize vector
void Camera::normalize(cv::Mat& v) {
	double magnitude = sqrt(v.at<double>(0,0) * v.at<double>(0,0) + v.at<double>(1,0) * v.at<double>(1,0) + v.at<double>(2,0) * v.at<double>(2,0));
	v.at<double>(0,0) /= magnitude,
	v.at<double>(1,0) /= magnitude,
	v.at<double>(2,0) /= magnitude;
}
