#ifndef CAMERA_H_
#define CAMERA_H_

#include <GL/gl.h>
#include <GL/glut.h>

#include <cv.h>
#include <cvaux.h>

#define PI 3.1415926535897932384626433832795
#define PIdiv180 (PI/180.0)

/**
 * @class Camera
 * @brief Implements a freeview camera
 * @details Code partially taken from: www.codecolony.de (Advanced CodeColony Camera, Philipp Crocoll, 2003)
 * @author Peter Kiechle
 */
class Camera {
private:
	cv::Mat positionVector; //camera's position

	//View- Right- and upVector constitute a base
	cv::Mat viewVector; //direction the camera looks at
	cv::Mat rightVector; //orthogonal to viewVector and upVector
	cv::Mat upVector; //orthogonal to viewVector and rightVector

	cv::Mat targetPosition; //position the camera looks at

	cv::Mat tmppositionVector;
	cv::Mat tmpviewVector;
	cv::Mat tmprightVector;
	cv::Mat tmpupVector;

	//frustum and camera internals
	int winWidth, winHeight, centerX, centerY; //window properties, updated in GLUTs reshape()


	float speedFactor;


public:
	double xrot; 			//pitch
	double yrot; 			//yaw
	double zrot; 			//roll
	float lastXrot;
	float lastYrot;

	Camera();

	void rotateX(double angle);	//pitch
	void rotateY(double angle);	//yaw
	void rotateZ(double angle);	//roll

	void setPosition(cv::Mat pos);
	void move(cv::Mat& direction);
	void moveX(double distance);
	void moveY(double distance);
	void moveZ(double distance);

	void setup(); //final step: executes gluLookAt()

	void normalize(cv::Mat& v);
};

#endif /* CAMERA_H_ */

