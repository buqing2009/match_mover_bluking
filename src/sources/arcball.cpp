
#include "arcball.h"

Arcball::Arcball() {

	pointOnSphereStart = cv::Mat_<float>(3,1);
	pointOnSphereEnd = cv::Mat_<float>(3,1);

	//initialize rotation matrices
	//first column
	transformationMatrixGL[0] = 1;
	transformationMatrixGL[1] = 0;
	transformationMatrixGL[2] = 0;
	transformationMatrixGL[3] = 0;
	//second column
	transformationMatrixGL[4] = 0;
	transformationMatrixGL[5] = 1;
	transformationMatrixGL[6] = 0;
	transformationMatrixGL[7] = 0;
	//third column
	transformationMatrixGL[8] = 0;
	transformationMatrixGL[9] = 0;
	transformationMatrixGL[10] = 1;
	transformationMatrixGL[11] = 0;
	//fourth column
	transformationMatrixGL[12] = 0;
	transformationMatrixGL[13] = 0;
	transformationMatrixGL[14] = 0;
	transformationMatrixGL[15] = 1;

	currentRotMat  = (cv::Mat_<float>(3,3) << 1,0,0,  0,1,0,  0,0,1 );
	previousRotMat = (cv::Mat_<float>(3,3) << 1,0,0,  0,1,0,  0,0,1 );
	transformationMatrix = (cv::Mat_<float>(4,4) << 1,0,0,0,  0,1,0,0,  0,0,1,0, 0,0,0,1 );

}

Arcball::~Arcball() {};

void Arcball::setArcballSize(int width, int height) {
	restrictedWidth = 1.0f / ((width - 1.0f) * 0.5f);
	restrictedHeight = 1.0f / ((height - 1.0f) * 0.5f);
}

void Arcball::computeSphereIntersection(const cv::Point2f& mousePosition, cv::Mat& pointOnSphere) {
	cv::Point2f tempMousePosition;
	float length; //length of the vector from the sphere's center to the mouse position in world coordinates

	tempMousePosition = mousePosition;

	//restrict mouse position to  [-1..1]
	tempMousePosition.x  =  (tempMousePosition.x * restrictedWidth) - 1.0;
	tempMousePosition.y  =  1.0f - (tempMousePosition.y * restrictedHeight); //invert y-axis

	//compute the square of the length of the vector from the sphere's center to the point on the surface
	length = (tempMousePosition.x * tempMousePosition.x) + (tempMousePosition.y * tempMousePosition.y);

	//mouse position is outside of the sphere...
	if (length > 1.0f) {
		//normalize vector from sphere's center to a point on the surface
		float norm = 1.0f / sqrt(length); //radius / sqrt(length)
		pointOnSphere.at<float>(0,0) = tempMousePosition.x * norm;
		pointOnSphere.at<float>(1,0) = tempMousePosition.y * norm;
		pointOnSphere.at<float>(2,0) = 0.0f;
	}
	else { //mouse position "inside" the sphere
		pointOnSphere.at<float>(0,0) = tempMousePosition.x;
		pointOnSphere.at<float>(1,0) = tempMousePosition.y;
		pointOnSphere.at<float>(2,0) = sqrt(1.0f - length); //sqrt(radius squared - length)
	}

	//normalize point on sphere
	cv::normalize(pointOnSphere, pointOnSphere);

	//get current model view matrix and extract rotation matrix
	double currentModelViewMatrix[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, currentModelViewMatrix);

	cv::Mat currentRotMat = cv::Mat_<float>(3,3);

	//column major order
	//right vector
	currentRotMat.at<float>(0,0) = currentModelViewMatrix[0];
	currentRotMat.at<float>(1,0) = currentModelViewMatrix[1];
	currentRotMat.at<float>(2,0) = currentModelViewMatrix[2];
	//up vector
	currentRotMat.at<float>(0,1) = currentModelViewMatrix[4];
	currentRotMat.at<float>(1,1) = currentModelViewMatrix[5];
	currentRotMat.at<float>(2,1) = currentModelViewMatrix[6];
	//forward vector
	currentRotMat.at<float>(0,2) = currentModelViewMatrix[8];
	currentRotMat.at<float>(1,2) = currentModelViewMatrix[9];
	currentRotMat.at<float>(2,2) = currentModelViewMatrix[10];

	//invert rotation matrix (could have also be done by extracting the elements in row major order)
	cv::transpose(currentRotMat, currentRotMat); //R^T = R^-1 due to orthogonality

	//apply rotation to point on sphere in order to take the camera orientation into account
	pointOnSphere = currentRotMat * pointOnSphere;
}


void Arcball::click(const cv::Point2f& mousePosition) {
	currentRotMat.copyTo(previousRotMat); //save current rotation state
	computeSphereIntersection(mousePosition, pointOnSphereStart); //map initial 2D mouse position onto sphere
}


void Arcball::drag(const cv::Point2f& mousePosition) {

	//map current 2D mouse position onto sphere
	computeSphereIntersection(mousePosition, pointOnSphereEnd);

    //compute axis vector that is perpendicular to the two points on the sphere
	cv::Mat axisVector = cv::Mat_<float>(3,1);
	axisVector = pointOnSphereStart.cross(pointOnSphereEnd); //cross product delivers a perpendicular vector - the arcball's rotation axis

	//cross product is > 0, i.e. both vectors differ from each other
	if (cv::norm(axisVector, cv::NORM_L2) > epsilon) {
		float rotationAngle = acos(pointOnSphereStart.dot(pointOnSphereEnd)); //cos^−1 of dotproduct = angle between two vectors
		cv::normalize(axisVector, axisVector);

		//create a rotation matrix out of the rotation axis and an angle
		buildRotationMatrix(axisVector, rotationAngle);

		//accumulate previous rotation matrix to current one
		currentRotMat = previousRotMat * currentRotMat;

		//create the final transformation matrix
		buildTransformationMatrix();

	} else { //cross product is zero, i.e. both vectors are parallel as the mouse was not moved in the meantime
		//noop
	}

}


void Arcball::buildRotationMatrix(cv::Mat& rotationAxis, float angle) {

	float c = cos(angle);
	float s = sin(angle);
	float t = 1-c;
	float x = rotationAxis.at<float>(0,0);
	float y = rotationAxis.at<float>(1,0);
	float z = rotationAxis.at<float>(2,0);

	//first row
	currentRotMat.at<float>(0,0) = t*x*x + c;
	currentRotMat.at<float>(0,1) = t*x*y - s*z;
	currentRotMat.at<float>(0,2) = t*x*z + s*y;

	//second row
	currentRotMat.at<float>(1,0) = t*x*y + s*z;
	currentRotMat.at<float>(1,1) = t*y*y + c;
	currentRotMat.at<float>(1,2) = t*y*z - s*x;

	//third row
	currentRotMat.at<float>(2,0) = t*x*z - s*y;
	currentRotMat.at<float>(2,1) = t*y*z + s*x;
	currentRotMat.at<float>(2,2) = t*z*z + c;
}


void Arcball::buildTransformationMatrix() {

	//OpenCV matrix
	transformationMatrix.at<float>(0,0) = currentRotMat.at<float>(0,0);
	transformationMatrix.at<float>(0,1) = currentRotMat.at<float>(0,1);
	transformationMatrix.at<float>(0,2) = currentRotMat.at<float>(0,2);
	transformationMatrix.at<float>(0,3) = 0.0;

	transformationMatrix.at<float>(1,0) = currentRotMat.at<float>(1,0);
	transformationMatrix.at<float>(1,1) = currentRotMat.at<float>(1,1);
	transformationMatrix.at<float>(1,2) = currentRotMat.at<float>(1,2);
	transformationMatrix.at<float>(1,3) = 0.0;

	transformationMatrix.at<float>(2,0) = currentRotMat.at<float>(2,0);
	transformationMatrix.at<float>(2,1) = currentRotMat.at<float>(2,1);
	transformationMatrix.at<float>(2,2) = currentRotMat.at<float>(2,2);
	transformationMatrix.at<float>(2,3) = 0.0;

	transformationMatrix.at<float>(3,0) = 0.0;
	transformationMatrix.at<float>(3,1) = 0.0;
	transformationMatrix.at<float>(3,2) = 0.0;
	transformationMatrix.at<float>(3,3) = 1.0;


	//column major order in OpenGL matrix
	transformationMatrixGL[0] = currentRotMat.at<float>(0,0);
	transformationMatrixGL[1] = currentRotMat.at<float>(1,0);
	transformationMatrixGL[2] = currentRotMat.at<float>(2,0);
	transformationMatrixGL[3] = 0.0;

	transformationMatrixGL[4] = currentRotMat.at<float>(0,1);
	transformationMatrixGL[5] = currentRotMat.at<float>(1,1);
	transformationMatrixGL[6] = currentRotMat.at<float>(2,1);
	transformationMatrixGL[7] = 0.0;

	transformationMatrixGL[8]  = currentRotMat.at<float>(0,2);
	transformationMatrixGL[9]  = currentRotMat.at<float>(1,2);
	transformationMatrixGL[10] = currentRotMat.at<float>(2,2);
	transformationMatrixGL[11] = 0.0;

	transformationMatrixGL[12] = 0.0;
	transformationMatrixGL[13] = 0.0;
	transformationMatrixGL[14] = 0.0;
	transformationMatrixGL[15] = 1.0;
}

