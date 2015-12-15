#ifndef ARCBALL_H_
#define ARCBALL_H_

#include <cv.h>
#include <cvaux.h>
#include <stdio.h>
#include <GL/gl.h>

using namespace std;

#define PI 3.1415926535897932384626433832795
#define epsilon 1.0e-13

/**
 * @class Arcball
 * @brief Rotate the object with the mouse
 * @note See "ARCBALL: A User Interface for Specifying Three-Dimensional Orientation Using a Mouse" by Ken Shoemake, In Proceedings of Graphics Interface '92
 * @author
 *
 */
class Arcball {
private:
	cv::Mat pointOnSphereStart; //initial point on the arcball
	cv::Mat pointOnSphereEnd; //point on the arcball while mouse-dragging
 	cv::Mat currentRotMat; //separate rotation matrices for accumulating consecutive rotations
	cv::Mat previousRotMat;
	cv::Mat transformationMatrix; //OpenCV matrix for convenient matrix multiplication
	float transformationMatrixGL[16]; //same as transformationMatrix, but compatible with OpenGL's glLoadMatrix()

	//unit size arcball is based on window resolution
	float restrictedWidth;
	float restrictedHeight;

public:

	Arcball();
	virtual ~Arcball();


	/**
	 * @brief Adjust Arcball size
	 * @details The Arcball uses the whole screen as a sensitive area for mouse movements.
	 * 			The Arcball's size has to be updated whenever the screen size changes.
	 * @param width The window width
	 * @param height The window height
	 */
	void setArcballSize(int width, int height);


	/**
	 * @brief Map 2D mouse position on the surface of a unit sphere
	 * @details The original paper by Shoemak assumes a static camera with a default orientation determined by the identity rotation matrix.
				In order to get view-independent points on the sphere, they have to be transformed by the inverse rotation matrix of the current camera transformation.
	 * @param mousePosition The input device position (x,y) in screen-coordinates
	 * @param pointOnSphere World space position of the input device mapped on a unit sphere
	 */
	void computeSphereIntersection(const cv::Point2f& mousePosition, cv::Mat& pointOnSphere);


	/**
	 * @brief Initial Arcball action when *click*
	 * @details Stores current rotation matrix and maps the 2D mouse position to the surface of the arcball\n
	 *          The saved rotation matrix can later be applied to the new rotation matrix for consecutive rotations
	 * @param mousePosition Mouse position on the window
	 */
	void click(const cv::Point2f& mousePosition);


	/**
	 * @brief Generates a up-to-date transformation matrix for the teapot based on the orientation of the arcball
	 * @details Maps the current 2D mouse position to the surface of the arcball.\n
	 *          From the difference to the initial mouse position to the current one, a transformation matrix is generated,\n
	 *          that describes the rotation of the arcball.
	 * @param mousePosition Mouse position on the window
	 */
	void drag(const cv::Point2f& mousePosition);


	/**
	 * @brief Creates a 3x3 rotation matrix for the teapot
	 * @details Given the normalized axis and the rotation angle, builds the rotation matrix.\n
	 *          See: Graphics Gems - Andrew S. Glassner, Academic Press, 1990
	 * @param rotationAxis Arcball rotation-axis vector
	 * @param angle The rotation angle
	 */
	void buildRotationMatrix(cv::Mat& rotationAxis, float angle);


	/**
	 * @brief Creates a 4x4 transformation matrix for the teapot
	 * @details Given the rotation matrix, sets up transformation matrices
	 */
	void buildTransformationMatrix();

	/**
	 * @brief Getter method
	 * @return The Transformation Matrix for internal use
	 */
	cv::Mat& getTansformationMatrix() { return transformationMatrix; }


	/**
	 * @brief Getter method
	 * @return The Transformation Matrix for OpenGL
	 */
	float* getTansformationMatrixGL() { return transformationMatrixGL; }
};


#endif /* ARCBALL_H_ */
