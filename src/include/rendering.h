#ifndef RENDERING_H_
#define RENDERING_H_


#include "SDL/SDL.h" //rendering with Simple DirectMedia Layer and OpenGL
#include "SDL/SDL_opengl.h"

#include <cv.h>
#include <cvaux.h>
#include <string>
#include <iostream>
#include "videodevice.h"
#include "camera.h"
#include "arcball.h"
#include "utils.h"
#include <cmath>

using namespace std;

typedef std::vector<cv::Point3f> Points3D; //list of world coordinates

/**
 * @class Renderer
 * @brief Renders an object based on the camera pose
 * @note Requires SDL
 * @author Peter Kiechle
 */
class Renderer {
private:

	float objectScalefactor; //object size
	float scaleFactor; //arbitrary scale factor of the whole scene
	float speedFactor; //movement
	float spin;
	std::vector<Points3D> worldCoordinates;
	std::vector<std::vector<float> > worldCoordinateColors;
	bool freeview;

	int drawWorldcoordinates;
	enum {
		ALL_WORLDCOORDINATES = 0,
		CURRENT_WORLDCOORDINATES = 1,
		NO_WORLDCOORDINATES = 2,
	};


	//select movement mode
	bool objectMovement;
	bool lightMovement;

	//free view camera
	bool cameraMoveLeft;
	bool cameraMoveRight;
	bool cameraMoveUp;
	bool cameraMoveDown;
	bool cameraMoveForward;
	bool cameraMoveBackward;

	//Object positioning
	bool objectMoveLeft;
	bool objectMoveRight;
	bool objectMoveUp;
	bool objectMoveDown;
	bool objectMoveForward;
	bool objectMoveBackward;
	bool objectEnlarge;
	bool objectShrink;
	cv::Point3f objectPosition;

	//light positioning
	bool lightMoveLeft;
	bool lightMoveRight;
	bool lightMoveUp;
	bool lightMoveDown;
	bool lightMoveForward;
	bool lightMoveBackward;
	float lightPosition[4];

	//SDL
	SDL_Event event; //SDL message handler
	int windowWidth, windowHeight;
	int videoWidth, videoHeight;

	//Mouse movement
	float mouseSensitivity;
	cv::Point2f mousePosition; //current mouse position
	cv::Point2f mousePressed; //mouse position during last *click*
	bool mouseLeftButtonDown;
	bool mouseRightButtonDown;
	bool mouseMiddleButtonDown;
	bool mouseDragging; //movement with pressed mouse button

	//arcball rotation
	Arcball* arcball;

	Camera* camera;
	double offset_x, offset_y; //shifted camera center
	double modelViewMatrixRecoveredCamera[16];
	double modelViewMatrixFreeViewCamera[16];

	//camera track
	std::vector<cv::Mat> rotations;
	std::vector<cv::Mat> translations;

	//shadow
	bool renderShadow;
	float shadowProjectionMatrix[4][4];
	cv::Mat planeVertex0;
	cv::Mat planeVertex1;
	cv::Mat planeVertex2;
	cv::Mat planeVertex3;
	cv::Mat planeVertex0_h;
	cv::Mat planeVertex1_h;
	cv::Mat planeVertex2_h;
	cv::Mat planeVertex3_h;


public:

	const char* caption;
	bool preview;
	uint requestedFrameNumber;

	Renderer(int w, int h, const char* title);
	virtual ~Renderer();


	/**
	 * @brief Initialize SDL and OpenGL context
	 * @return Initialization successful
	 */
	bool initRendering();


	/**
	 * @brief Defines the virtual camera's view frustum (projection matrix)
	 * @param K The Camera intrinsics
	 */
	void setViewFrustum(cv::Mat& K);


	/**
	 * @brief Sets the global scale factor
	 * @param scaleFactor The Scale factor
	 */
	void setScaleFactor(float scaleFactor);


	/**
	 * @brief Defines the object's position
	 * @param position World coordinate of the Teapot
	 * @param coordinates Set of triangulated world coordinates
	 */
	void setObjectPosition(cv::Point3f& position, std::vector<Points3D>& coordinates);


	/**
	 * @brief Setter for camera matrices
	 * @param r Set of camera rotations
	 * @param t Set of camera translations
	 */
	void setCameraTrack(std::vector<cv::Mat>& r, std::vector<cv::Mat>& t);


	/**
	 * @brief Sets the position and orientation of the virtual camera.
	 * @details To be more precise: It is the object and not the camera that is rotated and translated as the camera is fixed in OpenGL
	 * 			This function has to be called for each frame of the video sequence (with the corresponding camera transformations)
	 * @param R Rotation Matrix
	 * @param t Translation Vector
	 */
	void setCameraPose(cv::Mat& R, cv::Mat& t);


	/**
	 * @brief Sets the initial position and orientation of the Free View camera.
	 * @details To be more precise: It is the object and not the camera that is rotated and translated as the camera is fixed in OpenGL
	 * @param R Rotation Matrix
	 * @param t Translation Vector
	 * @param pos Updated camera position
	 * @param pitch Camera's pitch (rotation around x-axis)
	 * @param yaw Camera's pitch (rotation around y-axis)
	 * @param roll Camera's roll (rotation around z-axis)
	 */
	void setFreeViewCameraPose(cv::Mat& R, cv::Mat& t, cv::Mat& pos, float pitch, float yaw, float roll);


	/**
	 * @brief Renders Teapot ontop of the video frame
	 * @details Sets all objects of the scene (light, teapot, freeview camera, reconstructed camera) in place
	 *          Then renders the scene twice for the freeview camera and the reconstructed one
	 * @param background The corresponding frame of the video
	 */
	void render(cv::Mat& background);


	/**
	 * @brief SDL event handling
	 */
	void processInput();


	/**
	 * @brief Updates the Utah teapot's position and size
	 */
	void updateObject();


	/**
	 * @brief Updates the freeview camera's position and orientation
	 */
	void updateCamera();


	/**
	 * @brief Updates the light's position
	 */
	void updateLight();


	/**
	 * @brief Rotates the object in 3D according to the 2D-mouse movement
	 */
	void rotateObject();


	/**
	 * @brief Updates the camera's rotation
	 */
	void rotateCamera();


	/**
	 * @brief Draws a spinning cube representing the triangulated world points
	 * @param position The cube's position
	 * @param scale The cube's scale
	 */
	void drawSpinningCube(cv::Point3f position, float scale);


	/**
	 * @brief Draws a cube (predecessor of the teapot)
	 * @details Has to be translated scaled an rotated beforehand
	 * @param scale
	 */
	void drawCube(float scale);


	/**
	 * @brief Draws ground plate for orientation in freeview mode
	 * @param x Ground plate's center position x
	 * @param y Ground plate's center position y
	 * @param z Ground plate's center position z
	 * @param scale Ground plate's scale
	 */
	void drawGrid(float x, float y, float z, float scale);


	/**
	 * @brief Enable orthographic projection: i.e. Camera faces 2-dimensional Plane. Coordinate (0, 0) is bottom left
	 * @details See: http://www.lighthouse3d.com/opengl/glut/index.php3?bmpfontortho
	 */
	void setOrthographicProjection();


	/**
	 * @brief Back from orthographic projection to previous projection matrix
	 */
	void resetPerspectiveProjection();


	/**
	 * @brief Visualizes light source by drawing a sphere at lightPosition
	 */
	void drawLightSource();


	/**
	 * @brief Visualization of the camera track
	 * @details Draws a frustum for each keyframe camera
	 */
	void drawCameraTrack();


	/**
	 * @brief Defines a plane for the shadow
	 * @details Size of the plane depends on the scale factor of the scene
	 */
	void defineShadowPlane();


	/**
	 * @brief Computes shadow plane
	 * @details Given three points defining the plane (coordinate form)
	 *          a point in the plane and the corresponding normal is computed (normal form)
	 * @param v0 First vector
	 * @param v1 Second vector
	 * @param v2 Third vector
	 * @param shadowPlane The resulting plane
	 */
	void findShadowPlaneEquation(cv::Mat& v0, cv::Mat& v1, cv::Mat& v2, cv::Mat& shadowPlane);


	/**
	 * @brief Creates a matrix that projects the shadow
	 * @details y- and z- axis are reversed for known reasons...
	 * @details See: OpenGL Programming Guide ("Red Book") - Chapter 14 - Shadows
	 * @param groundplane
	 */
	void calculateShadowProjectionMatrix(cv::Mat& groundplane);


	/**
	 * @brief Draws the virtual floor where the shadow is mapped to
	 */
	void drawShadowPlane(void);


	/**
	 * @brief Renders teapot with shadow
	 * @details Shadow technique is based on shadow volumes using the stencil buffer of the fixed function pipeline
	 *          See: OpenGL Programming Guide ("Red Book") - Chapter 14 - Shadows
	 *          First, the teapot is rendered into the pixel buffer as usual.
	 *          Then a projection matrix is created that maps world space coordinates on a plane.
	 *          Each polygon of the teapot is then rendered into the stencil buffer using the defined projection matrix
	 *          (i.e. the camera plane is now in the shadow plane)
	 *          The stencil buffer is then blended with the previous pixel buffer
	 */
	void drawTeapotWithShadow();


	/**
	 * @brief Returns a vector of "nice" RGB values
	 * @details Generates "rainbow gradient" in HSV color space by varying hue values distributed in the golden ratio
	 *          Taken and modified from Alvy Ray Smith: http://www.alvyray.com/Papers/CG/hsv2rgb.htm
	 *          Hue: [0..6]
	 *          Saturation: [0..1]
	 *          Value: [0..1]
	 *          RGB: [0..1] each
	 * @param n length of the output vector
	 * @param colors Vector of Vector of R G B
	 */
	void getRandomColors(uint n, std::vector<std::vector<float> >& colors);


	/**
	 * @brief Returns a "random" float between min and max
	 * @param min Minimum number
	 * @param max Maximum number
	 * */
	float generate_random_float(float min, float max);
};

#endif /* RENDERING_H_ */
