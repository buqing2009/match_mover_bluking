#ifndef MULTIVIEW_H_
#define MULTIVIEW_H_

#include <cv.h>
#include "params.h"
#include "stereoview.h"


using namespace std;

class PointCorrespondences;
class ThreeWay;
class Vertex;

/**
 * @class Multiview
 * @brief Handles the set of consecutive video frames
 * @note This class implements the alternative tracking approach with Optical Flow between the Keyframes
 * @author Peter Kiechle
 */
class Multiview {

private:
	std::string inputBasename;  //file prefix of input video
	std::string inputExtension;  //file extension of input video
	VideoIO* video;
	cv::VideoWriter trackRecorder;
	cv::Mat frame0; ///first frame
	cv::Mat frame1; ///second frame
	int width, height;

	bool drawTracks;
	bool drawThreeWayTracks;
	int minPointCorrespondences; //minimum number of tracks between frames used for triangulation
	int keyFrameLimit;
	int frameLimit;
	int maxOpticalFlowLength;
	double T_min, T_max; //threshold for correspondence ratio
	float maxDrift; //allowed drift of pixels when arriving in next keyframe (compared to the already known coordinates)
	Renderer* renderer;

public:
	Multiview();
	virtual ~Multiview();

	/**
	 * @brief Test the renderer without opening the video
	 */
	void workflow_rendertest();


	/**
	 * @brief Entry function: This is where the work is done
	 */
	void workflow();


	/**
	 * @brief Combines two sets of point correspondences A <-> B  and B <-> C to a three-way point correspondence A <-> B <-> C
	 * @param previousPc Previous point correspondences
	 * @param currentPc The current point correspondences
	 * @param resultingThreeWay Combined point correspondences
	 * @return The number of three-way point-correspondences from A to B to C
	 */
	int  findThreeWayCorrespondences(PointCorrespondences& previousPc, PointCorrespondences& currentPc, ThreeWay& resultingThreeWay);


	/**
	 * @brief Triangulate between first two cameras A and B of a three-way point correspondence
	 * @param threeWayPc The three-way point correspondence to triangulate from
	 */
	void triangulateThreeWayPc(ThreeWay& threeWayPc);


	/**
	 * @brief Selects next keyframe
	 * @details Keyframe with maximal weighted GRIC PELC Score will be next keyframe
	 * @param lastKeyframe
	 * @param GRICDifferenceCriterion
	 * @param weightedCriterion
	 * @return
	 */
	int selectKeyframe(int lastKeyframe, std::vector<double>& GRICDifferenceCriterion, std::vector<double>& weightedCriterion);


	/**
	 * @brief Finds point correspondeces between the keyframes A, B and C of a three-way point correspondence
	 * @details Tracks feature points across a pair of keyframes using Lucas-Kanade optical flow.
	 *          Refines image coordinates if tracking of a point from keyframe A to B to C is lost on the way
	 * @param threeWay The three-way point correspondence
	 */
	void findInterKeyframePointCorrespondences(ThreeWay& threeWay);


	/**
	 * @brief Given world- and corresponding image coordinates, gain camera pose of intermediate frames by resectioning
	 * @param threeWay The
	 * @param K Intrinsic Camera Matrix
	 * @param OpenGLRotations Rotation Matrix for OpenGL
	 * @param OpenGLTranslations TRanslation vector for OpenGL
	 * @param isLast In case of the last three-way point correspondence, cameras between keyframes B->C are reconstructed as well
	 */
	void determineInterKeyframeCameras(ThreeWay& threeWay, cv::Mat& K, std::vector<cv::Mat>& OpenGLRotations, std::vector<cv::Mat>& OpenGLTranslations, bool isLast);


	/**
	 * @brief Import K and distortion coefficients from file
	 * @param filename The filename
	 * @param K The imported Intrinsic camera matrix
	 * @param distCoeffs The imported distortion coefficients
	 */
	void readCameraInstrinics(const string& filename, cv::Mat& K, cv::Mat& distCoeffs);


	/**
	 * @brief Determines the position of the rendered object in world coordinates
	 * @param worldCoordinates The set of all world coordinates from all cameras
	 * @param objectPosition The Utah Teapot's final position
	 */
	void determineObjectPosition(std::vector<Points3D> worldCoordinates, cv::Point3f& objectPosition);


	/**
	 * @brief Loops through keyframes and renders the reconsructed scene
	 * @param rotations Set of keyframe camera rotations
	 * @param translations Set of keyframe camera translations
	 * @param frameIndices Indices to keyframes of the video
	 */
	void showPreview(std::vector<cv::Mat>& rotations, std::vector<cv::Mat>& translations, std::vector<int>& frameIndices);


	/**
	 * @brief Superimposes rendered model on video frame
	 * @param R Rotation Matrix
	 * @param t Translation Vector
	 * @param image The background image for the OpenGL-context (ortho view without depth test)
	 */
	void renderObject(cv::Mat& R, cv::Mat& t, cv::Mat& image);


	/**
	 * @brief Loops through all frames of the video and calls the renderer to draw the teapot on it
	 * @param OpenGLRotations All rotations for the OpenGL renderer
	 * @param OpenGLTranslations All Translations for the OpenGL renderer
	 */
	void renderSequence(std::vector<cv::Mat>& OpenGLRotations, std::vector<cv::Mat>& OpenGLTranslations);


	/**
	 * @brief Visualizes tracked point of a three-way point correspondence
	 * @param startframe Index of the first frame in the sequence
	 * @param currentPosition The current position in the video
	 * @param currentFrame The current Frame
	 * @param imageCoordinatesLK All image coordinates of the Threeway Point correspondence
	 * @param twpcIdentifier Three-way identifier (A, B, C)
	 * @param label Should tracks be labeled?
	 */
	void drawTrackedPoints(uint startframe, uint currentPosition, cv::Mat& currentFrame, std::vector<Points2D> imageCoordinatesLK, const std::string& twpcIdentifier, bool label);


	/**
	 * @brief Visualize image points in three-way point correspondence
	 * @param threeWay The three-way point correspondence
	 * @param label label Should tracks be labeled?
	 */
	void drawThreeWayPointCorrespondence(ThreeWay& threeWay, bool label);


	/**
	 * @brief Output camera matrices
	 * @param keyframeThreeWayPointCorrespondences Set of three-way point correspondences
	 * @param keyFrameRotations Rotation matrices of the keyframes
	 * @param keyFrameTranslations Translation vectors of the keyframes
	 * @param keyFrameIndices Indices to keyframes of the video
	 */
	void printReconstructedCameras(std::vector<ThreeWay>& keyframeThreeWayPointCorrespondences, std::vector<cv::Mat>& keyFrameRotations, std::vector<cv::Mat>& keyFrameTranslations, std::vector<int>& keyFrameIndices);


	/**
	 * @brief Place a self-made wireframe cube into scene
	 * @details
	 *
	 *
	 * World Coordinate System:
	 *
	 *        ^ Z (into screen)
	 *       /
	 *      /
	 *     +-------> X
	 *     |
	 *     |
	 *     |
	 *   Y v
	 *
	 * @param P The Camera Projection Matrix
	 * @param image The image to draw the cube on
	 * @param X The position of the correspondence points in world coordinates
	 */
	void drawCube(cv::Mat& P, Points3D& X, cv::Mat& image);


	/**
	 * @brief Draws rectangles around triangulated keypoints
	 * @details Similar to drawReprojectionError but with a point size indicating the distance of the point from the camera
	 * @param P Camera projection Matrix
	 * @param X World Coordinate
	 * @param image The Image
	 * @param scale Size of the Worldpoint
	 */
	void drawTriangulatedWorldPoints(cv::Mat& P, Points3D& X, cv::Mat& image, float scale);

};


/**
 * @class PointCorrespondences
 * @brief Data structure for point correspondences between two frames including image and world coordinates
 */
class PointCorrespondences {

private:
	int frame0, frame1;
	std::vector<cv::KeyPoint> keypoints0, keypoints1; //detected feature points
	Points2D imagecoordinates0, imagecoordinates1; //center of keypoints
	Points3D worldCoordinates; //image coordinates in 3D
	Matches matches; //point correspondences with the consecutive frame

public:
	PointCorrespondences() {}
	PointCorrespondences(int frameA, int frameB) {frame0 = frameA; frame1 = frameB;}
	std::vector<cv::KeyPoint>& getKeypoints0() { return keypoints0; }
	std::vector<cv::KeyPoint>& getKeypoints1() { return keypoints1; }
	Points2D& getImageCoordinates0() { return imagecoordinates0; }
	Points2D& getImageCoordinates1() { return imagecoordinates1; }
	int getFrameNumber0() {return frame0;}
	int getFrameNumber1() {return frame1;}
	Points3D& getWorldCoordinates() { return worldCoordinates; }
	Matches& getMatches() { return matches; }
	cv::DMatch& getMatch(int index) { return matches[index]; }

	void setKeypoints0(std::vector<cv::KeyPoint> keypoints) { keypoints0 = keypoints; } //copy
};


/**
 * @class ThreeWay
 * @brief Holds information of a set of three keyframes and two intermediate sets of frames
 *        including the image coordinates of the features and the corresponding world coordinates
 *        as well as the Camera projection matrices
 * @author Peter Kiechle
 */
class ThreeWay {
	std::vector<int> frames;
	std::vector<Points2D> imageCoordinates; //image coordinates in frame A, B and C
	Points3D worldCoordinates; //shared world coordinates
	std::vector<cv::Mat> projectionMatrix;
	std::vector<Points2D> interKeyframeImageCoordinates0; //image coordinates shared by all frames between frame A, B (not including A and B)
	std::vector<Points2D> interKeyframeImageCoordinates1; //image coordinates shared by all frames between frame B, C (not including B and C)

public:
	ThreeWay() {}
	ThreeWay(int A, int B, Points2D& coordinatesA, Points2D& coordinatesB) {
		addCorrespondence(A, coordinatesA, B, coordinatesB);
	}

	int getStartFrame() { return frames[0]; }
	int getMiddleFrame() { return frames[1]; }
	int getEndFrame() { return frames[2]; }

	Points3D& getWorldCoordinates() {
		if(worldCoordinates.size() == 0) {
			cerr << "Error, no world coordinates available!" << endl;
		}
		return worldCoordinates;
	}

	std::vector<Points2D>& getInterKeyframeImageCoordinates0() {
		if(interKeyframeImageCoordinates0.size() == 0) {
			cerr << "Error, no inter-Keyframe image coordinates0 available!" << endl;
		}
		return interKeyframeImageCoordinates0;
	}

	Points2D& getInterKeyframeImageCoordinates0(uint i) {
		if(interKeyframeImageCoordinates0.size() <= i ) {
			cerr << "Error, specified inter-Keyframe image coordinates0 not available!" << endl;
		}
		return interKeyframeImageCoordinates0.at(i);
	}

	std::vector<Points2D>& getInterKeyframeImageCoordinates1() {
		if(interKeyframeImageCoordinates1.size() == 0) {
			cerr << "Error, no inter-Keyframe image coordinates1 available!" << endl;
		}
		return interKeyframeImageCoordinates1;
	}

	Points2D& getInterKeyframeImageCoordinates1(uint i) {
		if(interKeyframeImageCoordinates1.size() <= i ) {
			cerr << "Error, specified inter-Keyframe image coordinates1 not available! i="<< i << endl;
		}
		return interKeyframeImageCoordinates1.at(i);
	}

	Points2D& getImageCoordinatesA() {
		if(imageCoordinates.size() < 1) {
			cerr << "Error, no image coordinates A available!" << endl;
		}
		return imageCoordinates[0];
	}

	Points2D& getImageCoordinatesB() {
		if(imageCoordinates.size() < 2) {
			cerr << "Error, no image coordinates B available!" << endl;
		}
		return imageCoordinates[1];
	}

	Points2D& getImageCoordinatesC() {
		if(imageCoordinates.size() < 3) {
			cerr << "Error, no image coordinates C available!" << endl;
		}
		return imageCoordinates[2];
	}

	Points2D& getImageCoordinates(uint i) {
		if(imageCoordinates.size() <= i) {
			cerr << "Error, image coordinates not available!" << endl;
		}
		return imageCoordinates[i];
	}

	cv::Mat& getCamera(uint i) {
		if(projectionMatrix.size() <= i) {
			cerr << "Error, camera projection matrix "<< i << " not available!" << endl;
		}
		return projectionMatrix[i];
	}

	void setCamera(uint index, cv::Mat& c) {
		projectionMatrix.at(index) = c;
	}

	void setKeyframeImageCoordinates(uint i, Points2D& c) {
		if(imageCoordinates.size() <= i) {
			cerr << "Error, Keyframe Image Coordinates not available!" << endl;
		}
		imageCoordinates[i] = c;
	}

	void setInterKeyframeImageCoordinates0(std::vector<Points2D>& c) {
		interKeyframeImageCoordinates0 = c;
	}

	void setInterKeyframeImageCoordinates0(uint i, Points2D& c) {
		if(interKeyframeImageCoordinates0.size() <= i) {
			cerr << "Error, inter-Keyframe Image Coordinates0 not available!" << endl;
		}
		interKeyframeImageCoordinates0[i] = c;
	}

	void setInterKeyframeImageCoordinates1(std::vector<Points2D>& c) {
		interKeyframeImageCoordinates1 = c;
	}

	void setInterKeyframeImageCoordinates1(uint i, Points2D& c) {
		if(interKeyframeImageCoordinates1.size() <= i) {
			cerr << "Error, inter-Keyframe Image Coordinates1 not available!" << endl;
		}
		interKeyframeImageCoordinates1[i] = c;
	}

	void setWorldCoordinates(Points3D& c) {
		worldCoordinates = c;
	}

	void addImageCoordinates(Points2D& c) {
		imageCoordinates.push_back(c);
	}

	void addInterKeyframeImageCoordinates0(Points2D& c) {
		interKeyframeImageCoordinates0.push_back(c);
	}
	void addInterKeyframeImageCoordinates1(Points2D& c) {
		interKeyframeImageCoordinates1.push_back(c);
	}

	void addCamera(cv::Mat& c) {
		projectionMatrix.push_back(c);
	}

	void addCorrespondence(int frameNumber, Points2D& coordinates) {
		frames.push_back(frameNumber);
		imageCoordinates.push_back(coordinates);
	}

	void addCorrespondence(int frameNumber0, Points2D& coordinates0, int frameNumber1, Points2D& coordinates1) {
		frames.push_back(frameNumber0);
		frames.push_back(frameNumber1);
		imageCoordinates.push_back(coordinates0);
		imageCoordinates.push_back(coordinates1);
	}
};


#endif /* MULTIVIEW_H_ */
