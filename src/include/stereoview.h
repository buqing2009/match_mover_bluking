
#ifndef STEREORECONSTRUCTION_H_
#define STEREORECONSTRUCTION_H_

#include <cv.h>
#include <cvaux.h>
#include <string>
#include <vector>
#include <iostream>
#include <opencv2/nonfree/features2d.hpp>
#include "params.h"
#include "utils.h"
#include "rendering.h"

using namespace std;

typedef std::vector<cv::DMatch> Matches;
typedef std::vector<cv::Point2f> Points2D; //list of image coordinates
typedef std::vector<cv::Point3f> Points3D; //list of world coordinates

/**
 @class Stereo
 @brief Performs a 3D-reconstruction from a pair of images
 @author Peter Kiechle
 */
class Stereo {

private:
    cv::Ptr<cv::FeatureDetector> featurePointDetector; //OpenCV interface for feature detection
	cv::Ptr<cv::DescriptorExtractor> featurePointDescriptor; //OpenCV interface for feature description
	cv::Ptr<cv::DescriptorMatcher> matcher; //OpenCV interface for feature matching

	std::vector<uchar> status; //status of tracked features
	std::vector<float> err; //error in tracking

	double matchUniquenessRatio; //degree of "uniqueness" of a point correspondence (Lowe ratio)
	double maxDistance; //maximum distance from a point to an epipolar line in pixels (findFundamentalMatrix)
	double confidenceLevel; //desirable level of confidence in (RANSAC)
	double maxReprojectionError;  //maximum allowed reprojection error to treat a point pair as an inlier (findHomography)

public:

	//used feature descriptors
	enum {
		SURF = 0,
		SIFT = 1,
		FAST = 2,
		GoodFeaturesToTrack = 3,
		ORB = 4
	};
	int featurePointDetectorType;

	//matcher properties
	enum {
		ONEWAY = 0,
		INTERSECTION = 1,
		UNION = 2,
	};
	int matcherPolicy;




	Stereo();
	virtual ~Stereo();


	/**
	 * @brief Detects and describes feature points of an image using OpenCV's FeatureDetector and a suitable DescriptorExtractor
	 * @details
	 * @param image The input image
	 * @param keypoints Resulting keypoints
	 * @param descriptors Resulting keypoint descriptors
	 * @return Number of found features
	 */
	int detectAndDescribeInterestPoints(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);


	/**
	 * @brief Finds corresponding interest points in a pair of images
	 * @details
	 * 		(see paper "Distinctive Image Features from Scale-Invariant Keypoints", David G. Lowe)
	 * @param image0 First image
	 * @param image1 Second image
	 * @param matches Set of corresponding points
	 * @param descriptors0 Feature point descriptor of the first image
	 * @param descriptors1 Feature point descriptor of the second image
	 * @param keypoints0 Feature points in the first image
	 * @param keypoints1 Feature points in the second image
	 * @return Number of found correspondence pairs
	 */
	int findCorrespondingInterestPoints(const cv::Mat& image0, const cv::Mat& image1, Matches& matches, cv::Mat& descriptors0, cv::Mat& descriptors1, std::vector<cv::KeyPoint>& keypoints0, std::vector<cv::KeyPoint>& keypoints1);


	/**
	 * @brief Filters matches based on the degree of "uniqueness" of a point correspondence (Lowe ratio)
	 * @details
	 * There is probably more than one possible match candidate for given feature point.
	 * By looking at the distance of the feature descriptors that come into consideration,
	 * a decision is made if the match is kept or rejected.
	 * That is to say: If the distance among possible matches is large (i.e. the ratio is small)
	 * then the probability of a false match is low if the best match is picked.
	 * On the other hand the chances are 50/50 to pick the wrong match if both candidates share a similar distance.
	 * Therefore only distinctive matches with unique distances are kept whereas ambiguous or uncertain matches are rejected
	 * @param kNNmatches The raw matches as they are returned from the kNN-matcher
	 * @param filteredMatches The filtered unique matches
	 */
	void rejectAmbiguousMatches(std::vector<Matches>& kNNmatches, Matches& filteredMatches);


	/**
	 * @brief Combines found matches from image0 -> image1 with the matches from image1 -> image0
	 * @param m0 First set of matches
	 * @param m1 Second set of matches
	 * @param matchUnion The duplicate free combination of both sets of matches
	 */
	void computeMatchUnion(Matches& m0, Matches& m1, Matches& matchUnion);


	/**
	 * @brief Determines the intersection of found matches from "image0 -> image1" and the matches from "image1 -> image0"
	 * @param m0 First set of matches
	 * @param m1 Second set of matches
	 * @param matchIntersection The intersecting set of both sets of matches
	 */
	void computeMatchIntersection(Matches& m0, Matches& m1, Matches& matchIntersection);


	/**
	 * @brief Extract pure image coordinates from OpenCV's keypoint datas-tructure and undistorts them using given distortion coefficients
	 * @param K
	 * @param distCoeffs
	 * @param matches
	 * @param keypoints0
	 * @param keypoints1
	 * @param points0
	 * @param points1
	 */
	void extractAndUndistortPoints(cv::Mat& K, cv::Mat& distCoeffs, Matches& matches, std::vector<cv::KeyPoint>& keypoints0, std::vector<cv::KeyPoint>& keypoints1, Points2D& points0, Points2D& points1);


	/**
	 * @brief Computes the Homography Matrix H and refines points (small baseline preferred)
	 * @details A first guess of the Homography H is estimated from the set of corresponding points (outliers included) using RANSAC.
	 * 			Only the corresponding points that agree with H are kept, outliers are discarded.
	 * 			findHomography() then recalculates a new Matrix based on inliers only
	 * @param points0 Image coordinates, i.e. center of keypoint0
	 * @param points1 Image coordinates, i.e. center of keypoint1
	 * @param H The resulting Homography
	 * @param refine Should outliers be removed from the set of points using the calculated Homography
	 */
	void computeHomographyMatrix(Points2D& points0, Points2D& points1, cv::Mat& H, bool refine);


	/**
	 * @brief Computes the Fundamental Matrix F and refines points (large baseline preferred)
	 * @details A first guess of the Fundamental Matrix F is estimated from the set of corresponding points (outliers included) using RANSAC.
	 * 			Only the corresponding points that agree with F are kept, outliers are discarded.
	 * 			Then F is calculated again - this time with the refined set of matches using the 8-Point algorithm.
	 *
	 * @param points0 Image coordinates, i.e. center of keypoint0. Attention: points0[i] corresponds to points1[i]
	 * @param points1 Image coordinates, i.e. center of keypoint1
	 * @param F The resulting Fundamental Matrix
	 * @param refine Should outliers be removed from the set of points using the calculated Fundamental Matrix
	 */
	void computeFundamentalMatrix(Points2D& points0, Points2D& points1, cv::Mat& F, bool refine);


	/**
	 * @brief Computes Essential Matrix E from Fundamental Matrix F
	 * @details Matrix product: \f$ E=K^{T}FK \f$
	 * @param K Camera Intrinsics
	 * @param F Fundamental Matrix
	 * @param E Resulting Essential Matrix
	 */
	void computeEssentialFromFundamental(cv::Mat& K, cv::Mat& F, cv::Mat& E);


	/**
	 * @brief Gain rotation and translation of the initial camera pair from the essential matrix
	 * @details Decomposes E into R|t
	 *          Singular value decomposition of E is U D V^T where D is a diagonal matrix with diag(1,1,0)
	 *          [t]_x is the vector t transformed to a skew-symmetric matrix http://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication
	 *          See: "Multiple View Geometry in Computer Vision", Hartley and Zisserman, page 258, 9.6.2 - Extraction of cameras from the essential matrix
	 * @param K Intrinsic Camera Matrix
	 * @param E Resultiing Essential Matrix
	 * @param p0 Set of points in first image
	 * @param p1 Set of points in second image
	 * @param R Relative rotation of the second camera
	 * @param t Relative translation of the second camera
	 * @param P0 Resulting projection matrix of first camera
	 * @param P1 Resulting projection matrix of second camera
	 */
	void computeCameraPoseFromEssential(cv::Mat& K, cv::Mat& E, Points2D& p0, Points2D& p1,  cv::Mat& R,  cv::Mat& t, cv::Mat& P0, cv::Mat& P1);


	/**
	 * @brief Test points if they are in front of the specified camera
	 * @details This function is called from computeCameraPoseFromEssential()
	 *          A reconstructed point will be in front of both cameras P0 and P1 only in one of the four solutions
	 *          See: "Multiple View Geometry in Computer Vision", Hartley and Zisserman, page 258, 9.6.3 - Geometrical interpretation of the four solutions
	 * @param R Rotation of the second Camera
	 * @param t Translation of the second camera
	 * @param K Intrinsic Camera Matrix
	 * @param p0 Image coordinates of the first image
	 * @param p1 Image coordinates of the second image
	 * @return Number of points in front of both cameras
	 */
	uint testCameraPoseSolution(cv::Mat& R, cv::Mat& t, cv::Mat& K, Points2D& p0, Points2D& p1);


	/**
	 * @brief Triangulates world points given two projection matrices and a set of corresponding image coordinates for each camera
	 * @details  Homogeneous approach using Direct Linear Transformation: (DLT)
	 *			 Hartley & Zisserman: 12.2 Linear triangulation methods (page 312)
	 * @param P0 Projection Matrix of first camera
	 * @param P1 Projection Matrix of second camera
	 * @param x0 Point set of first camera
	 * @param x1 Point set of second camera (corresponding to point of first camera)
	 * @param WorldCoordinates Resulting world coordinates
	 */
	void triangulate(cv::Mat& P0, cv::Mat& P1, Points2D& x0, Points2D& x1, Points3D& WorldCoordinates);


	/**
	 * @brief Calculate the reprojection error by comparison with the original projection.
	 * @details Given the camera projection Matrix P, the set of 2D feature points and their corresponding points in 3D,
	 * 			the projection error made for each feature point as well as the average error is computed.
	 * @param P
	 * @param p
	 * @param worldCoordinates
	 * @param pReprojected
	 * @param reprojectionErrors
	 * @param avgReprojectionError
	 */
	void computeReprojectionError(cv::Mat& P, Points2D& p, Points3D& worldCoordinates, Points3D& pReprojected, Points2D& reprojectionErrors, cv::Point2f& avgReprojectionError);


	/**
	 * @brief Visualize reprojected points
	 * @param p Set of original points
	 * @param pReprojected Set of reprojected points
	 * @param image The image to draw on
	 */
	void drawReprojectionError(Points2D& p, Points3D& pReprojected, cv::Mat& image);


	/**
	 * @brief Visualizes epipolar lines
	 * @param F Fundamental Matrix
	 * @param img0 First image
	 * @param img1 Second image
	 * @param points0 Point set of first image
	 * @param points1 Point set of second image
	 */
	void drawEpipolarLines(cv::Mat& F, cv::Mat& img0, cv::Mat& img1, Points2D& points0, Points2D& points1);


	/**
	 * @brief Computes Transfer Error (cost function for H (and with limitations F))
	 * @details \f$ d(x',H\cdot x)^{2}\f$
				See: "Multiple View Geometry in Computer Vision", Hartley and Zisserman, page 94, equation 4.6
	 * @param point0 Measured points in the first image x
	 * @param point1 Measured points in the second image x'
	 * @param H Homography
	 * @return Deviance
	 */
	double transferError(cv::Point2f& point0, cv::Point2f& point1, cv::Mat& H);


	/**
	 * @brief Computes Symmetric Transfer Error (cost function for H (and with limitations F))
	 * @details \f$d(x,H^{-1}\cdot x')^{2}+d(x',H\cdot x)^{2}\f$
				See: "Multiple View Geometry in Computer Vision", Hartley and Zisserman, page 95, equation 4.7
	 * @param point0 Measured points in the first image x
	 * @param point1 Measured points in the second image x'
	 * @param H Homography
	 * @return Deviance
	 */
	double symmetricTransferError(cv::Point2f& point0, cv::Point2f& point1, cv::Mat& H);


	/**
	 * @brief Computes Point to Epipolar Line Cost a.k.a. PELC (cost function for F)
	 * @details \f$\frac{(x'^{T}Fx)^{2}}{(Fx)_{1}^{2}+(Fx)_{2}^{2}+(F^{T}x')_{1}^{2}+(F^{T}x')_{2}^{2}}\f$
	 *          See: "Multiple View Geometry in Computer Vision", Hartley and Zisserman, page 287, equation 11.9
	 * @param point0 Measured points in the first image x
	 * @param point1 Measured points in the second image x'
	 * @param F The Fundamental Matrix
	 * @return Deviance
	 */
	double pointToEpipolarLineCost(cv::Point2f& point0, cv::Point2f& point1, cv::Mat& F);


	/**
	 * @brief Computes criterion for detecting degenerated camera motion and planar point distribution
	 * @details If GRIC(H) < GRIC(F) then (camera motion fits H) else (camera motion fits F)
	 *          Residuals for H: transfer error
	 *          Residuals for F: transfer error + point to epipolar line costs
	 *          See: "An Assessment of Information Criteria for Motion Model Selection", T orr, P.H.S., Computer Vision and Pattern Recognition, 1997
	 * @param point0 Measured points in the first image x
	 * @param point1 Measured points in the second image x'
	 * @param H Input Homography
	 * @param F Input Fundamental Matrix
	 * @param gricH Geometric Robust Information Criterion (GRIC) for H
	 * @param gricF Geometric Robust Information Criterion (GRIC) for F
	 * @param pelc Pure Point to Epipolar Line Costs
	 */
	void computeGRICandPELC(Points2D& point0, Points2D& point1, cv::Mat& H, cv::Mat& F, double &gricH, double &gricF, double& pelc);


};


#endif /* STEREORECONSTRUCTION_H_ */
