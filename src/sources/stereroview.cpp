
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "stereoview.h"

using namespace std;

Stereo::Stereo() {
	confidenceLevel = 0.99; //RANSAC
	matchUniquenessRatio = Params::get()->getLoweRatio(); //ambiguous point correspondence rejection: more distinctive = smaller value, only makes sense between ]0.0, 1.0]
	maxDistance = Params::get()->getFTol(); //allowed distance of a point from epipolar line in order to be an inlier (findFundamentalMat)
	maxReprojectionError =  Params::get()->getHTol(); //allowed reprojection error of a point pair in order to be an inlier (findHomography)

	featurePointDetectorType = GoodFeaturesToTrack; // SURF, SIFT, GoodFeaturesToTrack, FAST, ORB
	matcherPolicy = ONEWAY; //ONEWAY, INTERSECTION, UNION

	switch(featurePointDetectorType) {
		//with orientation/scale
		case SURF: { //SURF-detector still works for larger baselines

			//featurePointDetector = new cv::SurfFeatureDetector(10);

			//incrementally change detector parameters until max features are found or max iteration is reached
			//int minFeatures = 400;
			//int maxFeatures = 1000;
			//int maxIterations = 5;
			//featurePointDetector = new cv::DynamicAdaptedFeatureDetector(new cv::SurfAdjuster(10.0, true), minFeatures, maxFeatures, maxIterations);

			//distribute features on a regular grid as best as possible
			int maxFeatures = 1000;
			int rows = 5;
			int columns = 5;
			featurePointDetector = new cv::GridAdaptedFeatureDetector(new cv::SurfAdjuster(5.0, true), maxFeatures, rows, columns);
			featurePointDescriptor = new cv::SurfDescriptorExtractor();
		}
		break;

		case SIFT: { //SIFT-detector still works for larger baselines
			featurePointDetector = new cv::SiftFeatureDetector();
			featurePointDescriptor = new cv::SiftDescriptorExtractor();
		}
		break;

		//without orientation/scale
		case GoodFeaturesToTrack: { //Variant of good old Harris Corner detector

			int maxCorners = 300;
			double qualityLevel = 0.05;
			double minDistance = 10.0;
			int blockSize = 3;
			bool useHarrisDetector = false;
			double k = 0.04;

			//featurePointDetector = new cv::GoodFeaturesToTrackDetector(maxCorners, qualityLevel, minDistance, blockSize, useHarrisDetector, k);

			//distribute features on a regular grid
			int maxFeatures = 4000;
			featurePointDetector = new cv::GridAdaptedFeatureDetector(new cv::GoodFeaturesToTrackDetector(maxCorners, qualityLevel, minDistance, blockSize, useHarrisDetector, k), maxFeatures, 5, 5); //max rows, cols

			//detect points over multiple levels of a Gaussian pyramid. Useful for detectors that are not inherently scaled
			//int pyramidLevels = 5;
			//featurePointDetector = new cv::PyramidAdaptedFeatureDetector(new cv::GoodFeaturesToTrackDetector(maxCorners, qualityLevel, minDistance, blockSize, useHarrisDetector, k), pyramidLevels);

			featurePointDescriptor = new cv::BriefDescriptorExtractor(32);
		}
		break;

		case FAST: { //FAST-detector works best for short baselines (missing orientation invariance)
			//featurePointDetector = new cv::FastFeatureDetector(20, true);

			//adaptively adjusting detector that iteratively detects until the desired number of features are found
			//int minFeatures = 1000;
			//int maxFeatures = 1000;
			//int maxIterations = 5;
			//featurePointDetector = new cv::DynamicAdaptedFeatureDetector(new cv::FastAdjuster(10,true), minFeatures, maxFeatures, maxIterations);

			//distribute features on a regular grid as best as possible
			int maxFeatures = 1000;
			int rows = 5;
			int columns = 5;
			featurePointDetector = new cv::GridAdaptedFeatureDetector(new cv::FastAdjuster(20,true), maxFeatures, rows, columns);
			featurePointDescriptor = new cv::BriefDescriptorExtractor(32);
		}
		break;

		case ORB: { //Oriented BRIEF
			featurePointDetector = new cv::OrbFeatureDetector(2000);
			featurePointDescriptor = new cv::OrbDescriptorExtractor();
		}
		break;
}

}

Stereo::~Stereo() { }


int Stereo::detectAndDescribeInterestPoints(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
	cout << "Detecting interest points..." << endl;

	//detect features
	featurePointDetector->detect(image, keypoints);

	//describe found features
	featurePointDescriptor->compute(image, keypoints, descriptors);

	return keypoints.size(); //number of found features
}



int Stereo::findCorrespondingInterestPoints(   const cv::Mat& image0, //first frame
												const cv::Mat& image1, //second frame
												Matches& matches, //output matches
												cv::Mat& descriptors0, cv::Mat& descriptors1,
												std::vector<cv::KeyPoint>& keypoints0,	//output keypoints of first image
												std::vector<cv::KeyPoint>& keypoints1)	//output keypoints of second image
{
	cout << "Finding corresponding pairs of interest points..." << endl;

	//match found features
	//chose matcher according to type of used descriptor (float/uchar)
	switch(featurePointDetectorType) {
		//with orientation/scale
		case SURF:
			matcher = new cv::FlannBasedMatcher(); //Fast Approximate Nearest Neighbor Search Library, L2 = normal euclidean distance
			break;

		case SIFT:
			matcher = new cv::FlannBasedMatcher(); //Fast Approximate Nearest Neighbor Search Library, L2 = normal euclidean distance
			break;

		//without orientation/scale
		case GoodFeaturesToTrack:
			matcher = new cv::BruteForceMatcher<cv::Hamming>;
			break;

		case FAST:
			matcher = new cv::BruteForceMatcher<cv::Hamming>;
			break;

		case ORB:
			matcher = new cv::BruteForceMatcher<cv::Hamming>;
			break;
	}

	//match found features using a the OpenCV kNN matcher which does not return a single candidate only,
	//but the k Nearest Neighbors in terms of the "distance" between the descriptors of the match candidates
	std::vector<Matches> kNNmatches0;
	matcher->knnMatch(descriptors0, descriptors1, kNNmatches0, 2); //image0 -> image1, 2 Nearest neighbors
	cout << "Found matches from image 0 to image 1: " << kNNmatches0.size() << endl;

	//there are two possible matches for a specific feature in the corresponding image
	//dismiss ambiguous matches that are too similar, only keep clearly distinguished pairs (Lowe ratio)
	Matches matches0;
	rejectAmbiguousMatches(kNNmatches0, matches0);
	cout << "Filtered matches from image 0 to image 1 (Lowe ratio): " << matches0.size() << endl;

	//matches not only from 0 -> 1 but also from 0 -> 1 are considered
	if(matcherPolicy != ONEWAY) {
		std::vector<Matches> kNNmatches1;
		matcher->knnMatch(descriptors1, descriptors0, kNNmatches1, 2); //image0 <- image1, 2 Nearest neighbors
		cout << "Found matches from image 1 to image 0: " << kNNmatches1.size() << endl;

		Matches matches1;
		rejectAmbiguousMatches(kNNmatches1, matches1);
		cout << "Filtered matches from image 1 to image 0 (Lowe ratio): " << matches1.size() << endl;

		if(matcherPolicy == INTERSECTION) {
			//remove unidirectional matches (only bidirectional matches survive)
			Matches matchIntersection;
			computeMatchIntersection(matches0, matches1, matchIntersection);
			matches = matchIntersection;
			cout << "Match intersection: " << matchIntersection.size() << endl;
		}

		if(matcherPolicy == UNION) {
			//combine matches (bidirectional matches are added only once)
			Matches matchUnion;
			computeMatchUnion(matches0, matches1, matchUnion);
			matches = matchUnion;
			cout <<"Match union: " << matchUnion.size() << endl;
		}

	} else {
		//skip the whole bijectivity issue and just take matches from 0 to 1
		matches = matches0;
	}

	return matches.size(); //number of found correspondences

}


void Stereo::rejectAmbiguousMatches(std::vector<Matches>& kNNmatches, Matches& filteredMatches) {
	for (std::vector<Matches>::iterator it = kNNmatches.begin(); it != kNNmatches.end(); ++it) {
		if(it->size() == 2) { //there are two possible matches to chose from
			if((*it)[0].distance / (*it)[1].distance < matchUniquenessRatio) { //only keep distinctive matches with a large description distance to the second best match
				filteredMatches.push_back( cv::DMatch((*it)[0].queryIdx, (*it)[0].trainIdx, (*it)[0].distance) ); //add first (=best) match to the filtered set of matches
			}
		}
	}
}


void Stereo::computeMatchUnion(Matches& m0, Matches& m1, Matches& matchUnion) {
	assert(matchUnion.empty());
	matchUnion.insert(matchUnion.end(), m0.begin(), m0.end()); //begin with the set of matches 0->1

	//iterate over matches from 0<-1 and add them to the union set if the match is not already part of the set 0->1
	for(Matches::iterator it1 = m1.begin(); it1 != m1.end(); ++it1) { //matches 0<-1
		bool unique = true;
		for(Matches::iterator it0 = m0.begin(); it0 != m0.end(); ++it0) { //matches 0->1
			if((*it0).queryIdx == (*it1).trainIdx && (*it1).queryIdx == (*it0).trainIdx) { //matches are identical (bijective)
				unique = false;
				break; //duplicate match is already in set
			}
		}
		if(unique) {
			matchUnion.push_back(cv::DMatch((*it1).queryIdx, (*it1).trainIdx, (*it1).distance));
		}
	}
}


void Stereo::computeMatchIntersection(Matches& m0, Matches& m1, Matches& matchIntersection) {
	for(Matches::iterator it0 = m0.begin(); it0 != m0.end(); ++it0) { //0->1
		for(Matches::iterator it1 = m1.begin(); it1 != m1.end(); ++it1) { //0<-1
			if((*it0).queryIdx == (*it1).trainIdx && (*it1).queryIdx == (*it0).trainIdx) { //matches are identical (bijective)
				matchIntersection.push_back(cv::DMatch((*it0).queryIdx, (*it0).trainIdx, (*it0).distance)); //add match to intersection set
				break; //there is just one correspondence
			}
		}
	}
}


void Stereo::extractAndUndistortPoints(cv::Mat& K, cv::Mat& distCoeffs, Matches& matches, std::vector<cv::KeyPoint>& keypoints0, std::vector<cv::KeyPoint>& keypoints1, Points2D& points0, Points2D& points1) {
	//extract matched image coordinates from list of matches
	vector<int> pointIndices0(matches.size()), pointIndices1(matches.size()); //indices into keypoint array
	for(uint i = 0; i < matches.size(); i++ ) {
		pointIndices0[i] = matches[i].queryIdx;
		pointIndices1[i] = matches[i].trainIdx;
	}
	cv::KeyPoint::convert(keypoints0, points0, pointIndices0); //convert from "center of keypoints" to Point2f
	cv::KeyPoint::convert(keypoints1, points1, pointIndices1);

	//undistort extracted feature points (input: pixel coordinates, output: "ideal" coordinates)
	cv::undistortPoints(points0, points0, K, distCoeffs);
	cv::undistortPoints(points1, points1, K, distCoeffs);

	//convert undistorted points back into pixel space
	//bug in undistortPoints: the output points are returned in "ideal" coordinates, i.e. independent from K
	//but the documentation states coordinates are in pixel space...
	//x = x*fx + cx
	//y = y*fy + cy
	float fx = static_cast<float>(K.at<double>(0,0));
	float cx = static_cast<float>(K.at<double>(0,2));
	float fy = static_cast<float>(K.at<double>(1,1));
	float cy = static_cast<float>(K.at<double>(1,2));
	for(unsigned int i = 0; i < points0.size(); i++) {
		points0[i].x = points0[i].x * fx + cx;
		points0[i].y = points0[i].y * fy + cy;
		points1[i].x = points1[i].x * fx + cx;
		points1[i].y = points1[i].y * fy + cy;
	}

	//delete points that are undistorted beyond the image borders (would lead to errors during triangulation)
	Points2D points0_restricted, points1_restricted;
	for(unsigned int i = 0; i < points0.size(); i++) {
		if(   points0[i].x >= 0.0 && points0[i].y >= 0.0
		   && points1[i].x >= 0.0 && points1[i].y >= 0.0 ) {
			points0_restricted.push_back(points0[i]);
			points1_restricted.push_back(points1[i]);
		}
	}
	points0 = points0_restricted;
	points1 = points1_restricted;

}


void Stereo::computeHomographyMatrix(	Points2D& points0, //centers of keypoints0
	    								Points2D& points1,  //centers of keypoints1
	    								cv::Mat& H, //Homography Matrix
	    								bool refine)
{

    //estimate Homography Matrix H using RANSAC
	std::vector<uchar> status(points0.size(), 1); //status for each point (inlier if 1, outlier if 0)
    if (points0.size() > 5) { //enough feature points found
	H = cv::findHomography(	cv::Mat(points0),
							cv::Mat(points1),
    		  	  	  	  	cv::RANSAC, //cv::LMEDS cv::RANSAC
    		  	  	  	  	maxReprojectionError, //maximum allowed reprojection error to treat a point pair as an inlier
    		  	  	  	  	status);
    } else {
		cerr << "Tracking lost: Not enough Features." << endl;
    	H = cv::Mat::eye(3, 3, CV_32FC1);
    }

    if(refine) {
    	//remove outliers from list of matches (WARNING: leads to nondeterministic match refinement due to RANSAC)
    	Points2D refinedPoints0, refinedPoints1;
    	for(uint index = 0; index < points0.size(); index++) {
    		if(status[index] == 1) { //inlier
				refinedPoints0.push_back(points0[index]); //updates image coordinates.
				refinedPoints1.push_back(points1[index]);
    		}
    	}
    	points0 = refinedPoints0;
    	points1 = refinedPoints1;
    	cout << "Homography refined matches: " << points0.size() << endl;

    }
    //Utils::printMatrix(H, "Homography Matrix:");
}


void Stereo::computeFundamentalMatrix(  Points2D& points0, //centers of keypoints0
	    								Points2D& points1,  //centers of keypoints1
	    								cv::Mat& F, //Fundamental Matrix
	    								bool refine)
{

	//estimate Fundamental Matrix F using RANSAC
	std::vector<uchar> status(points0.size(), 1); //status for each point (inlier if 1, outlier if 0)
	if (points0.size() > 8) { //enough feature points found
		F = cv::findFundamentalMat( cv::Mat(points0), //image coordinates from first image
									cv::Mat(points1), //image coordinates from second image
									status, //inlier or outlier
									CV_FM_RANSAC, //method CV_FM_LMEDS , CV_FM_RANSAC
									maxDistance, //maximum distance from a point to an epipolar line in pixels
									confidenceLevel); //desirable level of confidence
	} else {
		cerr << "Tracking lost: Not enough Features." << endl;
		F = cv::Mat::eye(3, 3, CV_32FC1);
	}

	if(refine) {
		//remove outliers from list of matches (WARNING: leads to nondeterministic match refinement due to RANSAC)
		Points2D refinedPoints0, refinedPoints1;
		for(uint index = 0; index < points0.size(); index++) {
			if(status[index] == 1) { //inlier
				refinedPoints0.push_back(points0[index]); //updates image coordinates.
				refinedPoints1.push_back(points1[index]);
			}
		}
		points0 = refinedPoints0;
		points1 = refinedPoints1;
		cout << "Fundamental Matrix refined matches: " << points0.size() << endl;

		//refine Fundamental Matrix with new set of inliers using the 8-Point algorithm
		F = cv::findFundamentalMat(	cv::Mat(points0),
									cv::Mat(points1),
									CV_FM_8POINT); //8-point method as noise is small
	}

	//Utils::printMatrix(F, "Fundamental Matrix:");

}


void Stereo::computeEssentialFromFundamental(cv::Mat& K, cv::Mat& F, cv::Mat& E) {
	E = K.t() * F * K;
	//Utils::printMatrix(E, "Essential Matrix:");
}

void Stereo::computeCameraPoseFromEssential(cv::Mat& K, cv::Mat& E, Points2D& p0, Points2D& p1, cv::Mat& R, cv::Mat& t, cv::Mat& P0, cv::Mat& P1) {

	//Determinant of E should be 0. But in reality, this won't be the case
	//TODO: If det(E) is far away from zero, replace E with closest substitute (Frobeniusnorm) - see Zhengyou Zhang rank-2 constraint
	double detE = cv::determinant(E);
	cout << setprecision(20) << "det(E): " << detE << endl;

	//det(U) and det(V) should both be positive which is only the case when det(E) >= 0
	if(detE < 0.0) {
		E = E * -1.0; //workaround SVD ambiguity if det < 0
	}

	//perform singular value decomposition of Essential Matrix
    cv::SVD decomposition(E);

    //Rotation R (2 solutions due to possible rotation of 180° about the baseline)
    // construct orthogonal matrix W
    //		┌         ┐
	//		│ 0 -1  0 │
	//  W = │ 1  0  0 │
	//		│ 0  0  1 │
	//		└         ┘
    cv::Mat W(3, 3, CV_64FC1, 0.0);
    W.at<double>(1,0) = 1.0;
    W.at<double>(0,1) = -1.0;
    W.at<double>(2,2) = 1.0;

    cv::Mat Wt;
    cv::transpose(W, Wt); //W^T

    cv::Mat Ra = decomposition.u * W * decomposition.vt; //R = U W V^T
    cv::Mat Rb = decomposition.u * Wt * decomposition.vt; //R = U W^T V^T
    //Utils::printMatrix(Ra, "Ra");
    //Utils::printMatrix(Rb, "Rb");


    //Translation t (2 solutions as t is parallel to last column of U and therefore determined up to direction only)
    cv::Mat ta = decomposition.u.col(2);
    cv::Mat tb = -ta;
    //Utils::printMatrix(ta, "ta");
    //Utils::printMatrix(ta, "tb");


    //select correct Rotation/Translation from 4 possible configurations
    uint pointsInFront[4]; //number of points that are in front of both cameras (one point should be enough if baseline is in the sweet spot)
    pointsInFront[0] = testCameraPoseSolution(Ra, ta, K,  p0, p1);
    pointsInFront[1] = testCameraPoseSolution(Ra, tb, K,  p0, p1);
    pointsInFront[2] = testCameraPoseSolution(Rb, ta, K,  p0, p1);
    pointsInFront[3] = testCameraPoseSolution(Rb, tb, K,  p0, p1);

    //find camera configuration with maximum number of valid points and detect possible ambiguities
    uint maxValidPoints = 0;
    int solution = -1;
    bool ambiguity = false;
    for(int i = 0; i < 4; i++) {
    	if(pointsInFront[i] != 0 && pointsInFront[i] == maxValidPoints) { //two or more cameras have the same number of points in front
    		ambiguity = true;
    	}
    	if(pointsInFront[i] > maxValidPoints) { //new candidate found
    		maxValidPoints = pointsInFront[i];
    		solution = i;
    		ambiguity = false;
    	}
    }

    if(ambiguity || maxValidPoints == 0) {
    	cerr << "Error: Ambiguous camera setting" << endl;
    	exit(0);
    }

    if(maxValidPoints != p0.size()) {
	   cerr << "Warning: Some ambiguous points in camera setting" << endl;
	   cerr << "Camera configuration 0) " << pointsInFront[0] << endl;
	   cerr << "Camera configuration 1) " << pointsInFront[1] << endl;
	   cerr << "Camera configuration 2) " << pointsInFront[2] << endl;
	   cerr << "Camera configuration 3) " << pointsInFront[3] << endl;
	   cerr << "Chosen configuration: " << solution << endl;
   }

    switch(solution) {
    case 0:
    	R = Ra;
    	t = ta;
    	break;

    case 1:
    	R = Ra;
    	t = tb;
    	break;

    case 2:
    	R = Rb;
    	t = ta;
    	break;

    case 3:
    	R = Rb;
    	t = tb;
    	break;
    }

    //Utils::printMatrix(R, "R:");
    //Utils::printMatrix(t, "t:");

    //build projection matrix of first camera P = K[I|0]
    P0 = (cv::Mat_<double>(3,4) << 	1.0, 0.0, 0.0, 0.0,
    								0.0, 1.0, 0.0, 0.0,
    								0.0, 0.0, 1.0, 0.0 );
    P0 = K * P0;

    //projection matrices of second camera P' = K[R|t]
    cv::hconcat(R, t, P1);
    P1 = K * P1;
}


uint Stereo::testCameraPoseSolution(cv::Mat& R, cv::Mat& t, cv::Mat& K, Points2D& p0, Points2D& p1) {
	assert(p0.size() == p0.size());

	//projection matrix of first camera P = K[I|0]
    cv::Mat P0 = (cv::Mat_<double>(3,4) << 	1.0, 0.0, 0.0, 0.0,
    										0.0, 1.0, 0.0, 0.0,
    										0.0, 0.0, 1.0, 0.0 );
    P0 = K * P0;

    //projection matrices of second camera P' = K[R|t]
    cv::Mat P1;
    cv::hconcat(R, t, P1);
    P1 = K * P1;

    //triangulate
    Points3D worldCoordinates;
    triangulate(P0, P1, p0, p1, worldCoordinates);

    //calculate reprojection error
    Points3D p0_r, p1_r; //reprojected cartesian image coordinate with depth value
    Points2D p0_err, p1_err; //difference between original and reprojected 2D coordinates
    cv::Point2f avgReprojectionError;
    computeReprojectionError(P0, p0, worldCoordinates, p0_r, p0_err, avgReprojectionError);
    computeReprojectionError(P1, p1, worldCoordinates, p1_r, p1_err, avgReprojectionError);

    //determine if points are in front of both cameras
    uint pointsInFront = 0;
    for(uint i = 0; i < p0.size(); i++) {
    	//cout << "Point " << i << ":\n";
    	//cout << "Original 2D coordinate: p: " << p0[i].x << ", " << p0[i].y << "  and  p': " << p1[i].x << ", " << p1[i].y << endl;
    	//cout << "3D coordinate:  X: " << worldCoordinates[i].x << ", " << worldCoordinates[i].y << ", " << worldCoordinates[i].z <<  endl;
    	//cout << "Reprojected 2D coordinates: x: " << p0_r[i].x << ", " << p0_r[i].y << "  and  x': " << p1_r[i].x << ", " << p1_r[i].y << "  depth0: " << p0_r[i].z << ", depth1: " << p1_r[i].z << endl;
    	//cout << "Reprojection error: " << p0_err[i].x << ", " << p0_err[i].y <<  "  and  x': " << p1_err[i].x << ", " << p1_err[i].y << endl;

    	double depth0 = p0_r[i].z; //depth information from homogeneous coordinate w (z-buffer style)
    	double depth1 = p1_r[i].z;
    	if (depth0 > 0 && depth1 > 0) { //valid camera configuration
    		pointsInFront++;;
    	}
    }

	return pointsInFront;
}


void Stereo::triangulate(cv::Mat& P0, cv::Mat& P1, Points2D& x0, Points2D& x1, Points3D& result3D) {
	assert(x0.size() == x1.size());
	result3D.clear();

    for(uint i = 0; i < x0.size(); i++) {
    	//set up a system of linear equations from x = PX and x' = P'X
    	cv::Mat A(4, 4, CV_64FC1);
    	A.row(0) = x0[i].x * P0.row(2) - P0.row(0);
    	A.row(1) = x0[i].y * P0.row(2) - P0.row(1);
    	A.row(2) = x1[i].x * P1.row(2) - P1.row(0);
    	A.row(3) = x1[i].y * P1.row(2) - P1.row(1);
    	//Utils::printMatrix(A, "A:");

    	//normalize each row of A with its L2 norm, i.e. |row| = sqrt(sum_j(row[j]^2)) to improve condition of the system
    	for (int i = 0; i < A.rows; i++) {
    		double dsquared = 0;
    			for(int j = 0; j < 4; j++) {
    				dsquared = dsquared + pow(A.at<double>(i, j), 2);
    			}
    			A.row(i) = A.row(i) * (1 / sqrt(dsquared));
    	}

    	double detA = cv::determinant(A);
    	//cout << setprecision(3) << "det(A): " << detA << endl;
    	if(detA < 0.0) {
    		//workaround SVD ambiguity if det < 0
       		A = A * -1.0;
    	}

    	//solve A x = 0 using Singular Value Decomposition
    	cv::SVD decomposition(A);

    	//homogeneous least-square solution corresponds to least singular vector of A, that is the last column of V or last row of V^T
    	//i.e. [x,y,z,w] = V^T.row(3)
    	float x = static_cast<float>(decomposition.vt.at<double>(3, 0));
    	float y = static_cast<float>(decomposition.vt.at<double>(3, 1));
    	float z = static_cast<float>(decomposition.vt.at<double>(3, 2));
    	float w = static_cast<float>(decomposition.vt.at<double>(3, 3));
     	//convert homogeneous to cartesian coordinates
    	result3D.push_back(cv::Point3f(x/w, y/w, z/w));

       	//cout << "2D Coordinates x: " << x0[i].x << ", " << x0[i].y << "  and  x': " << x1[i].x << ", " << x1[i].y << endl;
    	//cout << "3D Coordinate  X: " << x/w << ", " << y/w << ", " << z/w <<  endl;
    	//cout << "Homogeneous 3D Coordinate X : " << x << ", " << y << ", " << z << ", " << w << endl;

    }
}


void Stereo::computeReprojectionError(cv::Mat& P, Points2D& p, Points3D& worldCoordinates, Points3D& pReprojected, Points2D& reprojectionErrors, cv::Point2f& avgReprojectionError) {
	assert(p.size() == worldCoordinates.size());

	//for all points...
	for(uint i = 0; i < p.size(); i++) {

		//build homogeneous coordinate for projection
		cv::Mat WorldCoordinate_h = cv::Mat(4, 1, CV_64FC1);
		WorldCoordinate_h.at<double>(0,0) = worldCoordinates[i].x;
		WorldCoordinate_h.at<double>(1,0) = worldCoordinates[i].y;
		WorldCoordinate_h.at<double>(2,0) = worldCoordinates[i].z;
		WorldCoordinate_h.at<double>(3,0) = 1.0;

		//perform simple reprojection by multiplication with projection matrix
		cv::Mat pReprojected_h = P * WorldCoordinate_h; //homogeneous image coordinates 3x1

		//convert reprojected image point to carthesian coordinates
		float w = static_cast<float>(pReprojected_h.at<double>(2,0));
		float x_r = static_cast<float>(pReprojected_h.at<double>(0,0) / w); //x = x/w
		float y_r = static_cast<float>(pReprojected_h.at<double>(1,0) / w); //y = y/w

		pReprojected.push_back(cv::Point3f(x_r, y_r, w)); //reprojected cartesian image coordinate with depth value

		//calculate actual reprojection error
		float deltaX = (float)fabs(p[i].x - x_r);
		float deltaY = (float)fabs(p[i].y - y_r);
		reprojectionErrors.push_back(cv::Point2f(deltaX, deltaY));
	}

	//average reprojection error
	avgReprojectionError.x = avgReprojectionError.y = 0.0;
	for(uint i = 0; i < reprojectionErrors.size(); i++) {
		avgReprojectionError.x += reprojectionErrors[i].x;
		avgReprojectionError.y += reprojectionErrors[i].y;
	}
	avgReprojectionError.x /= reprojectionErrors.size();
	avgReprojectionError.y /= reprojectionErrors.size();
}

void Stereo::drawReprojectionError(Points2D& p, Points3D& pReprojected, cv::Mat& image) {
	//Color scheme is BGR
	for(uint i = 0; i < p.size(); i++) {
		if(pReprojected[i].z < 0) { //do we have an outlier?
			cv::circle(image, p[i], 4, cv::Scalar(255, 32, 32), -1); //blue
			cv::circle(image, cv::Point2f(pReprojected[i].x, pReprojected[i].y), 4, cv::Scalar(32, 255, 255), -1); //yellow
		} else {
			cv::circle(image, p[i], 3, cv::Scalar(64, 255, 64), 1); //green points are original
			cv::circle(image, cv::Point2f(pReprojected[i].x, pReprojected[i].y), 3, cv::Scalar(64, 64, 255), 1); //red one is reprojected
		}
	}
}

void Stereo::drawEpipolarLines(cv::Mat& F, cv::Mat& img0, cv::Mat& img1, Points2D& points0, Points2D& points1) {
    //visualize epipolar lines
    std::vector<cv::Vec3f> lines0, lines1;

    //from points in first image compute corresponding epipolar lines in second image
    cv::computeCorrespondEpilines(cv::Mat(points0), 1, F, lines1);
    //from points in second image compute corresponding epipolar lines in first image
    cv::computeCorrespondEpilines(cv::Mat(points1), 0, F, lines0);


    //draw all epipolar lines in second image
    for (vector<cv::Vec3f>::iterator it = lines1.begin(); it != lines1.end(); ++it) {
    	cv::line(img1,
    			 cv::Point(0,-(*it)[2]/(*it)[1]), //first column
    			 cv::Point(img1.cols,-((*it)[2] + (*it)[0] * img1.cols) / (*it)[1]), //last column
    			 cv::Scalar(255,255,255)); //color
    }

    //draw all epipolar lines in second image
    for (vector<cv::Vec3f>::iterator it = lines0.begin(); it != lines0.end(); ++it) {
    	cv::line(img0,
    			 cv::Point(0,-(*it)[2]/(*it)[1]), //first column
    			 cv::Point(img0.cols,-((*it)[2] + (*it)[0] * img0.cols) / (*it)[1]), //last column
    			 cv::Scalar(255,255,255)); //color
    }

    imshow("Epipolar lines in second image corresponding to points in first image", img1);
    imshow("Epipolar lines first image corresponding to points in second image", img0);
}


double Stereo::transferError(cv::Point2f& point0, cv::Point2f& point1, cv::Mat& H) {

	cv::Mat x0 = (cv::Mat_<double>(3,1) << point0.x, point0.y, 1.0);
	cv::Mat x0Projected = cv::Mat_<double>(3,1);

	x0Projected = H * x0; //H * x

	cv::Point2f point0Projected = cv::Point2f(x0Projected.at<double>(0,0), x0Projected.at<double>(1,0));
	cv::Mat diff = (cv::Mat_<double>(2,1) << fabs(point1.x - point0Projected.x), fabs(point1.y - point0Projected.y) ); // |x' - H*x|
	double distance = cv::norm(diff, cv::NORM_L2); //euclidean distance between original and projected image point and in second image

	double error = pow(distance, 2); //transfer error in first image + transfer error in second image

	return error;
}


double Stereo::symmetricTransferError(cv::Point2f& point0, cv::Point2f& point1, cv::Mat& H) {
	cv::Mat Hinverted;
	cv::Mat x0 = (cv::Mat_<double>(3,1) << point0.x, point0.y, 1.0);
	cv::Mat x1 = (cv::Mat_<double>(3,1) << point1.x, point1.y, 1.0);
	cv::invert(H, Hinverted);

	cv::Mat x0Projected = cv::Mat_<double>(3,1);
	cv::Mat x1Projected = cv::Mat_<double>(3,1);

	x0Projected = H * x0; //H * x
	x1Projected = Hinverted * x1; //H^-1 * x'

	cv::Point2f point0Projected = cv::Point2f(x0Projected.at<double>(0,0), x0Projected.at<double>(1,0));
	cv::Point2f point1Projected = cv::Point2f(x1Projected.at<double>(0,0), x1Projected.at<double>(1,0));

	cv::Mat diff0 = (cv::Mat_<double>(2,1) << fabs(point0.x - point1Projected.x), fabs(point0.y - point1Projected.y) ); // |x - H^T*x'|
	cv::Mat diff1 = (cv::Mat_<double>(2,1) << fabs(point1.x - point0Projected.x), fabs(point1.y - point0Projected.y) ); // |x' - H*x|
	double distance0 = cv::norm(diff0, cv::NORM_L2); //euclidean distance between original and projected image point and in first image
	double distance1 = cv::norm(diff1, cv::NORM_L2); //euclidean distance between original and projected image point and in second image

	double error = pow(distance0, 2) + pow(distance1, 2); //transfer error in first image + transfer error in second image

	return error;
}


double Stereo::pointToEpipolarLineCost(cv::Point2f& point0, cv::Point2f& point1, cv::Mat& F) {

		cv::Mat x0 = (cv::Mat_<double>(3,1) << point0.x, point0.y, 1.0);
		cv::Mat x1 = (cv::Mat_<double>(3,1) << point1.x, point1.y, 1.0);
		cv::Mat x0transposed;
		cv::Mat x1transposed;
		cv::Mat Ftransposed;
		cv::transpose(x0, x0transposed);
		cv::transpose(x1, x1transposed);
		cv::transpose(F, Ftransposed);
		cv::Mat x0Projected = cv::Mat_<double>(3,1);
		cv::Mat x1Projected = cv::Mat_<double>(3,1);
		x0Projected = F * x0; //F * x
		x1Projected = Ftransposed * x1; //F^T * x'

		//equation 11.9
		cv::Mat numerator = x1transposed*x0Projected;
		double x0SquaredFirstEntry =  pow(x0Projected.at<double>(0,0), 2);
		double x0SquaredSecondEntry = pow(x0Projected.at<double>(1,0), 2);
		double x1SquaredFirstEntry =  pow(x1Projected.at<double>(0,0), 2);
		double x1SquaredSecondEntry = pow(x1Projected.at<double>(1,0), 2);
		double error = pow(numerator.at<double>(0,0), 2) / (x0SquaredFirstEntry + x0SquaredSecondEntry + x1SquaredFirstEntry + x1SquaredSecondEntry);

		//equation 11.10
		//cv::Mat factor = x1transposed*x0Projected;
		//double x0SquaredFirstEntry =  pow(x0Projected.at<double>(0,0), 2);
		//double x0SquaredSecondEntry = pow(x0Projected.at<double>(1,0), 2);
		//double x1SquaredFirstEntry =  pow(x1Projected.at<double>(0,0), 2);
		//double x1SquaredSecondEntry = pow(x1Projected.at<double>(1,0), 2);
		//double error = pow(factor.at<double>(0,0), 2) * ( 1/(x0SquaredFirstEntry + x0SquaredSecondEntry) + 1/(x1SquaredFirstEntry + x1SquaredSecondEntry) );

		return error;
}


void Stereo::computeGRICandPELC(Points2D& points0, Points2D& points1, cv::Mat& H, cv::Mat& F, double &gricH, double &gricF, double& pelc) {
	double n = points0.size(); //total number of features matched in two frames (inliers and outliers)
	double dimData = 4; //dimension of the data (image coordinates between two frames)
	double lambda1 = log(dimData);
	double lambda2 = log(dimData*n);
	double lambda3 = 2.0; //error limit

	double dimH = 2; //dimension of H
	double dimF = 3; //dimension of F
	double kH = 8; //degrees of freedom for H
	double kF = 7; //degrees of freedom for F

	//estimated standard deviation of the error in the feature detector
	double standardDeviationH = 0.5;
	double standardDeviationF = 0.5;

	std::vector<double> transferErrorH(n); //distance between points x and H*x' and x and H*x'
	std::vector<double> transferErrorF(n); //distance between points x and F*x'
	std::vector<double> PELCErrorF(n); //point to epipolar line costs

	//calculate errors and sum
	double sumPELC = 0.0;
	for(int i = 0; i < n; i++) {
		transferErrorH[i] = transferError(points0[i], points1[i], H);
		transferErrorF[i] = transferError(points0[i], points1[i], F);
		PELCErrorF[i] = pointToEpipolarLineCost(points0[i], points1[i], F);
		sumPELC += PELCErrorF[i];
	}

	//calculate correlation coefficient rho
	double rhoH = 0.0;
	double rhoF = 0.0;
	double rhoPELC = 0.0;
	for(int i = 0; i < n; i++){
		rhoH += min(pow(transferErrorH[i], 2) / pow(standardDeviationH, 2), lambda3 * (dimData - dimH) );
		rhoF += min(pow(transferErrorF[i], 2) / pow(standardDeviationF, 2), lambda3 * (dimData - dimF) );
		rhoPELC += min(pow(PELCErrorF[i], 2) / pow(standardDeviationF, 2), lambda3 * (dimData - dimF) );
	}

	//finally calculate Geometric Robust Information Criterion (GRIC)
	double homographyGRIC  = rhoH + lambda1 * dimH + lambda2 * kH; //transfer error

	//double fundamentalGRIC = rhoF + lambda1 * dimF + lambda2 * kF; //transfer error
	//double fundamentalGRIC = rhoPELC + lambda1 * dimF + lambda2 * kF;  //epipolar line cost error
	double fundamentalGRIC = rhoF + lambda1 * dimF + lambda2 * kF + rhoPELC + lambda1 * dimF + lambda2 * kF; //transfer error + epipolar line cost error

	double pointToEpipolarLineCost = sumPELC / n;
	//double pointToEpipolarLineCost = sumPELC;

	gricH = homographyGRIC;
	gricF = fundamentalGRIC;
	pelc = pointToEpipolarLineCost;

}
