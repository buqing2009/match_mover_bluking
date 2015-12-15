#include <stdio.h>
#include <iostream>
#include <fstream>

#include "multiview.h"


Multiview::Multiview() {
	cout << "\nOpening video file from: " << Params::get()->getVideoFile() << endl;
	video = new VideoIO(Params::get()->getVideoFile()); //input video from command line
	video->parseFilename(Params::get()->getVideoFile(), inputBasename, inputExtension); //split specified filename into basename and extension
	std::stringstream outputVideoFilename;
	outputVideoFilename << inputBasename << "_" << "rendered" << ".avi";
	video->setOutput(outputVideoFilename.str(), Params::get()->getCompressOutputVideo(), Params::get()->getSaveAsImageSequence());

	width = video->getFrameWidth();
	height = video->getFrameHeight();
	drawThreeWayTracks =  Params::get()->getDrawThreeWayTracks();

	drawTracks = Params::get()->getDrawTracks();
	if(drawTracks) {
		std::stringstream trackVideoName;
		trackVideoName << inputBasename << "_" << "tracked" << ".avi";
		trackRecorder.open(outputVideoFilename.str(), CV_FOURCC('D','I','V','X'), video->getFrameRate(), cv::Size(width, height), true);
	}


	frameLimit = Params::get()->getFrameLimit(); //stop processing the video when limit is reached
	keyFrameLimit =  Params::get()->getKeyframeLimit(); //stop processing the video when limit is reached
	maxOpticalFlowLength =  Params::get()->getMaxOpticalFlowLength(); //maximal length of optical flow track

	//correspondence ratio thresholds heavily rely on used feature detector!!!!
	T_min =  Params::get()->getTmin(); //lower limit for correspondence ratio: the lower T_min, the larger the allowed baseline
	T_max = Params::get()->getTmax(); //upper limit for correspondence ratio: the higher T_max, the smaller the allowed the baseline

	//optical flow settings
	maxDrift = 2.0; //allowed drift of pixels when arriving in next keyframe (compared to the already known coordinates)

}


Multiview::~Multiview() { }


void Multiview::workflow_rendertest() {
	cv::Mat K;
	cv::Mat distCoeffs;
	readCameraInstrinics("calibration.xml", K, distCoeffs);

	cv::Mat R0 = (cv::Mat_<double>(3,3) << 	1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 );
	cv::Mat t0 = (cv::Mat_<double>(3,1) << 	0.0, 0.0, 0.0 );
	std::vector<cv::Mat> rotations; //R for each frame
	std::vector<cv::Mat> translations; //t for each frame
	rotations.push_back(R0);
	translations.push_back(t0);

	std::vector<Points3D> worldCoordinateSet;
	Points3D worldCoordinates;


	//some random world coordinates
	worldCoordinates.push_back(cv::Point3f(0.0, -1.0, 4.0));
	worldCoordinates.push_back(cv::Point3f(1.0, 0.0, 5.0));
	worldCoordinates.push_back(cv::Point3f(-1.0, 1.0, 6.0));
	worldCoordinates.push_back(cv::Point3f(-1.0, 1.0, -3.0));
	worldCoordinates.push_back(cv::Point3f(0.0, 0.0, 0.0));
	worldCoordinateSet.push_back(worldCoordinates);
	std::vector<int> frameIndices;
	frameIndices.push_back(0);


	//instantiate our OpenGL based renderer
	renderer = new Renderer(width, height, "Teapot");

	//place virtual object
	cv::Point3f objectPosition;
	determineObjectPosition(worldCoordinateSet, objectPosition);
	renderer->setScaleFactor(objectPosition.z/75.0);
	renderer->setObjectPosition(objectPosition, worldCoordinateSet);
	renderer->setCameraTrack(rotations, translations); //tell the renderer about the various camera poses
	renderer->setViewFrustum(K); //setup virtual camera parameters

	cv::Mat cameraPosition = (cv::Mat_<double>(3,1) << -objectPosition.z, -objectPosition.z, -objectPosition.z);
	renderer->setFreeViewCameraPose(rotations[0], translations[0], cameraPosition, -20.0, 0.0, 0.0); //just the initial free view opengl camera

	//visualize reconstruction
	cout << "Showing reconstructed scene..." << endl;
	showPreview(rotations, translations, frameIndices);

}


void Multiview::workflow() {

	cv::Mat K;
	cv::Mat distCoeffs;

	//import camera intrinsics from file
	readCameraInstrinics("calibration.xml", K, distCoeffs);

	std::vector<PointCorrespondences> keyframePointCorrespondences; //keyframes: 0<->1, 1<->2, ...
	std::vector<ThreeWay> keyframeThreeWayPointCorrespondences; ///keyframes: 0<->1<->2, 1<->2<->3, ....

	int nFrames = std::min(frameLimit, static_cast<int>(video->getNumberOfFrames())); //minimum defines the end
	int keyframe = 0; //hard-coded first keyframe = 0
	bool reachedEnd = false;
	bool lastFrameIsKeyframe = false;

	 /*###############################################################################################
	##    __   ___ ___  ___  __                ___          ___      ___  __              ___  __    ##
	##   |  \ |__   |  |__  |__)  |\/| | |\ | |__     |__/ |__  \ / |__  |__)  /\   |\/| |__  /__`   ##
	##   |__/ |___  |  |___ |  \  |  | | | \| |___    |  \ |___  |  |    |  \ /~~\  |  | |___ .__/   ##
	##                                                                                               ##
	 ###############################################################################################*/

	for(int i = 0; i < keyFrameLimit-1; i++) {
		//get next keyframe from video
		cv::Mat tmp;
		video->getFrame(keyframe, tmp);
		cv::Size gaussSize;
		gaussSize.width =  Params::get()->getGaussSize();
		gaussSize.height =  Params::get()->getGaussSize();
		if(gaussSize.width > 0) {
			cout << "Applying Gaussian blur (size: " << gaussSize.width << " sigma: " << Params::get()->getGaussSigma() << ")..." << endl;
			cv::GaussianBlur(tmp, frame0, gaussSize, Params::get()->getGaussSigma());
		} else {
			tmp.copyTo(frame0);
		}


		Stereo stereo;
		std::vector<cv::KeyPoint> keypointsKeyframe;
		std::vector<PointCorrespondences> interKeyframePointCorrespondences;

		//detect and describe features in key frame
		cv::Mat descriptors0;
		int numberOfFeatures0 = stereo.detectAndDescribeInterestPoints(frame0, keypointsKeyframe, descriptors0);
		
		std::vector<double> PELCCriterion(maxOpticalFlowLength, 0.0); //Point To Epipolar Line Cost (PELC) is an indicator for blurry images
		std::vector<double> GRICDifferenceCriterion(maxOpticalFlowLength, 0.0); //criterion for selecting the next keyframe
		std::vector<double> weightedCriterion(maxOpticalFlowLength, -9999999.0); //weighted criterion (GRICDifference and PELC)for selecting the next keyframe
		int tmpKeyframe = 1;
		int currentFrame;

		//iterate through consecutive frames and compare features with previous keyframe
		for(currentFrame = keyframe+1; currentFrame <= keyframe+maxOpticalFlowLength; currentFrame++) {

			if(currentFrame == nFrames ) { //reached end of video
				cout << "Touched the video's end! (Frame: " << currentFrame-1 << ")" << endl;

				if(tmpKeyframe == 1) { //end of the video was reached without finding a proper keyframe
					cout << "Last frame is no keyframe candidate!" << endl;
					reachedEnd = true;
				}
				if(keyframe+1 == nFrames) { //found keyframe is also last frame of the video
					cout << "Last frame is a proper keyframe!" << endl;
					lastFrameIsKeyframe = true;
				}
				break; //break in any case
			}

			cout << "\n\nFrames: " << keyframe << " -> " << currentFrame << endl;
			int interKeyframeIndex = currentFrame-keyframe; //next frame after keyframe has value 1
			video->getFrame(currentFrame, tmp);

			if(gaussSize.width > 0) {
				cout << "Applying Gaussian blur (size: " << gaussSize.width << " sigma: " << Params::get()->getGaussSigma() << ")..." << endl;
				cv::GaussianBlur(tmp, frame1, gaussSize, Params::get()->getGaussSigma());
			}else {
				tmp.copyTo(frame1);
			}


			cv::Mat H, F;

			PointCorrespondences currentPc(keyframe, currentFrame); //point correspondence between last keyframe and current frame j
			currentPc.setKeypoints0(keypointsKeyframe);
			cv::Mat descriptors1;

			//detect features in second image
			int numberOfFeatures1 = stereo.detectAndDescribeInterestPoints(frame1, currentPc.getKeypoints1(), descriptors1);

			//find point correspondences between two frames
			stereo.findCorrespondingInterestPoints(frame0, frame1, currentPc.getMatches(), descriptors0, descriptors1, currentPc.getKeypoints0(), currentPc.getKeypoints1());

			//extract image coordinates from feature points and undistort them using the distortion coefficients
			stereo.extractAndUndistortPoints(K, distCoeffs, currentPc.getMatches(), currentPc.getKeypoints0(), currentPc.getKeypoints1(), currentPc.getImageCoordinates0(), currentPc.getImageCoordinates1());

			//find Homography Matrix (used for small baseline)
			stereo.computeHomographyMatrix(currentPc.getImageCoordinates0(), currentPc.getImageCoordinates1(), H, false);

			//find Fundamental Matrix (used for large baseline)
			stereo.computeFundamentalMatrix(currentPc.getImageCoordinates0(), currentPc.getImageCoordinates1(), F, false);

			//compute GRIC estimator value
			double homographyGRIC, fundamentalGRIC; //Geometric Robust Information Criterion (GRIC) is an indicator for the camera motion (Rotation: H, Shift: F)
			double PELC; //Point To Epipolar Line Cost (PELC) is an indicator for blurry images
			stereo.computeGRICandPELC(currentPc.getImageCoordinates0(), currentPc.getImageCoordinates1(), H, F, homographyGRIC, fundamentalGRIC, PELC);

			//refine matches using F or H depending on GRIC
			if(homographyGRIC < fundamentalGRIC) {
				cout << setprecision(5) << left << fixed << "H probably describes the camera movement more accurately\t" << "GRIC H: " << homographyGRIC << "\tGRIC F: " << fundamentalGRIC << "\tPELC: " << PELC << endl;
				if(currentFrame == nFrames-1) { //in case the last frame of the video was reached, refine points with H
					//find final Homography Matrix (used for small baseline) Refinement of points may lead to nondeterministic matches if RANSAC is used
					stereo.computeHomographyMatrix(currentPc.getImageCoordinates0(), currentPc.getImageCoordinates1(), H, true);
				}
			} else {
				cout << setprecision(5) << left << fixed << "F probably describes the camera movement more accurately\t" << "GRIC H: " << homographyGRIC << "\tGRIC F: " << fundamentalGRIC << "\tPELC: " << PELC << endl;
				//find final Fundamental Matrix (used for large baseline) Refinement of points may lead to nondeterministic matches if RANSAC is used
				stereo.computeFundamentalMatrix(currentPc.getImageCoordinates0(), currentPc.getImageCoordinates1(), F, true);
			}


			//temporary point correspondences between two keyframes (i.e. i<->i+1, i<->i+2 ... i<->i+n )
			interKeyframePointCorrespondences.push_back(currentPc);

			//compute correspondence ratio
			//see: "Optimal keyframe selection algorithm for three-dimensional reconstruction in uncalibrated multiple images", Seo et al. (2008)
			int T_f = min(numberOfFeatures0, numberOfFeatures1); //number of found features
			int T_c = currentPc.getImageCoordinates0().size(); //number of found correspondences
			double correspondenceRatio = static_cast<double>(T_c) / static_cast<double>(T_f);

			//now check if current frame is a keyframe candidate
			if(T_min <= correspondenceRatio && correspondenceRatio <= T_max) { //baseline is in acceptable range
				cout << "Baseline within allowed range: T_min ("<< T_min <<") < "  << correspondenceRatio << " < T_max ("<< T_max <<")" << endl;
				if(fundamentalGRIC <= homographyGRIC) { //F is probably not degenerated
						//compute normalized GRIC Difference Criterion and save current PELC
						//see "Robust key frame extraction for 3d reconstruction from video streams" In Proceedings of VISAPP Conference, Ahmed, M., Dailey, M., Landabaso, J. & Herrero, N., 2010
						GRICDifferenceCriterion[interKeyframeIndex-1] = (homographyGRIC-fundamentalGRIC) / homographyGRIC;
						PELCCriterion[interKeyframeIndex-1] = PELC;

						//find min/max GRIC value for weighting
						std::vector<double>::iterator GRICDifferenceIterator;
						GRICDifferenceIterator = std::min_element(GRICDifferenceCriterion.begin(), GRICDifferenceCriterion.end());
						double minGRICDifference = *GRICDifferenceIterator;
						GRICDifferenceIterator = std::max_element(GRICDifferenceCriterion.begin(), GRICDifferenceCriterion.end());
						double maxGRICDifference = *GRICDifferenceIterator;

						//find min/max PELC value for weighting
						std::vector<double>::iterator PELCiterator;
						PELCiterator = std::min_element(PELCCriterion.begin(), PELCCriterion.end());
						double minPELC = *PELCiterator;
						PELCiterator = std::max_element(PELCCriterion.begin(), PELCCriterion.end());
						double maxPELC = *PELCiterator;

						//weight GRIC and PELC in order to sensibly combine them
						double weightPELC = fabs(minGRICDifference-maxGRICDifference) / fabs(minPELC-maxPELC);
						double weightGRIC = 1 - weightPELC;
						double weightedScore = weightGRIC * GRICDifferenceCriterion[interKeyframeIndex-1] - weightPELC * PELCCriterion[interKeyframeIndex-1];
						weightedScore *= correspondenceRatio;
						weightedCriterion[interKeyframeIndex-1] = weightedScore;
						cout << "Keyframe candidate with weighted GRIC-PELC score: " << weightedScore<< endl;

						//select frame with best score
						tmpKeyframe = selectKeyframe(keyframe, GRICDifferenceCriterion, weightedCriterion); //current next keyframe

				} else {
					cout << "No Keyframe candidate: F is probably degenerated" << endl;
				}
			} else {
				if(correspondenceRatio < T_min) {
					cout << "No Keyframe candidate: Baseline too large! " << correspondenceRatio << " < T_min ("<< T_min << ")" << endl;
				}
				if(correspondenceRatio > T_max) {
					cout << "No Keyframe candidate: Baseline too small! "  << correspondenceRatio << " > T_max ("<< T_max << ")" << endl;
				}
			}

		} //end of inter-keyframe loop

		//next keyframe is now determined

		if(!reachedEnd) { //not reached the end of the video yet
			if(tmpKeyframe == 1) {
				cerr << "Error: unable to determine next Keyframe! Try to decrease T_min or increase T_max." << endl;
				delete video;
				exit(0);
			}
			//save keyframe point correspondence i<->i+1
			keyframePointCorrespondences.push_back(interKeyframePointCorrespondences[tmpKeyframe-1]);

			//complete previous three-way point correspondence i<->i+1<->i+2
			if(keyframePointCorrespondences.size() > 1) { //assuming there are two previous point correspondence...
				ThreeWay threeWayPointCorrespondence;
				PointCorrespondences& previousPc = keyframePointCorrespondences.at(keyframePointCorrespondences.size()-2);
				PointCorrespondences& currentPc =  keyframePointCorrespondences.at(keyframePointCorrespondences.size()-1);
				int nPc = findThreeWayCorrespondences(previousPc, currentPc, threeWayPointCorrespondence);
				cout << "\nFound correspondences between Keyframes " << previousPc.getFrameNumber0() << " <-> " << previousPc.getFrameNumber1() <<  " <-> " << currentPc.getFrameNumber1() << ":   " << nPc << endl;
				keyframeThreeWayPointCorrespondences.push_back(threeWayPointCorrespondence);
			}

			keyframe += tmpKeyframe;

			cout << "\n####################################" << endl;
			cout << "#########  "<< "Next Keyframe: " << keyframe << endl;
			cout << "####################################" << endl << endl;
			cout << "New Sequence: " << keyframePointCorrespondences.at(0).getFrameNumber0();
			for(uint k = 0; k < keyframePointCorrespondences.size(); k++) {
				cout << " <-> " << keyframePointCorrespondences.at(k).getFrameNumber1();
			}
			cout << endl << endl;
		} else { //reached end of video
			cout << "End of video reached" << endl;
			if(!lastFrameIsKeyframe) { //last frame of the video is a not a valid keyframe

				//last inter-keyframe point correspondence  will be the last keyframe point correspondences
				keyframePointCorrespondences.push_back(interKeyframePointCorrespondences.back());

				//complete previous three-way point correspondence i<->i+1<->i+2
				if(keyframePointCorrespondences.size() > 0) { //assuming there is a previous point correspondence...
					ThreeWay threeWayPointCorrespondence;
					PointCorrespondences& previousPc = keyframePointCorrespondences.at(keyframePointCorrespondences.size()-2);
					PointCorrespondences& currentPc =  keyframePointCorrespondences.at(keyframePointCorrespondences.size()-1);
					int nPc = findThreeWayCorrespondences(previousPc, currentPc, threeWayPointCorrespondence);
					cout << "\nFound correspondences between last frame and previous keyframe " << previousPc.getFrameNumber0() << " <-> " << previousPc.getFrameNumber1() <<  " <-> " << currentPc.getFrameNumber1() << ":   " << nPc << endl;
					keyframeThreeWayPointCorrespondences.push_back(threeWayPointCorrespondence);
				}else {
					cerr << "There have to be at least three keyframes in the video!" << endl;
				}
			}
			break; //leave keyframe loop
		}

	} //end of keyframe loop


	//build final list of keyframes
	std::vector<int> keyFrameIndices;
	keyFrameIndices.push_back(keyframePointCorrespondences.at(0).getFrameNumber0());
	for(uint k = 0; k < keyframePointCorrespondences.size(); k++) {
		keyFrameIndices.push_back(keyframePointCorrespondences.at(k).getFrameNumber1());
	}
	uint nKeyframes = keyFrameIndices.size();

	//print keyframe-Sequence
	cout << endl;
	cout << "##    ##  ########  ##    ##  ########  ########      ###     ##     ##  ########   ######  " << endl;
	cout << "##   ##   ##         ##  ##   ##        ##     ##    ## ##    ###   ###  ##        ##    ## " << endl;
	cout << "##  ##    ##          ####    ##        ##     ##   ##   ##   #### ####  ##        ##       " << endl;
	cout << "#####     ######       ##     ######    ########   ##     ##  ## ### ##  ######     ######  " << endl;
	cout << "##  ##    ##           ##     ##        ##   ##    #########  ##     ##  ##              ## " << endl;
	cout << "##   ##   ##           ##     ##        ##    ##   ##     ##  ##     ##  ##        ##    ## " << endl;
	cout << "##    ##  ########     ##     ##        ##     ##  ##     ##  ##     ##  ########   ######  " << endl;
	cout << endl;
	cout << "Final Sequence ("<< nKeyframes << " Keyframes): " << keyFrameIndices[0];
	for(uint k = 1; k < nKeyframes; k++) {
		cout << " <-> " << keyFrameIndices[k];
	}
	cout << endl << endl;


	//track all points from keyframe A to keyframe B and refine image coordinates if tracking of a point is lost on the way
	cout << "Track inter-keyframe image coordinates using sparse Lucas-Kanade Optical Flow and remove unfit correspondences" << endl;
	for(uint t = 0; t < keyframeThreeWayPointCorrespondences.size(); t++) {
		ThreeWay& currentThreeWayPc = keyframeThreeWayPointCorrespondences.at(t);
		cout << "Tracking between " << currentThreeWayPc.getStartFrame() << " <-> "<< currentThreeWayPc.getMiddleFrame() << " <-> " << currentThreeWayPc.getEndFrame() << ": " << endl;
		findInterKeyframePointCorrespondences(currentThreeWayPc);
		if(drawThreeWayTracks) {
			drawThreeWayPointCorrespondence(currentThreeWayPc, false);
		}
	}



	 /*#############################################################################################################################
	##    __   __         __       ___  ___             ___                 __              ___  __           __   __   __   ___   ##
	##   /  ` /  \  |\/| |__) |  |  |  |__     | |\ | |  |  |  /\  |       /  `  /\   |\/| |__  |__)  /\  __ |__) /  \ /__` |__    ##
	##   \__, \__/  |  | |    \__/  |  |___    | | \| |  |  | /~~\ |___    \__, /~~\  |  | |___ |  \ /~~\    |    \__/ .__/ |___   ##
	##                                                                                                                             ##
	 #############################################################################################################################*/

	Stereo stereo;
	PointCorrespondences keyframes;
	cv::Mat F;
	cv::Mat E;
	cv::Mat R0 = (cv::Mat_<double>(3,3) << 	1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 );
	cv::Mat R1;
	cv::Mat t0 = (cv::Mat_<double>(3,1) << 	0.0, 0.0, 0.0 );
	cv::Mat t1;
	cv::Mat P0, P1;
	ThreeWay& firstThreeWayPc = keyframeThreeWayPointCorrespondences.front(); //first three way correspondence
	std::vector<cv::Mat> keyFrameRotations; //R for each keyframe
	std::vector<cv::Mat> keyFrameTranslations; //t for each keyframe


	cout << "\nProcessing initial three-way point correspondence: " << firstThreeWayPc.getStartFrame() << " <-> " << firstThreeWayPc.getMiddleFrame() << " <-> " << firstThreeWayPc.getEndFrame() << endl;

	//compute F (again) from remaining point correspondences
	cout << "Compute Fundamental Matrix" << endl;
	F = cv::findFundamentalMat(cv::Mat(firstThreeWayPc.getImageCoordinatesA()), cv::Mat(firstThreeWayPc.getImageCoordinatesB()),	CV_FM_8POINT);

	//compute E from F
	cout << "Compute Essential from Fundamental Matrix" << endl;
	stereo.computeEssentialFromFundamental(K, F, E);

	//compute both camera positions and orientations
	cout << "Compute camera-pose from Essential Matrix" << endl;
	stereo.computeCameraPoseFromEssential(K, E, firstThreeWayPc.getImageCoordinatesA(), firstThreeWayPc.getImageCoordinatesB(), R1, t1, P0, P1); //compute from track endpoints
	firstThreeWayPc.addCamera(P0);
	firstThreeWayPc.addCamera(P1);

	//triangulate word coordinates
	cout << "Triangulate first three-way point correspondence" << endl;
	triangulateThreeWayPc(firstThreeWayPc);


	//calculate reprojection error
	Points3D points0_reprojected, points1_reprojected; //reprojected cartesian image coordinate with depth value
	Points2D points0_error, points1_error; //difference between original and reprojected 2D coordinates
	cv::Point2f avgReprojectionError0, avgReprojectionError1;
	stereo.computeReprojectionError(P0, firstThreeWayPc.getImageCoordinatesA(), firstThreeWayPc.getWorldCoordinates(), points0_reprojected, points0_error, avgReprojectionError0);
	stereo.computeReprojectionError(P1, firstThreeWayPc.getImageCoordinatesB(), firstThreeWayPc.getWorldCoordinates(), points1_reprojected, points1_error, avgReprojectionError1);
	cout << "Average reprojection error: " << avgReprojectionError0 << endl;

	//gain third camera by resectioning (and the rotation/translation vectors of the first two)
	for(int i = 0; i < 3; i++) {
		cv::Mat R;
		cv::Mat t;
		cv::Mat rotationVector;
		cv::Mat NoDistortion;
		cv::solvePnP(	cv::Mat(firstThreeWayPc.getWorldCoordinates()),
						cv::Mat(firstThreeWayPc.getImageCoordinates(i)),
						K,
						NoDistortion, //distCoeffs
						rotationVector,
						t,
						false );

		cv::Rodrigues(rotationVector, R); //convert rotation vector to rotation matrix

		//save rotations/translation for renderer
		keyFrameRotations.push_back(R);
		keyFrameTranslations.push_back(t);

		if(i == 2) { //build third camera projection matrix P for triangulation
			cv::Mat P;
			//projection matrices P = K[R|t]
			cv::hconcat(R, t, P);
			P = K * P;
			firstThreeWayPc.addCamera(P);
		}
	}



	 /*######################################################################################################
	##	 __              ___  __           __   ___  __   __        __  ___  __        __  ___    __         ##
	##	/  `  /\   |\/| |__  |__)  /\     |__) |__  /  ` /  \ |\ | /__`  |  |__) |  | /  `  |  | /  \ |\ |   ##
	##	\__, /~~\  |  | |___ |  \ /~~\    |  \ |___ \__, \__/ | \| .__/  |  |  \ \__/ \__,  |  | \__/ | \|   ##
	##                                                                                                       ##
	 #######################################################################################################*/

	//process chain of three-way correspondences (triangulation and resectioning)
	cout << "\nProcessing chain of all three-way correspondences:" << endl;
	for(uint i = 1; i < keyframeThreeWayPointCorrespondences.size(); i++) {
		ThreeWay& previousThreeWayPc = keyframeThreeWayPointCorrespondences.at(i-1);
		ThreeWay& currentThreeWayPc = keyframeThreeWayPointCorrespondences.at(i);
		cout << "Processing three-way correspondence: " << currentThreeWayPc.getStartFrame() << " <-> " << currentThreeWayPc.getMiddleFrame() << " <-> " << currentThreeWayPc.getEndFrame() << endl;

		//copy last two cameras from previous three-way point correspondence
		currentThreeWayPc.addCamera(previousThreeWayPc.getCamera(1));
		currentThreeWayPc.addCamera(previousThreeWayPc.getCamera(2));

		//determine new set of world coordinates
		cout << "Triangulating: " << currentThreeWayPc.getStartFrame() << " <-> " << currentThreeWayPc.getMiddleFrame() << endl;
		triangulateThreeWayPc(currentThreeWayPc);

		//gain third camera by resectioning
		cout << "Resectioning: " << currentThreeWayPc.getEndFrame() << endl;
		cv::Mat R;
		cv::Mat t;
		cv::Mat rotationVector;
		cv::Mat NoDistortion;
		cv::solvePnP(	cv::Mat(currentThreeWayPc.getWorldCoordinates()),
						cv::Mat(currentThreeWayPc.getImageCoordinates(2)),
						K,
						NoDistortion, //distCoeffs
						rotationVector,
						t,
						false );

		cv::Rodrigues(rotationVector, R); //convert rotation vector to rotation matrix

		//save rotations/translation for renderer
		keyFrameRotations.push_back(R);
		keyFrameTranslations.push_back(t);

		//build third camera projection matrix P for triangulation
		cv::Mat P;
		cv::hconcat(R, t, P);
		P = K * P;
		currentThreeWayPc.addCamera(P);
	}


	//print projection matrices of keyframes
	//printReconstructedCameras(keyframeThreeWayPointCorrespondences, keyFrameRotations, keyFrameTranslations, keyFrameIndices);


	 /*#############################################################################################################################
	##          ___  ___  __           ___      ___  __              ___     __   ___  __   ___  __  ___    __               __    ##
	##   | |\ |  |  |__  |__) __ |__/ |__  \ / |__  |__)  /\   |\/| |__     |__) |__  /__` |__  /  `  |  | /  \ |\ | | |\ | / _`   ##
	##   | | \|  |  |___ |  \    |  \ |___  |  |    |  \ /~~\  |  | |___    |  \ |___ .__/ |___ \__,  |  | \__/ | \| | | \| \__>   ##
	##                                                                                                                             ##
	 #############################################################################################################################*/

	std::vector<cv::Mat> OpenGLRotations; //R for all frames (keyframes + intermediate)
	std::vector<cv::Mat> OpenGLTranslations; //t for all frames (keyframes + intermediate)
	cout << "\nComputing all inter-keyframe cameras:" << endl;
	uint nThreeWayPcs = keyframeThreeWayPointCorrespondences.size();
	for(uint t = 0; t < nThreeWayPcs; t++) {
		//add cameras for openGL renderer sequential order
		OpenGLRotations.push_back(keyFrameRotations.at(t));
		OpenGLTranslations.push_back(keyFrameTranslations.at(t));

		ThreeWay& currentThreeWayPc = keyframeThreeWayPointCorrespondences.at(t);

		//perform resectioning of cameras between keyframe A and keyframe B
		cout << "Resectioning cameras between keyframes: " << currentThreeWayPc.getStartFrame() << " <-> " << currentThreeWayPc.getMiddleFrame() << endl;;
		determineInterKeyframeCameras(currentThreeWayPc, K, OpenGLRotations, OpenGLTranslations, false);
	}

	//add camera second to last
	OpenGLRotations.push_back(keyFrameRotations.at(nThreeWayPcs));
	OpenGLTranslations.push_back(keyFrameTranslations.at(nThreeWayPcs));
	//perform resectioning of cameras between keyframe B and keyframe C of the last threeway point correspondence
	ThreeWay& lastThreeWayPc = keyframeThreeWayPointCorrespondences.back();
	cout << "Resectioning cameras between the last two keyframes: " << lastThreeWayPc.getMiddleFrame() << " <-> " << lastThreeWayPc.getEndFrame() << endl;;
	determineInterKeyframeCameras(lastThreeWayPc, K, OpenGLRotations, OpenGLTranslations, true);
	//add last camera
	OpenGLRotations.push_back(keyFrameRotations.at(nThreeWayPcs+1));
	OpenGLTranslations.push_back(keyFrameTranslations.at(nThreeWayPcs+1));



	cout << "\nAll Cameras reconstructed!" << endl;

	 /*################################################
	##	  __   ___       __   ___  __          __     ##
	##	 |__) |__  |\ | |  \ |__  |__) | |\ | / _`    ##
	##	 |  \ |___ | \| |__/ |___ |  \ | | \| \__>    ##
	##                                                ##
	 ################################################*/

	//combine different world coordinates
	std::vector<Points3D> combinedWorldCoordinates;
	for(uint t = 0; t < keyframeThreeWayPointCorrespondences.size(); t++) {
		ThreeWay& currentThreeWayPc = keyframeThreeWayPointCorrespondences.at(t);
		combinedWorldCoordinates.push_back(currentThreeWayPc.getWorldCoordinates());
	}

	//instantiate our OpenGL based renderer
	renderer = new Renderer(width, height, "Teapot");
	renderer->setCameraTrack(keyFrameRotations, keyFrameTranslations); //tell the renderer about the various camera poses

	//place virtual object
	cv::Point3f objectPosition; //world coordinates of superimposed virtual object
	determineObjectPosition(combinedWorldCoordinates, objectPosition);
	renderer->setScaleFactor(objectPosition.z/75.0);
	renderer->setObjectPosition(objectPosition, combinedWorldCoordinates);

	//setup virtual camera parameters
	renderer->setViewFrustum(K);
	cv::Mat cameraPosition = (cv::Mat_<double>(3,1) << -objectPosition.z, -objectPosition.z, -objectPosition.z);
	renderer->setFreeViewCameraPose(keyFrameRotations[0], keyFrameTranslations[0], cameraPosition, -25.0, 20.0, 0.0); //just the initial free view opengl camera

	//visualize reconstruction
	cout << "Showing reconstructed scene..." << endl;
	showPreview(keyFrameRotations, keyFrameTranslations, keyFrameIndices);

	//finally render teapot onto frames
	cout << "Rendering frames..." << endl;
	renderSequence(OpenGLRotations, OpenGLTranslations);

	delete video;
	delete renderer;
	cout << "Done!" << endl;
}


int Multiview::findThreeWayCorrespondences(PointCorrespondences& previousPc, PointCorrespondences& currentPc, ThreeWay& resultingThreeWay) {
	float matchTolerance = 0.1; //could theoretica3lly be 0.0 or in the range of floating point precision.
	int frameA = previousPc.getFrameNumber0(); //frame index number
	int frameB = previousPc.getFrameNumber1();
	int frameC = currentPc.getFrameNumber1();
	Points2D coordinatesA; //part of the final point correspondence
	Points2D coordinatesB;
	Points2D coordinatesC;

	for(uint m = 0; m < previousPc.getImageCoordinates1().size(); m++) {
		for(uint n = 0; n < currentPc.getImageCoordinates0().size(); n++) {
			if(    fabs(previousPc.getImageCoordinates1()[m].x - currentPc.getImageCoordinates0()[n].x) < matchTolerance
				&& fabs(previousPc.getImageCoordinates1()[m].y - currentPc.getImageCoordinates0()[n].y) < matchTolerance ) { //valid point correspondence

				cv::Point2f& coordinateFrameA = previousPc.getImageCoordinates0()[m];
				cv::Point2f& coordinateFrameB = previousPc.getImageCoordinates1()[m];
				cv::Point2f& coordinateFrameC = currentPc.getImageCoordinates1()[n];

				coordinatesA.push_back(coordinateFrameA);
				coordinatesB.push_back(coordinateFrameB);
				coordinatesC.push_back(coordinateFrameC);
			}
		}
	}

	resultingThreeWay.addCorrespondence(frameA, coordinatesA);
	resultingThreeWay.addCorrespondence(frameB, coordinatesB);
	resultingThreeWay.addCorrespondence(frameC, coordinatesC);

	return coordinatesA.size(); //number of three-way point-correspondences from A to B to C
}


void Multiview::triangulateThreeWayPc(ThreeWay& threeWayPc) {
	Stereo stereo;
	Points3D worldCoordinates;

	stereo.triangulate(threeWayPc.getCamera(0), threeWayPc.getCamera(1), threeWayPc.getImageCoordinatesA(), threeWayPc.getImageCoordinatesB(), worldCoordinates);

	//report coordinates that appear in the back of the camera (possible outliers)
	for(uint i = 0; i < worldCoordinates.size(); i++) {
		 if(worldCoordinates[i].z < 0.0) {
			 cout << "World point "<< i << " behind camera!!!" << endl;
		 }
	}

	threeWayPc.setWorldCoordinates(worldCoordinates);
}


int Multiview::selectKeyframe(int lastKeyframe, std::vector<double>& GRICDifferenceCriterion, std::vector<double>& weightedCriterion) {

	//find max GRIC Difference value
	std::vector<double>::iterator maxGRICDifferenceElement;
	maxGRICDifferenceElement = std::max_element(GRICDifferenceCriterion.begin(), GRICDifferenceCriterion.end());
	int gricDifferenceKeyframe = std::distance(GRICDifferenceCriterion.begin(), maxGRICDifferenceElement);

	//find max Weighted Criterion value
	std::vector<double>::iterator maxWeightedCriterionElement;
	maxWeightedCriterionElement = std::max_element(weightedCriterion.begin(), weightedCriterion.end());
	int weightedGricPelcKeyframe = std::distance(weightedCriterion.begin(), maxWeightedCriterionElement);

	std::cout << "Next keyframe according to GRIC Difference:              " << lastKeyframe+1 + gricDifferenceKeyframe << endl;
	std::cout << "Next keyframe according to weighted GRIC-PELC Criterion: " << lastKeyframe+1 + weightedGricPelcKeyframe << endl;

	//int finalKeyframe = 1 + std::max(gricDifferenceKeyframe, weightedGricPelcKeyframe);
	int finalKeyframe = 1 + weightedGricPelcKeyframe;

	if(GRICDifferenceCriterion[weightedGricPelcKeyframe] <= 0) { //GRICDifferenceCriterion must be > 0 or F will be degenerated
		cerr << "GRIC Difference Criterion < 0 => F will be degenerated" << endl;
	}

	return finalKeyframe;
}


void Multiview::findInterKeyframePointCorrespondences(ThreeWay& threeWayPc) {
	// Example configuration:
	//  ___                          ___                                  ___
	// | A |     _      _      _    | B |     _      _      _      _     | C |
	// |___|    |_|    |_|    |_|   |___|    |_|    |_|    |_|    |_|    |___|
	//   |                            |                                    |
	//   |--  keyframeDistance0: 3  --|--      keyframeDistance1: 4      --|


	uint nkeypoints = threeWayPc.getImageCoordinatesA().size();
	uint keyframeDistance0 = threeWayPc.getMiddleFrame() - threeWayPc.getStartFrame() - 1; //see ascii-art ;-)
	uint keyframeDistance1 = threeWayPc.getEndFrame() - threeWayPc.getMiddleFrame() - 1;

	std::vector<Points2D> imageCoordinatesLK0(keyframeDistance0+2); //image coordinates including surrounding keyframes
	imageCoordinatesLK0[0] = threeWayPc.getImageCoordinatesA(); //initialize left boundary
	std::vector<Points2D> imageCoordinatesLK1(keyframeDistance1+2); //image coordinates including surrounding keyframes
	imageCoordinatesLK1[0] = threeWayPc.getImageCoordinatesB(); //initialize left boundary

	std::vector<bool> validPoints(nkeypoints, true); //index vector of correctly tracked points
	//now try to track the intermediate "short baseline" frames with optical flow
	cv::Mat tmp, previousFrame, currentFrame;
	std::vector<uchar> status; //status of tracked features
	std::vector<float> err; //error in tracking

	//track A -> B
	for(uint i = 0; i < keyframeDistance0+1; i++) {
		if (i == 0) { //for the first frame of the sequence
			video->getFrame(threeWayPc.getStartFrame(), tmp);
			cv::cvtColor(tmp, previousFrame, CV_BGR2GRAY); //convert to gray-scale
		}

		video->getFrame(threeWayPc.getStartFrame()+i+1, tmp);
		cv::cvtColor(tmp, currentFrame, CV_BGR2GRAY);	//convert to gray-scale

		//track interest points in second frame, given its position in the first frame
		//tracked position in current iteration will be start position in the next iteration
		cv::calcOpticalFlowPyrLK(	previousFrame, currentFrame,
									imageCoordinatesLK0[i], //input point positions in first image
									imageCoordinatesLK0[i+1], //output point positions in the 2nd image
									status, //optical flow detection success
									err);	//tracking error


		//mark lost tracks to be removed
		for (uint j = 0; j < nkeypoints; j++) {
			//only keep points that survived cv::calcOpticalFlowPyrLK
			if (!status[j]) {
				validPoints[j] = false;
				//cout << "Feature point " << j << " in inter-keyframe " << i << " removed!" << endl;
			}
		}

		//draw tracked points
		if(drawTracks) {
			std::stringstream threeWayIdentifier;
			threeWayIdentifier << threeWayPc.getStartFrame() << "-" << threeWayPc.getMiddleFrame() << "-" << threeWayPc.getEndFrame();
			drawTrackedPoints(threeWayPc.getStartFrame(), i, previousFrame, imageCoordinatesLK0, threeWayIdentifier.str(), false);
		}
		//current frame will be the previous one in the next step
		cv::swap(previousFrame, currentFrame);
	}

	//track B -> C
	for(uint i = 0; i < keyframeDistance1+1; i++) {
		if (i == 0) { //for the first frame of the sequence
			video->getFrame(threeWayPc.getMiddleFrame(), tmp);
			cv::cvtColor(tmp, previousFrame, CV_BGR2GRAY); //convert to gray-scale
		}

		video->getFrame(threeWayPc.getMiddleFrame()+i+1, tmp);
		cv::cvtColor(tmp, currentFrame, CV_BGR2GRAY);	//convert to gray-scale

		//track interest points in second frame, given its position in the first frame
		//tracked position in current iteration will be start position in the next iteration
		cv::calcOpticalFlowPyrLK(	previousFrame, currentFrame,
									imageCoordinatesLK1[i], //input point positions in first image
									imageCoordinatesLK1[i+1], //output point positions in the 2nd image
									status, //optical flow detection success
									err);	//tracking error


		//mark lost tracks to be removed
		for (uint j = 0; j < nkeypoints; j++) {
			//only keep points that survived cv::calcOpticalFlowPyrLK
			if (!status[j]) {
				validPoints[j] = false;
				//cout << "Feature point " << j << " in inter-keyframe " << i << " removed!" << endl;
			}
		}

		//draw tracked points
		if(drawTracks) {
			std::stringstream threeWayIdentifier;
			threeWayIdentifier << threeWayPc.getStartFrame() << "-" << threeWayPc.getMiddleFrame() << "-" << threeWayPc.getEndFrame();
			drawTrackedPoints(threeWayPc.getMiddleFrame(), i, previousFrame, imageCoordinatesLK1, threeWayIdentifier.str(), false);
		}

		//current frame will be the previous one in the next step
		cv::swap(previousFrame, currentFrame);
	}



	//check if optical flow points starting from keyframe A arrived at their known "destination" in keyframe B
	for(uint j = 0; j < nkeypoints; j++) {
		if(validPoints[j]) { //only consider valid points (as determined by calcOpticalFlowPyrLK)
			float distanceX = fabs(imageCoordinatesLK0[keyframeDistance0+1][j].x - threeWayPc.getImageCoordinatesB()[j].x);
			float distanceY = fabs(imageCoordinatesLK0[keyframeDistance0+1][j].y - threeWayPc.getImageCoordinatesB()[j].y);

			if(distanceX > maxDrift || distanceY > maxDrift) { // bad tracking
				//cout << "Removing image point: " << std::setfill(' ') << std::setw(5) << j << "  Optical Flow drift A -> B: [" << distanceX << ", " << distanceY << "]"<< endl;
				validPoints[j] = false;
			}
		}
	}
	//check if optical flow points starting from keyframe B arrived at their known "destination" in keyframe C
	for(uint j = 0; j < nkeypoints; j++) {
		if(validPoints[j]) { //only consider valid points (as determined by calcOpticalFlowPyrLK)
			float distanceX = fabs(imageCoordinatesLK1[keyframeDistance1+1][j].x - threeWayPc.getImageCoordinatesC()[j].x);
			float distanceY = fabs(imageCoordinatesLK1[keyframeDistance1+1][j].y - threeWayPc.getImageCoordinatesC()[j].y);

			if(distanceX > maxDrift || distanceY > maxDrift) { // bad tracking
				//cout << "Removing image point: " << std::setfill(' ') << std::setw(5) << j << "  Optical Flow drift B -> C: [" << distanceX << ", " << distanceY << "]"<< endl;
				validPoints[j] = false;
			}
		}
	}


	int validCounter = 0;
	for(uint j = 0; j < nkeypoints; j++) {
		if(validPoints[j]) {
			validCounter++;
		}
	}

	cout << "Tracked points:         " << nkeypoints << endl;
	cout << "Valid points remaining: " << validCounter << "\t(" << 100.0 * ((float)validCounter/(float)nkeypoints) << "%)" << endl << endl;
	if(validCounter < 8) { //at least 8 points are needed for camera reconstruction
		cout << "Error: Tracking lost! Not enough point correspondences for camera reconstruction :-(" << endl << endl;
		exit(0);
	}

	//Now remove lost tracks from image coordinates...

	//update image coordinates in keyframes A, B and C of three-way point correspondence
	Points2D imageCoordinatesA_refined; //remaining image coordinates in Keyframe A after removal of lost tracks
	Points2D imageCoordinatesB_refined; //remaining image coordinates in Keyframe B after removal of lost tracks
	Points2D imageCoordinatesC_refined; //remaining image coordinates in Keyframe C after removal of lost tracks
	for(uint j = 0; j < nkeypoints; j++) {
		if(validPoints[j]) {
			imageCoordinatesA_refined.push_back(threeWayPc.getImageCoordinatesA()[j]);
			imageCoordinatesB_refined.push_back(threeWayPc.getImageCoordinatesB()[j]);
			imageCoordinatesC_refined.push_back(threeWayPc.getImageCoordinatesC()[j]);
		}
	}
	threeWayPc.setKeyframeImageCoordinates(0, imageCoordinatesA_refined);
	threeWayPc.setKeyframeImageCoordinates(1, imageCoordinatesB_refined);
	threeWayPc.setKeyframeImageCoordinates(2, imageCoordinatesC_refined);


	//update inter-keyframe image coordinates of three-way point correspondence
	//A -> B
	for(uint i = 1; i < keyframeDistance0+1; i++) {
		Points2D interKeyframeImageCoordinates0_refined; //remaining image coordinates after removal of lost tracks
		for(uint j = 0; j < nkeypoints; j++) { //build list of valid image coordinates
			if(validPoints[j]) {
				interKeyframeImageCoordinates0_refined.push_back(imageCoordinatesLK0[i][j]);
			}
		}
		threeWayPc.addInterKeyframeImageCoordinates0(interKeyframeImageCoordinates0_refined);
	}
	//B -> C
	for(uint i = 1; i < keyframeDistance1+1; i++) {
		Points2D interKeyframeImageCoordinates1_refined; //remaining image coordinates after removal of lost tracks
		for(uint j = 0; j < nkeypoints; j++) { //build list of valid image coordinates
			if(validPoints[j]) {
				interKeyframeImageCoordinates1_refined.push_back(imageCoordinatesLK1[i][j]);
			}
		}
		threeWayPc.addInterKeyframeImageCoordinates1(interKeyframeImageCoordinates1_refined);
	}

}


void Multiview::determineInterKeyframeCameras(ThreeWay& threeWay, cv::Mat& K, std::vector<cv::Mat>& OpenGLRotations, std::vector<cv::Mat>& OpenGLTranslations, bool isLast) {

	if(!isLast) { //in case of a "normal" three way point correspondence: resection cameras between A <-> B

	uint keyframeDistance = threeWay.getMiddleFrame() - threeWay.getStartFrame() - 1;

	//gain all cameras within keyframe A and B of the three way point correspondence by resectioning
	for(uint i = 0; i < keyframeDistance; i++) {
		cv::Mat R;
		cv::Mat t;
		cv::Mat rotationVector;
		cv::Mat NoDistortion;
		cv::solvePnP(	cv::Mat(threeWay.getWorldCoordinates()),
						cv::Mat(threeWay.getInterKeyframeImageCoordinates0(i)),
						K,
						NoDistortion, //distCoeffs
						rotationVector,
						t,
						false );

		cv::Rodrigues(rotationVector, R); //convert rotation vector to rotation matrix

		//save rotations/translation for renderer
		OpenGLRotations.push_back(R);
		OpenGLTranslations.push_back(t);
	}

	} else { //in case of the last three way point correspondence: resection cameras between B <-> C
		uint keyframeDistance = threeWay.getEndFrame() - threeWay.getMiddleFrame() - 1;

		//gain all cameras within keyframe A and B of the three way point correspondence by resectioning
		for(uint i = 0; i < keyframeDistance; i++) {
			cv::Mat R;
			cv::Mat t;
			cv::Mat rotationVector;
			cv::Mat NoDistortion;
			cv::solvePnP(	cv::Mat(threeWay.getWorldCoordinates()), //shared by all cameras
							cv::Mat(threeWay.getInterKeyframeImageCoordinates1(i)), //the image coordinates between keyframes B <-> C
							K,
							NoDistortion, //distCoeffs,
							rotationVector,
							t,
							false );

			cv::Rodrigues(rotationVector, R); //convert rotation vector to rotation matrix

			//save rotations/translation for renderer
			OpenGLRotations.push_back(R);
			OpenGLTranslations.push_back(t);
		}
	}


}


void Multiview::readCameraInstrinics(const string& filename, cv::Mat& K, cv::Mat& distCoeffs) {
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	fs["IntrinsicCameraMatrix"] >> K;
	fs["DistortionCoefficients"] >> distCoeffs;
	fs.release();
	cout << "Camera matrix imported" << endl;
}


void Multiview::determineObjectPosition(std::vector<Points3D> worldCoordinates, cv::Point3f& objectPosition) {

	//The translation vector t of the second camera is known only up to scale.
	//As a consequence, the coordinates of the reconstructed 3D points are only relative.
	//In order to position the teapot into the scene, a scale factor has to be found that normalizes the coordinates
	//solution: place teapot at (0, 0, median(z))
	//			the scale factor is then related to the teapot's position

	std::vector<float> z_coordinates;
	for(uint i = 0; i < worldCoordinates.size(); i++) {
		if(worldCoordinates[0][i].z < 0.0) { //there are outliers or the camera configuration is wrong
			cout << "Degenerated!!! World point behind Camera\n";
			exit(0); //TODO: go back to start or try at least a clean shutdown
		}
		//collect z coordinates into separate container
		z_coordinates.push_back(worldCoordinates[0][i].z);
	}

	//determine median of z coordinates (dont'let outliers destroy the teapot's scale)
	std::vector<float>::iterator first = z_coordinates.begin();
	std::vector<float>::iterator last = z_coordinates.end();
	std::vector<float>::iterator upperMedian = first + (last - first) / 2; //= median of odd number of elements
	std::vector<float>::iterator lowerMedian = first + (last - first) / 2 - 1;

	float median_z;
	if(z_coordinates.size() % 2 == 0) { //even length
		std::nth_element(first, upperMedian, last);
		std::nth_element(first, lowerMedian, last);
		median_z = 0.5*(*upperMedian + *lowerMedian);
	} else { //odd length
		std::nth_element(first, upperMedian, last);
		median_z = *upperMedian;
	}

	objectPosition.x = 0;
	objectPosition.y = 0;
	objectPosition.z = median_z/2;  //max_z/2;

	cout << "Median of world points z-coordinate: " << median_z << "  Scale factor: " << objectPosition.z/75.0 << endl;
}


void Multiview::showPreview(std::vector<cv::Mat>& rotations,
							std::vector<cv::Mat>& translations,
							std::vector<int>& frameIndices)
{
	//show preview
	while(renderer->preview) {
		cv::Mat previewImage;
		video->getFrame(frameIndices[renderer->requestedFrameNumber], previewImage); //get background frame

		//draw frame number onto frame
		std::stringstream message;
		message << "Camera " << frameIndices[renderer->requestedFrameNumber];
		cv::Point2f messagePosition(10.0, 25.0);
		cv::putText(previewImage, message.str(), messagePosition, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2); //outline
		cv::putText(previewImage, message.str(), messagePosition, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1); //inner font

		//update lower left viewport of the renderer
		renderObject(rotations[renderer->requestedFrameNumber], translations[renderer->requestedFrameNumber], previewImage);
	}
}


void Multiview::renderObject(cv::Mat& R, cv::Mat& t, cv::Mat& background) {

	cv::Mat renderedImage(height, width, CV_8UC3);
	cv::Mat mask(height, width, CV_8UC1); //one channel mask

	renderer->setCameraPose(R, t); //place camera into scene
	cv::flip(background, background, 0); //flip image as origin is bottom left in OpenGL and top left in OpenCV
	renderer->render(background); //render object to SDL screen

	//read pixels from SDL opengl context and convert to opencv image
	glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, (uchar*)renderedImage.data); //grab OpenGL frame buffer and store it in OpenCV image
	cv::flip(renderedImage, renderedImage, 0); //flip image as origin is bottom left in OpenGL and top left in OpenCV
	background = renderedImage;


// Old approach: black background in OpenGL context is replaced by the video frame, the "cut out" rendered object is used as an overlay
//	//create an overlay mask
//	cv::Mat_<uchar>::iterator it1 = mask.begin<uchar>();	//single channel image iterator
//	cv::Mat_<cv::Vec3b>::iterator it3 = renderedImage.begin<cv::Vec3b>(); //3 channel image iterator
//	cv::Mat_<cv::Vec3b>::iterator it3end = renderedImage.end<cv::Vec3b>();
//	for (; it3 != it3end; ++it3) {
//		if( (*it3)[0] == 0 && (*it3)[1] == 0 && (*it3)[2] == 0 ) { //black background pixel
//			(*it1) = 0xFF; //should be transparent
//		} else { //teapot pixel
//			(*it1) = 0x00; //leave it alone
//		}
//		++it1;
//	}
//
//	//combine rendered image (masked object) with video frame
//	cv::add(renderedImage, composite, renderedImage, mask); //if(mask[i]) => tea[i] = tea[i] + frame[i]
//	composite = renderedImage;
}


void Multiview::renderSequence(std::vector<cv::Mat>& OpenGLRotations, std::vector<cv::Mat>& OpenGLTranslations) {

	cv::Mat currentFrame;
	std::stringstream imageFilename;

	//run through video and render object onto frames
	for(uint i = 0; i < OpenGLRotations.size(); i++) {

		video->getFrame(i, currentFrame);

		cv::Mat R = OpenGLRotations.at(i);
		cv::Mat t = OpenGLTranslations.at(i);

		renderObject(R, t, currentFrame);

		//save frame to video file
		video->writeNextFrame(currentFrame);
	}
}


void Multiview::drawTrackedPoints(uint startframe, uint currentPosition, cv::Mat& tmp, std::vector<Points2D> imageCoordinatesLK, const std::string& twpcIdentifier, bool label) {
	cv::Mat currentFrame;
	cv::cvtColor(tmp, currentFrame, CV_GRAY2BGR);

	uint limit = currentPosition;
	for(uint i = 0; i < imageCoordinatesLK[0].size(); i++) { //for all tracks
		for(uint j = 0; j < currentPosition; j++) { //for all previous frames
			for(uint k = 1; k <= limit; k++) { //for all intermediate steps
				cv::line(currentFrame, imageCoordinatesLK[k-1][i], imageCoordinatesLK[k][i], cv::Scalar(128, 255, 128), 1, 8, 0); //green
			}
		}
		cv::circle(currentFrame, imageCoordinatesLK[0][i], 3, cv::Scalar(128, 128, 255), 1); //origin (red)
		cv::circle(currentFrame, imageCoordinatesLK[currentPosition][i], 3, cv::Scalar(128, 255, 128), 1); //temporal end point (green)

		//print track label
		if(label) {
			std::stringstream trackLabel;
			trackLabel << i;
			cv::Point2f trackLabelPos =  imageCoordinatesLK[0][i];
			cv::putText(currentFrame, trackLabel.str(), trackLabelPos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2); //outline
			cv::putText(currentFrame, trackLabel.str(), trackLabelPos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1); //inner
		}
	}

	if(trackRecorder.isOpened()) {
		trackRecorder << currentFrame;
	}

	//save frame as image
	std::stringstream imageFilename;
	imageFilename.str(""); //clear stringstream
	imageFilename << inputBasename << "_tracked_"<< twpcIdentifier << "_frame_" << std::setfill('0') << std::setw(5) << startframe+currentPosition << ".png";
	cv::imwrite(imageFilename.str(), currentFrame);
}


void Multiview::drawThreeWayPointCorrespondence(ThreeWay& threeWay, bool label) {

//		cout << "Image coordinates of frame: " << threeWay.getStartFrame() << endl << threeWay.getImageCoordinatesA() << endl<< endl;
//		cout << "Image coordinates of frame: " << threeWay.getMiddleFrame() << endl << threeWay.getImageCoordinatesB() << endl<< endl;
//		cout << "Image coordinates of frame: " << threeWay.getEndFrame() << endl << threeWay.getImageCoordinatesC() << endl<< endl;

		cv::Mat tmpImageCoordinates, imageCoordinatesA, imageCoordinatesB, imageCoordinatesC;
		video->getFrame(threeWay.getStartFrame(), tmpImageCoordinates);
		cv::cvtColor(tmpImageCoordinates, imageCoordinatesA, CV_BGR2GRAY);
		cv::cvtColor(imageCoordinatesA, imageCoordinatesA, CV_GRAY2BGR);

		video->getFrame(threeWay.getMiddleFrame(), tmpImageCoordinates);
		cv::cvtColor(tmpImageCoordinates, imageCoordinatesB, CV_BGR2GRAY);
		cv::cvtColor(imageCoordinatesB, imageCoordinatesB, CV_GRAY2BGR);

		video->getFrame(threeWay.getEndFrame(), tmpImageCoordinates);
		cv::cvtColor(tmpImageCoordinates, imageCoordinatesC, CV_BGR2GRAY);
		cv::cvtColor(imageCoordinatesC, imageCoordinatesC, CV_GRAY2BGR);

		for(uint i = 0; i < threeWay.getImageCoordinatesA().size(); i++) {
			cv::Point2f A = threeWay.getImageCoordinatesA()[i];
			cv::Point2f B = threeWay.getImageCoordinatesB()[i];
			cv::Point2f C = threeWay.getImageCoordinatesC()[i];
//			cv::Point2f diffAB = cv::Point2f(A-B);
//			cv::Point2f diffBC = cv::Point2f(B-C);
//			cout <<  setprecision(5) << setw(5) << i << " A: " << A << " B: " << B << " C: " << C << "  Diff A<->B: " << diffAB << "  Diff B<->C: " << diffBC << endl;

			cv::circle(imageCoordinatesA, A, 3, cv::Scalar(128, 128, 255), 1); //red

			cv::circle(imageCoordinatesB, B, 3, cv::Scalar(128, 255, 128), 1); //green

			cv::circle(imageCoordinatesC, C, 3, cv::Scalar(255, 255, 128), 1); //aqua

			//point label
			if(label) {
				std::stringstream trackLabel;
				trackLabel << i;
				cv::Point2f trackLabelPos =  A;
				cv::putText(imageCoordinatesA, trackLabel.str(), trackLabelPos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2); //outline
				cv::putText(imageCoordinatesA, trackLabel.str(), trackLabelPos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1); //inner
				cv::putText(imageCoordinatesB, trackLabel.str(), trackLabelPos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2); //outline
				cv::putText(imageCoordinatesB, trackLabel.str(), trackLabelPos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1); //inner
				cv::putText(imageCoordinatesC, trackLabel.str(), trackLabelPos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2); //outline
				cv::putText(imageCoordinatesC, trackLabel.str(), trackLabelPos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1); //inner
			}
		}

		//save frame
		std::stringstream imageFilename;
		imageFilename.str(""); //clear stringstream
		imageFilename << inputBasename << " - three-way point correspondence - " << threeWay.getStartFrame() << " - " << threeWay.getMiddleFrame() << " - " << threeWay.getEndFrame() << " - A.png";
		cv::imwrite(imageFilename.str(), imageCoordinatesA);
		imageFilename.str(""); //clear stringstream
		imageFilename << inputBasename << " - three-way point correspondence - " << threeWay.getStartFrame() << " - " << threeWay.getMiddleFrame() << " - " << threeWay.getEndFrame() << " - B.png";
		cv::imwrite(imageFilename.str(), imageCoordinatesB);
		imageFilename.str(""); //clear stringstream
		imageFilename << inputBasename << " - three-way point correspondence - " << threeWay.getStartFrame() << " - " << threeWay.getMiddleFrame() << " - " << threeWay.getEndFrame() << " - C.png";
		cv::imwrite(imageFilename.str(), imageCoordinatesC);

}


void Multiview::printReconstructedCameras(std::vector<ThreeWay>& keyframeThreeWayPointCorrespondences, std::vector<cv::Mat>& keyFrameRotations, std::vector<cv::Mat>& keyFrameTranslations, std::vector<int>& keyFrameIndices) {
	std::stringstream projectionMatrixName;
	for(uint t = 0; t < keyframeThreeWayPointCorrespondences.size(); t++) {
		ThreeWay& currentThreeWayPc = keyframeThreeWayPointCorrespondences.at(t);
		cout << "\nCameras in Keyframes: " << currentThreeWayPc.getStartFrame() << " <-> " << currentThreeWayPc.getMiddleFrame() << " <-> " << currentThreeWayPc.getEndFrame() << endl;
		projectionMatrixName << "\nProjection Matrix: " << currentThreeWayPc.getStartFrame() << endl;
		Utils::printMatrix(currentThreeWayPc.getCamera(0), projectionMatrixName.str());
		projectionMatrixName.str(""); //clear stringstream

		projectionMatrixName << "\nProjection Matrix: " << currentThreeWayPc.getMiddleFrame() << endl;
		Utils::printMatrix(currentThreeWayPc.getCamera(1), projectionMatrixName.str());
		projectionMatrixName.str(""); //clear stringstream

		projectionMatrixName << "\nProjection Matrix: " << currentThreeWayPc.getEndFrame() << endl;
		Utils::printMatrix(currentThreeWayPc.getCamera(2), projectionMatrixName.str());
		projectionMatrixName.str(""); //clear stringstream
	}
	//print opengl rotation matrices
	for (uint i = 0; i < keyFrameRotations.size(); i++) {
		cout << "OpenGL Rotation Matrix: " << keyFrameIndices[i] << endl;
		cout << keyFrameRotations[i] << endl << endl;
	}
	//print opengl translation vectors
	for (uint i = 0; i < keyFrameTranslations.size(); i++) {
		cout << "OpenGL Translation Vector: " << keyFrameIndices[i] << endl;
		cout << keyFrameTranslations[i] << endl << endl;
	}
}


/**
 * @class Vertex
 * @brief Simple vertex class for self-made wire-frame cube
 */
class Vertex {
	cv::Scalar color;
	cv::Point3d worldcoordinate;
	cv::Point2d imagecoordinate;

public:
	Vertex(cv::Point3d coord) {color = cv::Scalar(255, 255, 255); worldcoordinate = coord;} //default color is white
	Vertex(cv::Scalar col, cv::Point3d coord) {color = col; worldcoordinate = coord;}

	cv::Scalar getColor() {return color;}
	double getWorldX() {return worldcoordinate.x; }
	double getWorldY() {return worldcoordinate.y; }
	double getWorldZ() {return worldcoordinate.z; }

	cv::Point2d& getImageCoodinate() { return imagecoordinate; }
	double getImageX() {return imagecoordinate.x; }
	double getImageY() {return imagecoordinate.y; }

	void setImageX(double x) {imagecoordinate.x = x; }
	void setImageY(double y) {imagecoordinate.y = y; }
};


void Multiview::drawCube(cv::Mat& P, Points3D& X, cv::Mat& image) {
	typedef std::vector<Vertex> Polygon;
	std::vector<Polygon> objects;

	//construct wireframe cube
	Polygon front;
	front.push_back(Vertex(cv::Scalar(32, 32, 255), cv::Point3d( 1, 1, -1))); //red
	front.push_back(Vertex(cv::Scalar(32, 32, 255), cv::Point3d( 1,-1, -1)));
	front.push_back(Vertex(cv::Scalar(32, 32, 255), cv::Point3d(-1,-1, -1)));
	front.push_back(Vertex(cv::Scalar(32, 32, 255), cv::Point3d(-1, 1, -1)));
	objects.push_back(front);

	Polygon back;
	back.push_back(Vertex(cv::Scalar(255, 255, 32), cv::Point3d( 1,  1,  1))); //aqua
	back.push_back(Vertex(cv::Scalar(255, 255, 32), cv::Point3d(-1,  1,  1)));
	back.push_back(Vertex(cv::Scalar(255, 255, 32), cv::Point3d(-1, -1,  1)));
	back.push_back(Vertex(cv::Scalar(255, 255, 32), cv::Point3d( 1, -1,  1)));
	objects.push_back(back);

	Polygon left;
	left.push_back(Vertex(cv::Scalar(255, 32, 255), cv::Point3d(-1, -1, -1))); //pink
	left.push_back(Vertex(cv::Scalar(255, 32, 255), cv::Point3d(-1, -1,  1)));
	left.push_back(Vertex(cv::Scalar(255, 32, 255), cv::Point3d(-1,  1,  1)));
	left.push_back(Vertex(cv::Scalar(255, 32, 255), cv::Point3d(-1,  1, -1)));
	objects.push_back(left);

	Polygon right;
	right.push_back(Vertex(cv::Scalar(255, 32, 32), cv::Point3d( 1,  1, -1))); //blue
	right.push_back(Vertex(cv::Scalar(255, 32, 32), cv::Point3d( 1,  1,  1)));
	right.push_back(Vertex(cv::Scalar(255, 32, 32), cv::Point3d( 1, -1,  1)));
	right.push_back(Vertex(cv::Scalar(255, 32, 32), cv::Point3d( 1, -1, -1)));
	objects.push_back(right);

	Polygon top;
	top.push_back(Vertex(cv::Scalar(32, 255, 32), cv::Point3d( 1, -1, -1))); //green
	top.push_back(Vertex(cv::Scalar(32, 255, 32), cv::Point3d( 1, -1,  1)));
	top.push_back(Vertex(cv::Scalar(32, 255, 32), cv::Point3d(-1, -1,  1)));
	top.push_back(Vertex(cv::Scalar(32, 255, 32), cv::Point3d(-1, -1, -1)));
	objects.push_back(top);

	Polygon bottom;
	bottom.push_back(Vertex(cv::Scalar(32, 255, 255), cv::Point3d( 1,  1,  1))); //yellow
	bottom.push_back(Vertex(cv::Scalar(32, 255, 255), cv::Point3d( 1,  1, -1)));
	bottom.push_back(Vertex(cv::Scalar(32, 255, 255), cv::Point3d(-1,  1, -1)));
	bottom.push_back(Vertex(cv::Scalar(32, 255, 255), cv::Point3d(-1,  1,  1)));
	objects.push_back(bottom);


	//The translation vector t of the second camera is known only up to scale.
	//As a consequence, the coordinates of the reconstructed 3D points are only relative.
	//In order to position the cube into the scene, a scale factor has to be found that normalizes the coordinates

	//find max value
	double min_z = X[0].z;
	double max_z = X[0].z;
	for(uint i = 0; i < X.size(); i++) {
		 if(X[i].z < min_z) min_z = X[i].z;
		 if(X[i].z > max_z) max_z = X[i].z;
	}

	if(min_z < 0) { //there are outliers or the camera configuration is wrong
		cout << "Degenerated!!!\n";
		return;
	}

	 //build scale matrix for the cube
	double scaleFactor = 1/max_z;
	double s = (1/scaleFactor) * 0.1;
    cv::Mat scaleMatrix = (cv::Mat_<double>(4,4) << 	 s,  0.0, 0.0, 0.0,
    													0.0,  s,  0.0, 0.0,
    													0.0, 0.0,  s,  0.0,
    													0.0, 0.0, 0.0, 1.0 );

    //build translation matrix for the cube
    double tx = 0;
    double ty = 0;
    double tz = max_z;
    cout << "translating cube to: " << tx << "  "<< ty << "  " << tz <<  endl << endl << endl;
    cv::Mat translationMatrix = (cv::Mat_<double>(4,4) << 	1.0, 0.0, 0.0, tx,
    														0.0, 1.0, 0.0, ty,
    														0.0, 0.0, 1.0, tz,
    														0.0, 0.0, 0.0, 1.0 );

    //build rotation matrix for the cube
    double pi = 3.1415926535897932384626433832795; //where is the built in constant?  ;-)
    double deg2Rad = pi/180.0;
    double angleX = 20;
    double angleY = 30;
    double angleZ = 0;
    double sx = sin(angleX * deg2Rad);
	double cx = cos(angleX * deg2Rad);
	double sy = sin(angleY * deg2Rad);
	double cy = cos(angleY * deg2Rad);
	double sz = sin(angleZ * deg2Rad);
	double cz = cos(angleZ * deg2Rad);

    cv::Mat rotX = (cv::Mat_<double>(4,4) << 	1,  0,   0,  0,
    											0, cx, -sx,  0,
    											0, sx,  cx,  0,
    											0,  0,   0,  1 );

    cv::Mat rotY = (cv::Mat_<double>(4,4) << 	cy,   0,  sy,  0,
    											 0,   1,   0,  0,
    											-sy,  0,  cy,  0,
    											 0,   0,   0,  1 );

    cv::Mat rotZ = (cv::Mat_<double>(4,4) << 	cz, -sz,  0,  0,
    											sz,  cz,  0,  0,
    										 	 0,   0,  1,  0,
    										 	 0,   0,  0,  1 );

    //perform projective transformation
    for(uint i = 0; i < objects.size(); i++) { //for all objects
    	//cout << "Quad: " << i << endl;
    	for(uint j = 0; j < objects[i].size(); j++) { //for all points...
    		//build homogeneous coordinate for projection
    		cv::Mat worldpoint_h = cv::Mat(4, 1, CV_64FC1);
    		worldpoint_h.at<double>(0,0) = objects[i][j].getWorldX();
    		worldpoint_h.at<double>(1,0) = objects[i][j].getWorldY();
    		worldpoint_h.at<double>(2,0) = objects[i][j].getWorldZ();
    		worldpoint_h.at<double>(3,0) = 1.0;

    		//scale
    		worldpoint_h = scaleMatrix * worldpoint_h;

    		//rotate (could be combined in a single matrix)
    		worldpoint_h = rotX * worldpoint_h;
    		worldpoint_h = rotY * worldpoint_h;
    		worldpoint_h = rotZ * worldpoint_h;

    		//translate
    		worldpoint_h = translationMatrix * worldpoint_h;

//    		//convert homogeneous to cartesian coordinates
//    		float x = static_cast<float>(point_h.at<double>(0, 0));
//    		float y = static_cast<float>(point_h.at<double>(0, 1));
//    		float z = static_cast<float>(point_h.at<double>(0, 2));
//    		float w = static_cast<float>(point_h.at<double>(0, 3));
//    		cout << "Point: " << j << "  x: "<< x << "  y: " << y << "  z: " << z << endl;

    		//simple projection by multiplication with projection matrix
    		cv::Mat pProjected_h = P * worldpoint_h; //homogeneous image coordinates 3x1

    		//convert projected image point to carthesian coordinates
    		float w = static_cast<float>(pProjected_h.at<double>(2,0));
    		objects[i][j].setImageX( static_cast<float>(pProjected_h.at<double>(0,0) / w) ); //x = x/w
    		objects[i][j].setImageY( static_cast<float>(pProjected_h.at<double>(1,0) / w) ); //y = y/w

    	}
    }

    //draw vertices and edges of the cube (only green and yellow corners can be see due to the "painter's algorithm")
    for(uint i = 0; i < objects.size(); i++) {//for all objects
    	//cout << "Quad: " << i << endl;
    	for(uint j = 0; j < objects[i].size(); j++) { //for all points...
    		//cout << "x: "<< objects[i][j].getImageCoodinate().x << "   y: " << objects[i][j].getImageCoodinate().y << endl;
    		cv::circle(image, objects[i][j].getImageCoodinate(), 4,  objects[i][j].getColor(), -1);
        		if(j < objects[i].size()-1) { //draw line from first to last vertex
    			cv::line(image, objects[i][j].getImageCoodinate(), objects[i][j+1].getImageCoodinate(), cv::Scalar(255, 255, 255), 1, CV_AA);
    		}
    		if(j == objects[i].size()-1) { //draw line from last to first vertex
    			cv::line(image, objects[i][j].getImageCoodinate(), objects[i][j-3].getImageCoodinate(), cv::Scalar(255, 255, 255), 1, CV_AA);
    		}
    	}
    }
}


void Multiview::drawTriangulatedWorldPoints(cv::Mat& P, Points3D& X, cv::Mat& image, float scale) {
	typedef std::vector<Vertex> Polygon;
	std::vector<Polygon> objects;

	//construct wireframe quad
	Polygon front;
	front.push_back(Vertex(cv::Scalar(32, 32, 255), cv::Point3d( 1, 1, -1))); //red
	front.push_back(Vertex(cv::Scalar(32, 32, 255), cv::Point3d( 1,-1, -1)));
	front.push_back(Vertex(cv::Scalar(32, 32, 255), cv::Point3d(-1,-1, -1)));
	front.push_back(Vertex(cv::Scalar(32, 32, 255), cv::Point3d(-1, 1, -1)));
	objects.push_back(front);


	//build scale matrix for the quad
	double s = scale;
	cv::Mat scaleMatrix = (cv::Mat_<double>(4,4) << 	 s,  0.0, 0.0, 0.0,
			0.0,  s,  0.0, 0.0,
			0.0, 0.0,  s,  0.0,
			0.0, 0.0, 0.0, 1.0 );


	for(uint p = 0; p < X.size(); p++) { //for all world coordinates

		//build translation matrix for the quad
		double tx = X[p].x;
		double ty = X[p].y;
		double tz = X[p].z;

		// cout << "translating cube to: " << tx << "  "<< ty << "  " << tz <<  endl << endl << endl;
		cv::Mat translationMatrix = (cv::Mat_<double>(4,4) << 	1.0, 0.0, 0.0, tx,
				0.0, 1.0, 0.0, ty,
				0.0, 0.0, 1.0, tz,
				0.0, 0.0, 0.0, 1.0 );


		//perform projective transformation
		for(uint i = 0; i < objects.size(); i++) { //for all objects
			//cout << "Quad: " << i << endl;
			for(uint j = 0; j < objects[i].size(); j++) { //for all points...
				//build homogeneous coordinate for projection
				cv::Mat worldpoint_h = cv::Mat(4, 1, CV_64FC1);
				worldpoint_h.at<double>(0,0) = objects[i][j].getWorldX();
				worldpoint_h.at<double>(1,0) = objects[i][j].getWorldY();
				worldpoint_h.at<double>(2,0) = objects[i][j].getWorldZ();
				worldpoint_h.at<double>(3,0) = 1.0;

				//scale
				worldpoint_h = scaleMatrix * worldpoint_h;

				//translate
				worldpoint_h = translationMatrix * worldpoint_h;

				//simple projection by multiplication with projection matrix
				cv::Mat pProjected_h = P * worldpoint_h; //homogeneous image coordinates 3x1

				//convert projected image point to carthesian coordinates
				float w = static_cast<float>(pProjected_h.at<double>(2,0));
				objects[i][j].setImageX( static_cast<float>(pProjected_h.at<double>(0,0) / w) ); //x = x/w
				objects[i][j].setImageY( static_cast<float>(pProjected_h.at<double>(1,0) / w) ); //y = y/w

			}
		}

		//draw simple rectangles where the quad is projected onto the image plane
		cv::rectangle(image, objects[0][0].getImageCoodinate(), objects[0][2].getImageCoodinate(), objects[0][0].getColor(), 1, 8, 0);
	}
}
