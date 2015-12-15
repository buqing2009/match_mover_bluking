/*
 * params.cpp
 *
 *  Created on: 14.02.2012
 *      Author: rainer
 */

#include "params.h"
#include <stdlib.h>
#include <string.h>

Params::Params() {
	videoFile = "";
	calibrationFile = "calibration.xml"; //calibration file set to "calibration.xml"
	gaussSize = -1;
	gaussSigma = 2.0;
	m = 0;
	n = 0;
	fTol = 3.0;
	hTol = 2.0;
	detectMin = 100;
	detectMax = 5000;
	detectIterations = 10;
	loweRatio = 0.9;

	T_min = 0.2; //lower limit for correspondence ratio: the lower T_min, the larger the allowed baseline
	T_max = 0.5; //upper limit for correspondence ratio: the higher T_max, the smaller the allowed the baseline

	compressOutputVideo = true;
	saveAsImageSequence = false;
	drawThreeWayTracks = false;
	drawTracks = false;
	maxOpticalFlowLength = 30;
	keyframeLimit = 100;
	frameLimit = 1000;

}

bool Params::parse(int argc, char *argv[]) {
	if (argc < 2) {
		cout << "Missing video file parameter. Example usage: " << argv[0] << " input_video.avi" << endl;
		printInfo();
		return false;
	}
	else {
		for (int i=1; i<argc; i++) {
			const char* arg = argv[i];
			if (strcmp("-video", arg) == 0 && (i+1 < argc)) {
				videoFile = argv[i+1];
			}
			else if (strcmp("-camera", arg) == 0 && (i+1 < argc)) {
				calibrationFile = argv[i+1];
			}
			else if (strcmp("-calibrate", arg) == 0 && (i+2 < argc)) {
				m = atoi(argv[i+1]);
				n = atoi(argv[i+2]);
			}
			else if (strcmp("-gauss", arg) == 0 && (i+1 < argc)) {
				gaussSize = atoi(argv[i+1]);
			}
			else if (strcmp("-gausssigma", arg) == 0 && (i+1 < argc)) {
				gaussSigma = atof(argv[i+1]);
			}
			else if (strcmp("-ftol", arg) == 0 && (i+1 < argc)) {
				fTol = atof(argv[i+1]);
			}
			else if (strcmp("-htol", arg) == 0 && (i+1 < argc)) {
				hTol = atof(argv[i+1]);
			}
			else if (strcmp("-lowe", arg) == 0 && (i+1 < argc)) {
				loweRatio = atof(argv[i+1]);
			}
			else if (strcmp("-tmin", arg) == 0 && (i+1 < argc)) {
				T_min = atof(argv[i+1]);
			}
			else if (strcmp("-tmax", arg) == 0 && (i+1 < argc)) {
				T_max = atof(argv[i+1]);
			}
			else if (strcmp("-tracklength", arg) == 0 && (i+1 < argc)) {
				maxOpticalFlowLength = atoi(argv[i+1]);
			}
			else if (strcmp("-framelimit", arg) == 0 && (i+1 < argc)) {
				frameLimit = atoi(argv[i+1]);
			}
			else if (strcmp("-keyframelimit", arg) == 0 && (i+1 < argc)) {
				keyframeLimit = atoi(argv[i+1]);
			}
			else if (strcmp("-drawtracks", arg) == 0 && (i+1 < argc)) {
				drawTracks = true;
			}
			else if (strcmp("-drawthreeway", arg) == 0 && (i+1 < argc)) {
				drawThreeWayTracks = true;
			}
			else if (strcmp("-help", arg) == 0
					|| strcmp("-h", arg) == 0
					|| strcmp("-?", arg) == 0
					|| strcmp("?", arg) == 0) {
				printInfo();
				exit(EXIT_SUCCESS);
			}
			else if (strcmp("-detect", arg) == 0 && (i+3 < argc)) {
				detectMin = atof(argv[i+1]);
				detectMax = atof(argv[i+2]);
				detectIterations = atof(argv[i+3]);
			}
			else if (argc < 3) { //single parameter treated as video-file, calibration file set to "calibration.xml"
				videoFile = argv[1];
			}
		}
	}
	return true;
}

void Params::printInfo() {
	printf("\n\n Matchmover:");
	printf("\n\t parameters:");
	printf("\n\t [-help] .................... print out this information.");
	printf("\n\t [-video] VIDEOFILE ......... set file name of video");
	printf("\n\t [-camera CAMERAFILE] ....... set file name of intrinsic camera parameters");
	printf("\n\t [-calibrate M N] ........... set size of calibration chessboard");
	printf("\n\t [-verbose VERBOSE] ......... determine console output");
	printf("\n\t [-gauss GAUSS_SIZE] ........ set size of gaussian filter");
	printf("\n\t [-gausssigma GAUSS_SIGMA] .. set sigma of gaussian blur");
	printf("\n\t [-drawtracks] .............. outputs Lukas-Kanade Optical Flow Tracks");
	printf("\n\t [-drawthreeway] ............ outputs Keypoints of Three-Way Point Correspondences");
	printf("\n\t [-ftol FUND_TOLERANCE] ..... set tolerance for fundamental matrix computation.");
	printf("\n\t [-htol HOM_TOLERANCE] ...... set tolerance for homography matrix computation.");
	printf("\n\t [-tmin THRESHOLD] .......... set minimum correspondence ration threshold, currently: %f", T_min);
	printf("\n\t [-tmax THRESHOLD] .......... set minimum correspondence ration threshold, currently: %f", T_max);
	printf("\n\t [-detect MIN MAX ITERATE] .. set minimum, maximum number of key points to detect and number of iterations used (currently: %d %d %d).", detectMin, detectMax, detectIterations);
	printf("\n\t [-lowe LOWE_RATIO] ......... set LOWE ratio., currently: %f", loweRatio);
	printf("\n\t [-framelimit LIMIT] ........ sets a limit for the maximum number of frames, currently: %d", frameLimit);
	printf("\n\t [-keyframelimit LIMIT] ..... sets a limit for the maximum number of keyframes, currently: %d", keyframeLimit);
	printf("\n\t [-tracklength] ............. sets a limit for the Lukas-Kanade Optical Flow Track length, currently: %d", maxOpticalFlowLength);
	printf("\n");
	printf("\n\t\t VIDEOFILE ...... full path of video file");
	printf("\n\t\t CAMERFILE ...... full path to camera calibration file");
	printf("\n\t\t M .............. number of horizontal squares on the calibration chessboard");
	printf("\n\t\t N .............. number of vertical squares on the calibration chessboard");
	printf("\n\t\t GAUSS_SIZE ..... size of gaussian blur (3, 5, 7, 9, 11, ...) -1 to disable gaussian blur, currently: %d", gaussSize);
	printf("\n\t\t GAUSS_SIGMA .... sigma of gaussian blur (floating point), currently: %f", gaussSigma);
	printf("\n\t\t FUND_TOLERANCE . allowed distance of a point from epipolar line in order to be an inlier, currently: %f", fTol);
	printf("\n\t\t HOM_TOLERANCE .. /allowed reprojection error of a point pair in order to be an inlier, currently: %f", hTol);
	printf("\n\t\t LOWE_RATIO ..... set LOWE ratio, a value between 0.1 and 0.9 to filter out bad matches., currently: %f", loweRatio);
	printf("\n\t\t THRESHOLD ...... correspondence ratio, a value between 0.0 and 1.0 related to the baseline., currently: %f", loweRatio);
	printf("\n");
	printf("\n");
}

