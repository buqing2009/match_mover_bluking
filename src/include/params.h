/*
 * params.h
 *
 *  Created on: 14.02.2012
 *      Author: rainer
 */

#ifndef _HEADER_PARAMS_INCLUDED
#define _HEADER_PARAMS_INCLUDED

#include <iostream>
#include <string>
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <sys/types.h>
#include <time.h>

using namespace std;

/**
 * @class Params
 * @brief Handle program settings and parameters via command line
 */
class Params {

public:

private:

	int gaussSize;
	double gaussSigma;
	string videoFile, calibrationFile;
	int m, n; //calibration chess-board size
	double fTol;
	double hTol;
	double T_min;
	double T_max;
	int detectMin;
	int detectMax;
	int detectIterations;
	double loweRatio;

	bool compressOutputVideo;
	bool saveAsImageSequence;
	bool drawTracks;
	bool drawThreeWayTracks;

	int frameLimit;
	int keyframeLimit;
	int maxOpticalFlowLength;


	static void init() { if (Params::instance == NULL) {Params::instance = new Params(); } }

public:
	static Params *instance;

	Params();

	static Params* get() { Params::init(); return Params::instance; }

	void printInfo();
	bool parse(int argc, char *argv[]);

	int getGaussSize() const { return gaussSize; }
	double getGaussSigma() const { return gaussSigma; }

	double getFTol() const { return fTol; }
	double getHTol() const { return hTol; }

	string getVideoFile() const { return videoFile; }
	string getCalibrationFile() const { return calibrationFile; }

	int getM() const { return m; }
	int getN() const { return n; }

	int getDetectMin() const { return detectMin; }
	int getDetectMax() const { return detectMax; }
	int getDetectIterations() const { return detectIterations; }

	double getLoweRatio() const { return loweRatio; }

	int getFrameLimit() const { return frameLimit; }

	bool getSaveAsImageSequence() const { return saveAsImageSequence; }
	bool getCompressOutputVideo() const { return compressOutputVideo; }
	bool getDrawTracks() const { return drawTracks; }
	bool getDrawThreeWayTracks() const { return drawThreeWayTracks; }

	int getKeyframeLimit() {return keyframeLimit;}
	int getFrameLimit() {return frameLimit;}
	int getMaxOpticalFlowLength() {return maxOpticalFlowLength;}

	double getTmin(){ return T_min;}
	double getTmax(){ return T_max;}

};

#endif // _HEADER_PARAMS_INCLUDED
