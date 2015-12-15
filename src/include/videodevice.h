#ifndef VIDEODEVICE_H_
#define VIDEODEVICE_H_

#include <cv.h>
#include <highgui.h>
#include <string>
#include <iostream>

#include "videodevice.h"


/**
 * @class VideoIO
 * @brief Encapsulates OpenCV's video processing capabilities
 */
class VideoIO {

public:
    VideoIO(const std::string& inputVideoFileName); //just capturing, without recording
	VideoIO(const std::string& videoInputfile, const std::string& videoOutputfile, bool compress = false); //capture and record

	virtual ~VideoIO();

	void parseFilename(const std::string &filename, std::string &basename, std::string &extension);


	//input video
    bool setInput(const std::string &filename);
	int getFrameRate() {return framerate;}
	long int getNumberOfFrames() {return nFrames;}
	long int getCurrentFrameNumber() {return capture.get(CV_CAP_PROP_POS_FRAMES); }//zero indexed...;
	int getFrameWidth() { return static_cast<int>(capture.get(CV_CAP_PROP_FRAME_WIDTH)); }
	int getFrameHeight() { return static_cast<int>(capture.get(CV_CAP_PROP_FRAME_HEIGHT)); }
    bool goToFrame(long int position);
    bool getNextFrame(cv::Mat& frame);
    bool getFrame(long int position, cv::Mat& frame);


    //output video
    bool setOutput(const std::string &filename, bool compress = false, bool imageSequence = false);
    void writeNextFrame(cv::Mat& frame);


private:


    //input video
	cv::VideoCapture capture;
	long int nFrames; //number of frames in the video
	int framerate;


	//output video
    enum OutputType {VIDEO_SEQUENCE, IMAGE_SEQUENCE, VIDEO_AND_IMAGE_SEQUENCE};
	OutputType outType;
	cv::VideoWriter recorder;
	std::string outputVideoFilename; //filename of output video/image sequence
	int fourCC;
	long int currentOutputIndex; //current index for output frames
	std::string fileBasename;  //file prefix of output images
	std::string fileExtension;  //file extension of output images


};



#endif /* VIDEODEVICE_H_ */
