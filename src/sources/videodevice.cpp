#include <iomanip>
#include "videodevice.h"

using namespace std;

VideoIO::VideoIO(const std::string& videoInputfile) {
	if(!setInput(videoInputfile)) {
		exit(0);
	}
}

VideoIO::VideoIO(const std::string& videoInputfile, const std::string& videoOutputfile, bool compress) {

	if(!setInput(videoInputfile)) {
		cerr << "Error, unable to set input device!" << endl;
		exit(0);
	}

	if(!setOutput(videoOutputfile, compress)) {
		cerr << "Error, unable to set output device!" << endl;
		exit(0);
	}
	cout << "VideoIO established" << endl;
}

VideoIO::~VideoIO() {}


bool VideoIO::setInput(const std::string& filename) {
	//capture specified movie file
	capture = cv::VideoCapture(filename);

	if (!capture.isOpened()) {
		cout << "Could not open video file!" << endl;
		return false;
	}

	//determine frame rate
	framerate = static_cast<int>(capture.get(CV_CAP_PROP_FPS));

	//determine the number of frames in the video
	nFrames = static_cast<long int>(capture.get(CV_CAP_PROP_FRAME_COUNT));

	cout << "Video: " << getFrameWidth() << "x" << getFrameHeight()
			<< "  Aspect ratio: " << (float)getFrameWidth() / (float)getFrameHeight() << "  "
			<< framerate << "fps  "
			<< nFrames << " frames" << endl;

	return true;
}

//Note: ffempeg, libjpeg-dev and libpng-dev better be installed
bool VideoIO::setOutput(const std::string &filename, bool compress, bool imageSequence) {

	//split specified filename into basename and extension
	parseFilename(filename, fileBasename, fileExtension);

	//file extension determines video or image file format.
	if(fileExtension.empty()) { //no file extension parsed
		cerr << "Error, no file extension specified, try to add .jpg, .png or .avi" << endl;
		return false;
	}else if(fileExtension == "jpg" || fileExtension == "png") {
		outType = IMAGE_SEQUENCE;
	}else if (fileExtension == "avi") {
		outType = VIDEO_SEQUENCE;
	} else if (fileExtension == "avi" && imageSequence) {
		outType = VIDEO_AND_IMAGE_SEQUENCE;
	} else {
		cerr << "Error, file extension not recognized" << endl;
		return false;
	}

	if(outType == VIDEO_SEQUENCE || outType == VIDEO_AND_IMAGE_SEQUENCE) { //write to video file
		if(compress) {
			fourCC = CV_FOURCC('D','I','V','X'); //DIVX MPEG-4 codec (currently the "best" available in OpenCV)
		} else {
			fourCC = CV_FOURCC('I', 'Y', 'U', 'V'); //uncompressed yuv420p in avi container
		}
		outputVideoFilename = filename;
		//init output video
		return recorder.open(outputVideoFilename, fourCC, framerate, cv::Size(getFrameWidth(), getFrameHeight()), true);
	}

	if(outType == IMAGE_SEQUENCE || outType == VIDEO_AND_IMAGE_SEQUENCE) { //write to image files
		currentOutputIndex = 0;
	}

	return true;
}

//deconstructs filename which decides whether we write to a video or an image sequence
void VideoIO::parseFilename(const std::string &filename, std::string &basename, std::string &extension) {

	string::size_type index = filename.find_last_of('.'); //search for last "." in file name

	if(index == string::npos) { //file name does not contain any period
		basename = filename;
		extension = "";
	} else { //split at period
		basename = filename.substr(0, index);
		extension = filename.substr(index + 1);
	}

}

//store frame to output video/image
void VideoIO::writeNextFrame(cv::Mat& frame) {
	if(outType == IMAGE_SEQUENCE || outType == VIDEO_AND_IMAGE_SEQUENCE) {
			//construct unique image file name with incrementing numbers
			std::stringstream imageFilename;
			imageFilename << fileBasename << "_" << std::setfill('0') << std::setw(5) << currentOutputIndex++ << "." << fileExtension;
			cv::imwrite(imageFilename.str(), frame);
	} else { //write to video file
		if(recorder.isOpened()) {
			recorder << frame;
		}
	}
}

//set video capture device to an absolute frame position between 0 and nFrames
bool VideoIO::goToFrame(long int absolutePosition) {
	if(absolutePosition >= 0 && absolutePosition < nFrames) {
		capture.set(CV_CAP_PROP_POS_FRAMES, absolutePosition);
		return true;
	} else {
		cout << "Error accessing frame position" << endl;
		return false;
	}
}

//get the reference of the next frame
bool VideoIO::getNextFrame(cv::Mat& frame) {
	capture >> frame; //get a new frame from camera or video file
	if(!frame.data) { //no image data available
		return false;
	}
	return true;
}

//get the reference of the specified frame
bool VideoIO::getFrame(long int number, cv::Mat& frame) {
	goToFrame(number);
	capture >> frame; //get a new frame from camera or video file
	if(!frame.data) { //no image data available
		cout << "Error while capturing frame. Frame number: " << number << endl;
		exit(0);
	}
	return true;
}
