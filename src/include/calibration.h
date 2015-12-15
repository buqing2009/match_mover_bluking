#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#include <gtkmm.h>
#include <cv.h>
#include <highgui.h>
#include <string>
#include <iostream>
#include <sys/time.h> //for measuring execution times and limiting fps

#include "utils.h"


using namespace std;
using namespace cv;

class Unknowns; //forward declaration

/**
 * @class Calibration
 * @brief Performs user guided calibration of a video camera
 * @note Click on the video to capture a frame for calibration
 */
class Calibration {

public:
	Calibration();
	virtual ~Calibration();



	int board_w; //number of horizontal corners on the chessboard
	int board_h; //number of vertical corners on the chessboard
	int board_squares; //number of squares on the board
	Size board_sz;
	vector<Point2f> corners; //position of the chessboard corners
	int cornerCount; //number of corners

    vector<vector <Point2f> > imagePoints;
    vector<vector <Point3f> > objectPoints;
	vector<Point3f> helperCoordinates;

	Mat currentFrame; //openCV image object for video frame
	Mat imageGrey; //grey scale image for corner detection
	Mat imageUndistorted; //undistorted video frame

	Mat intrinsicMat; //camera intrinsics matrix
    Mat distCoeffs; //radial and tangential distortion coefficients
    vector<Mat> rvecs; //vector of rotation vectors
    vector<Mat> tvecs; //vector of translation vectors

	bool findCorners; //search for corners on current frame
	bool cornersFound;
	int successes; //number of successful snapshots for calibration
	bool showUndistorted; //show undistorted video
	bool captureFinished; //stop capturing
	string camerafile;
	string videofile;


	Gtk::EventBox *eventBox;
	Unknowns *unknowns;
	bool on_eventbox_button_press(GdkEventButton* event);

	void collectDataCallback(cv::Mat& inputFrame, cv::Mat& resultingFrame);
	static void callbackWrapper(void* pt2Object, cv::Mat& inputFrame, cv::Mat& resultingFrame);

	void calibrateCamera();
	void printMatrix(Mat &mat, string description);
    void runCalibration();
};


/**
 * @class Unknowns
 * @brief GTK Elements
 */
class Unknowns : public Gtk::Window {

public:
	Unknowns(Calibration *cal);
	virtual ~Unknowns();

	Calibration *cal;
	double sum;
	double delta;
	void showResult(Mat& intrinsics, int successes);

protected:
	//Child widgets:
	Gtk::VBox m_VBox; //the rest is inside this
	Gtk::Label m_Label;
	Gtk::Frame m_Frame;
	Gtk::Table m_Table;
	Gtk::Label m_a, m_b, m_c, m_d, m_e, m_f, m_g, m_h, m_i;
	Gtk::HButtonBox m_ButtonBox;
	Gtk::Button m_buttonFinished;
	Gtk::ToggleButton m_buttonFindCorners;
	Gtk::ToggleButton m_buttonShowUndistorted;
	void on_buttonFinished_clicked();
	void on_buttonFindCorners_clicked();
	void on_buttonShowUndistorted_clicked();

};


#endif /* CALIBRATION_H_ */
