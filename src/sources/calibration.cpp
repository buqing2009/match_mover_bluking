#include <iomanip>
#include "calibration.h"
#include "videoplayer.h"
#include "params.h"

Calibration::Calibration() {
	camerafile = Params::get()->getCalibrationFile();
	videofile = Params::get()->getVideoFile();

	//Chessboard size
	board_w = Params::get()->getM() - 1; //number of horizontal corners on the chessboard
	board_h = Params::get()->getN() - 1; //number of vertical corners on the chessboard
	board_squares = board_w * board_h; //number of squares on the board
	board_sz = Size(board_w, board_h);

	printf("\ninit calibration: board_w = %d, board_h = %d.", this->board_w, this->board_h);

	findCorners = false; //search for corners on current frame
	cornersFound = false;
	successes = 0; //number of successful snapshots for calibration
	captureFinished = false; //stop capturing
	showUndistorted = false;

	//The chessboard is considered to be the origin of the world coordinate system.
	//By convention, the board is positioned in the x-z-plane while the camera moves around it.
	//
	// World Coordinate System:
	//  Y ^
	//    |  ^ Z
	//    | /
	//    |/
	//    +-------> X
	//
	//A list of coordinates (0,0,0), (0,1,0), (0,2,0), ..., (1,4,0), ... is created
	//in order to set the mapped corners of the squares on the board in relation to the corresponding 3D coordinates
	//This means, that the distance between two squares in 2D equals one unit in 3D
    for(int i = 0; i < board_squares;i++) { //prepare data structure for cv::calibrateCamera()
        helperCoordinates.push_back(Point3d(i/board_w, i%board_w, 0.0f));
    }
}


Calibration::~Calibration() {
	delete eventBox;
}


void Calibration::runCalibration() {
	//actual video play-back area
	VideoPlayer video(this->videofile, true, true, true, false); //videofile, capture, process, showCaptured, showProcessed
	video.set_title("Calibration");
	video.move(0, 400);
	//engage into video player
	video.setProcessingCallback((void*) this, Calibration::callbackWrapper);
    //connect signal handle of video event boxes
    video.getLeftGTKEventBox().signal_button_press_event().connect(sigc::mem_fun(*this, &Calibration::on_eventbox_button_press));
    video.getRightGTKEventBox().signal_button_press_event().connect(sigc::mem_fun(*this, &Calibration::on_eventbox_button_press));

	unknowns = new Unknowns(this);
	unknowns->move(0,0); //top left

	//enter gtk main loop and return when window is closed.
	Gtk::Main::run(video);
}


bool Calibration::on_eventbox_button_press(GdkEventButton* event) {
	if(cornersFound) {
		//get subpixel accuracy on those corners
		cornerSubPix(imageGrey, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

		//store found corners
		imagePoints.push_back(corners);
		objectPoints.push_back(helperCoordinates);

		successes++;
		cout << successes << " snapshots collected." << endl;

		//calibrateCamera(); //now perform actual calibration

	} else {
		cout << "Chessboard not identified!" << endl;
	}
	return true;
}


//performs actual camera calibration using all snapshots taken so far
void Calibration::calibrateCamera() {
	printf("\nCalibrating the camera...\n");

	//				┌               ┐
	//				│ f_x  0    c_x │
	// intrinsics = │ 0    f_y  c_y │
	//				│ 0    0    1   │
	//				└               ┘
	// f_x,y = focal length
	// c_x,y = principal point (image center)
	intrinsicMat = Mat(3, 3, CV_32FC1); //camera intrinsics matrix (CV_32FC1 = 32 bit floats, single channel)

	//set focal length to 1
	intrinsicMat.ptr<float>(0)[0] = 1.0; //(0,0)
	intrinsicMat.ptr<float>(1)[1] = 1.0; //(1,1)

    //Magic happens here
    //see http://opencv.willowgarage.com/documentation/cpp/camera_calibration_and_3d_reconstruction.html
    cv::calibrateCamera(objectPoints, imagePoints, currentFrame.size(), intrinsicMat, distCoeffs, rvecs, tvecs);

    Utils::printMatrix(intrinsicMat, "Intrinsic Matrix:");
	unknowns->showResult(intrinsicMat, successes);
}

//static wrapper function, that encapsulates the call to a member function
//C like call-back functions in C++ have to be static (i.e. shared by all instances)
//as each instance of a class has its own memory location
void Calibration::callbackWrapper(void* pt2Object, cv::Mat& inputFrame, cv::Mat& resultingFrame) {

	//explicitly cast to a pointer of type Calibration
	Calibration* mySelf = (Calibration*) pt2Object;

	//call member function
	mySelf->collectDataCallback(inputFrame, resultingFrame);

}

//Collects reference points in order to calibrate the camera.
//Is called for each frame of the video.
void Calibration::collectDataCallback(cv::Mat& inputFrame, cv::Mat& resultingFrame) {
	currentFrame = inputFrame;

	if (inputFrame.channels() == 3) {
		cv::cvtColor(inputFrame, imageGrey, CV_BGR2GRAY); //convert to grayscale image
	}

	cornersFound = false;
	if(findCorners) {
		//try to find chessboard corners:
		cornersFound = findChessboardCorners(inputFrame, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		//draw found corners onto frame
		drawChessboardCorners(inputFrame, board_sz, Mat(corners), cornersFound);
	}

	if(showUndistorted) {
		undistort(currentFrame, imageUndistorted, intrinsicMat, distCoeffs);
		cv::imshow("Undistorted", imageUndistorted);
	}
}


Unknowns::Unknowns(Calibration *calibration)
	  : m_Label("Calibration result after 0 snapshots."),
		m_Frame("Intrinsic Matrix"),
		m_Table(3, 3, true),
		m_a("f_x"), m_b("0"), m_c("c_x"), m_d("0"), m_e("f_y"), m_f("c_y"), m_g("0"), m_h("0"), m_i("1"),
		m_buttonFinished("Save"),
		m_buttonFindCorners("Find corners"),
		m_buttonShowUndistorted("Show undistorted")
{
	set_title("Intrinsics and distortion");
	set_default_size(400,300);
	cal = calibration;

	//Gtk::Window -> m_VBox -> m_Label
	//               m_VBox -> m_Frame -> m_Table
	add(m_VBox);
	m_VBox.add(m_Label);
	m_VBox.add(m_Frame);
	m_VBox.set_border_width(10);
	m_Frame.add(m_Table);
	m_Label.set_justify(Gtk::JUSTIFY_LEFT);

	//Add button box:
	m_VBox.pack_start(m_ButtonBox, Gtk::PACK_SHRINK);
	m_ButtonBox.pack_start(m_buttonFindCorners, Gtk::PACK_SHRINK);
	m_ButtonBox.pack_start(m_buttonShowUndistorted, Gtk::PACK_SHRINK);
	m_ButtonBox.pack_start(m_buttonFinished, Gtk::PACK_SHRINK);
	m_ButtonBox.set_layout(Gtk::BUTTONBOX_END);	//force children to the end (= right)
	m_ButtonBox.set_border_width(6);
	m_ButtonBox.set_spacing(6);
	m_buttonFinished.signal_clicked().connect( sigc::mem_fun(*this, &Unknowns::on_buttonFinished_clicked) );
	m_buttonFindCorners.signal_clicked().connect( sigc::mem_fun(*this, &Unknowns::on_buttonFindCorners_clicked) );
	m_buttonFindCorners.set_active();
	m_buttonShowUndistorted.signal_clicked().connect( sigc::mem_fun(*this, &Unknowns::on_buttonShowUndistorted_clicked) );

	//Fill Table:
	//	    0   1   2
	//	  +---+---+---+
	//	0 | a | b | c |
	//	  +---+---+---+
	//	1 | d | e | f |
	//	  +---+---+---+
	//	2 | g | h | i |
	//	  +---+---+---+
	m_Table.attach(m_a, 0, 1, 0, 1);
	m_Table.attach(m_b, 1, 2, 0, 1);
	m_Table.attach(m_c, 2, 3, 0, 1);
	m_Table.attach(m_d, 0, 1, 1, 2);
	m_Table.attach(m_e, 1, 2, 1, 2);
	m_Table.attach(m_f, 2, 3, 1, 2);
	m_Table.attach(m_g, 0, 1, 2, 3);
	m_Table.attach(m_h, 1, 2, 2, 3);
	m_Table.attach(m_i, 2, 3, 2, 3);

	sum = 0;
	show_all();
}

Unknowns::~Unknowns() {
}

void Unknowns::showResult(Mat& intrinsics, int successes) {
	std::ostringstream buffer;
	buffer.precision(5);

	//simple error control
	double sum_new = intrinsics.at<double>(0, 0)
			+ intrinsics.at<double>(0, 3)
			+ intrinsics.at<double>(1, 2)
			+ intrinsics.at<double>(1, 3);

	delta = abs(sum - sum_new);
	sum = sum_new;

	buffer << "Calibration result after " << successes << " snapshots. \nAverage Difference to last run: " << delta;
	m_Label.set_label(buffer.str());
	buffer.str(""); //clear buffer


	//print matrix elements to gtk widget
	buffer << intrinsics.at<double>(0, 0); m_a.set_label(buffer.str()); buffer.str("");
	buffer << intrinsics.at<double>(0, 1); m_b.set_label(buffer.str()); buffer.str("");
	buffer << intrinsics.at<double>(0, 2); m_c.set_label(buffer.str()); buffer.str("");

	buffer << intrinsics.at<double>(1, 0); m_d.set_label(buffer.str()); buffer.str("");
	buffer << intrinsics.at<double>(1, 1); m_e.set_label(buffer.str()); buffer.str("");
	buffer << intrinsics.at<double>(1, 2); m_f.set_label(buffer.str()); buffer.str("");

	buffer << intrinsics.at<double>(2, 0); m_g.set_label(buffer.str()); buffer.str("");
	buffer << intrinsics.at<double>(2, 1); m_h.set_label(buffer.str()); buffer.str("");
	buffer << intrinsics.at<double>(2, 2); m_i.set_label(buffer.str()); buffer.str("");

}

void Unknowns::on_buttonFinished_clicked() {
	if(cal->successes > 0) { //finish capturing
		cal->captureFinished = true;

		cal->calibrateCamera(); //now perform actual calibration

		//Save values to file
	    FileStorage fs("calibration.xml", FileStorage::WRITE);
	    fs << "IntrinsicCameraMatrix" << cal->intrinsicMat << "DistortionCoefficients" << cal->distCoeffs;
		fs.release();
		cout << "Files saved!" << endl;

	} else { //continue
		cout << "Not enough data to compute intrinsic matrix and distortion coefficients!" << endl;
	}
}

void Unknowns::on_buttonFindCorners_clicked() {
	cal->findCorners = !cal->findCorners;
}
void Unknowns::on_buttonShowUndistorted_clicked() {
	if(cal->successes > 0) { //only makes sense after successful snapshots
		if(cal->showUndistorted) { //destroy window
			cvDestroyWindow("Undistorted");
		} else { //create window
			namedWindow("Undistorted", CV_WINDOW_AUTOSIZE);
			cvMoveWindow("Undistorted", 600, 400); //no c++ version of this function available
		}

		cal->showUndistorted = !cal->showUndistorted;
	}
}

