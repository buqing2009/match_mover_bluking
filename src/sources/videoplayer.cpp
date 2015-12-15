#include "videoplayer.h"
#include "utils.h"

using namespace std;


VideoPlayer::VideoPlayer(const std::string& videofile, bool play, bool process, bool showCaptured, bool showProcessed)
	:	captureFrame(play),
		processFrame(process),
		showCapturedFrame(showCaptured),
		showProcessedFrame(showProcessed),
		m_VBox(false, 10),
		m_HBoxNavigation(false, 0),
		m_adjustment(0.0, 0.0, 1.0, 0.1, 0.0, 0.0), //initial value, lower, upper, step_increment, page_increment(pointless), page_size(pointless), max value = upper - page_size
		m_slider(m_adjustment),
		m_label("0 / 0")
{
	video = new VideoIO(videofile); //input only

	//Play-Pause button
	m_buttonPlay.set_use_stock(true); //use label as stock id
	captureFrame ? m_buttonPlay.set_label(Gtk::Stock::MEDIA_PAUSE.id) : m_buttonPlay.set_label(Gtk::Stock::MEDIA_PLAY.id);
	m_buttonPlay.set_size_request(120, 30);
    m_buttonPlay.signal_clicked().connect(sigc::mem_fun(*this, &VideoPlayer::on_buttonPlay_clicked));

	//video position slider:
	m_slider.set_digits(2); //number of digits in slider position
	m_slider.set_draw_value(false); //don't show position label
	m_slider.set_value_pos(Gtk::POS_RIGHT); //where to draw the position label (if drawn at all)
	m_slider.set_size_request(200, 30);
	m_slider.signal_button_press_event().connect(sigc::mem_fun(*this, &VideoPlayer::on_slider_clicked), false);
	m_slider.signal_button_release_event().connect(sigc::mem_fun(*this, &VideoPlayer::on_slider_released), false);
	m_slider.signal_change_value().connect(sigc::mem_fun(*this, &VideoPlayer::on_slider_value_changed));

	m_label.set_size_request(100, 30);

	//video navigation bar
	m_HBoxNavigation.pack_start(m_buttonPlay, Gtk::PACK_SHRINK);
	m_HBoxNavigation.pack_start(m_slider);
	m_HBoxNavigation.pack_start(m_label, Gtk::PACK_SHRINK);

	//combine GUI elements
	set_border_width(10);
	eventBoxLeft.add(imageLeft); //gtk-event box to receive mouse clicks on gtk-imageLeft
	eventBoxRight.add(imageRight); //gtk-event box to receive mouse clicks on gtk-imageRight

	if(showCaptured)
		m_HBoxVideo.pack_start(eventBoxLeft, Gtk::PACK_SHRINK);
	if(showCaptured && showProcessed)
		m_HBoxVideo.pack_start(m_seperator, Gtk::PACK_SHRINK, 5);
	if(showProcessed)
		m_HBoxVideo.pack_start(eventBoxRight, Gtk::PACK_SHRINK);


	m_VBox.pack_start(m_HBoxVideo, Gtk::PACK_EXPAND_WIDGET);
	m_VBox.pack_start(m_HBoxNavigation, Gtk::PACK_EXPAND_WIDGET);
	add(m_VBox);
	show_all();

	//adjust the slider properties (one tick per frame)
	m_slider.get_adjustment()->set_value(0.0); //initial value
	m_slider.get_adjustment()->set_lower(0.0);
	m_slider.get_adjustment()->set_upper(static_cast<double>(video->getNumberOfFrames()-1)); //max value = upper - page_size
	m_slider.get_adjustment()->set_step_increment(1.0);
	m_slider.get_adjustment()->set_page_increment(0.0);
	m_slider.get_adjustment()->set_page_size(0.0);

	fps = video->getFrameRate();
	updateInterval = static_cast<int>(1.0 / fps * 1000.0);
	cout << "frame rate: " << fps << ", update interval: " << updateInterval << "ms" << endl;

	//let GTK refresh video frame as often as possible
	connection = Glib::signal_idle().connect(sigc::mem_fun(*this, &VideoPlayer::on_idle));
}

VideoPlayer::~VideoPlayer() {
	delete video;
}

void VideoPlayer::showCapturedVideo(bool val) {
	showCapturedFrame = val ? true : false;
}
void VideoPlayer::showProcessedVideo(bool val) {
	showProcessedFrame = val ? true : false;
}

//pause capturing video
void VideoPlayer::stopCapturing() {
	captureFrame = false;
}

//stop capturing video
void VideoPlayer::resumeCapturing() {
	captureFrame = true;
}

//stop processing of the video frame (subset of stopCapturing())
void VideoPlayer::stopProcessing() {
	processFrame = false;
}

//registers a function of a foreign class instance to the processing chain
void VideoPlayer::setProcessingCallback(void* pt2Object, void (*pt2Function)(void* pt2Object, cv::Mat&, cv::Mat&)) {
	classInstance = pt2Object; //pointer to instance of foreign class (is also wrapped into processingCallback)
	processingCallback =  pt2Function;  //pointer to foreign function
}

//gets next frame from video device, performs video processing and displays the result if desired
bool VideoPlayer::updateVideo() {

	if(!video->getNextFrame(currentFrame)) { //get next frame
		if(video->getCurrentFrameNumber() == video->getNumberOfFrames()-1) { //end of video reached
			cout << "Playback stopped. (End of file)" << endl;
			captureFrame = false;
			return true;
		} else {
			cout << "Error while capturing frame. Frame number: " << video->getCurrentFrameNumber() << endl;
			return false; //destroys signal handle when capturing fails (implicit disconnect)
		}
	}

	//process the frame in call-back function of classInstance
	if (processFrame) {
		processingCallback(classInstance, currentFrame, processedFrame);
	} else {
		processedFrame = currentFrame;
	}

	//display currently captured video frame
	if(showCapturedFrame) {
		cv::cvtColor(currentFrame, currentFrame, CV_BGR2RGB); //color swap from openCV (BGR) to Gdk::Pixbuf (RGB)
		//create gtk pixel buffer from opencv image
		pixbufLeft = Gdk::Pixbuf::create_from_data((guint8*) currentFrame.data, Gdk::COLORSPACE_RGB, false, 8, currentFrame.cols, currentFrame.rows, currentFrame.step);

		//resize if screen resolution is too low to display both images next to each other
		float screenWidth = get_screen()->get_width();
		if(screenWidth < 2*currentFrame.cols) {
			float aspectRatio = (float)currentFrame.cols / (float)currentFrame.rows;
			pixbufLeft = pixbufLeft->scale_simple(static_cast<int>(screenWidth/2 - 20), static_cast<int>((screenWidth/2 - 20)/aspectRatio), Gdk::INTERP_BILINEAR);
		}

		imageLeft.set(pixbufLeft);
	}

	//display processed video frame
	if(showProcessedFrame && processedFrame.data) {
		if (processedFrame.type() == CV_8UC1) { //greyscale
			rgb = cv::Mat(processedFrame.size(), CV_8UC3); //3 channels
			cv::cvtColor(processedFrame, rgb, CV_GRAY2RGB); //"pump" the greyscale channels up 3 color channels
			pixbufRight = Gdk::Pixbuf::create_from_data((guint8*) rgb.data, Gdk::COLORSPACE_RGB, false, 8, rgb.cols, rgb.rows, rgb.step);
		} else if (processedFrame.type() == CV_8UC3){ //color
			cv::cvtColor(processedFrame, processedFrame, CV_BGR2RGB); //color swap from openCV (BGR) to Gdk::Pixbuf (RGB)
			pixbufRight = Gdk::Pixbuf::create_from_data((guint8*) processedFrame.data, Gdk::COLORSPACE_RGB, false, 8, processedFrame.cols, processedFrame.rows, processedFrame.step);
		}

		//resize if screen resolution is too low to display both images next to each other
		float screenWidth = get_screen()->get_width();
		if(screenWidth < 2*processedFrame.cols) {
			float aspectRatio = (float)processedFrame.cols / (float)processedFrame.rows;
			pixbufRight = pixbufRight->scale_simple(static_cast<int>(screenWidth/2 - 20), static_cast<int>((screenWidth/2 - 20)/aspectRatio), Gdk::INTERP_BILINEAR);
		}

		imageRight.set(pixbufRight);
	}

	//update video navigation bar
	m_slider.set_value(video->getCurrentFrameNumber());
	std::ostringstream buffer;
	buffer << video->getCurrentFrameNumber()+1 << " / " << video->getNumberOfFrames();
	m_label.set_label(buffer.str());
	return true;
}

//this timer callback function is called every 1/fps seconds
//and updates the the current video frame (Gtk::Image) periodically
bool VideoPlayer::on_idle() {
	Glib::RefPtr<Gdk::Window> window = get_window();
	if(window) {

		t1 = Utils::getCurrentTimeMilliseconds(); // get current system time in milliseconds

		if(captureFrame) {
			updateVideo();  //grab next frame, process and display
		}

		//frame rate limiter
		t2 = Utils::getCurrentTimeMilliseconds(); //get current system time in milliseconds
		duration = t2 - t1;	//elapsed time between t1 and t2
		delay = static_cast<unsigned long>(updateInterval - duration);	//difference between wanted refresh rate and actual delay
		if(duration > 0 && duration < updateInterval) { //we are too fast -> slow down playback
			Utils::msleep(delay); //add delay (non-blocking)
		}

	}
	return true;
}


void VideoPlayer::on_buttonPlay_clicked() {
	if(captureFrame) {
		stopCapturing();
		m_buttonPlay.set_label(Gtk::Stock::MEDIA_PLAY.id);
	} else {
		resumeCapturing();
		m_buttonPlay.set_label(Gtk::Stock::MEDIA_PAUSE.id);
	}
}

bool VideoPlayer::on_slider_value_changed(Gtk::ScrollType type, double value) {
	if(video->goToFrame(m_adjustment.get_value())) {
		m_slider.set_value(m_adjustment.get_value());
	}
	updateVideo();  //grab next frame, process and display
	return true;
}

//Gtk's Hscale widgets normally "jump" to a specific position with a middle-click.
//To achieve this with the left mouse button, the event is manipulated before the widgets reacts to it
bool VideoPlayer::on_slider_clicked(GdkEventButton* event) {
	if (event->button == 1) { //left click
        event->button = 2; //middle click
	}
    return false;
}

bool VideoPlayer::on_slider_released(GdkEventButton* event) {
	if (event->button == 1) { //left click
        event->button = 2; //middle click
	}
    return false;
}




