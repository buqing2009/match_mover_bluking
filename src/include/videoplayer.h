#ifndef VIDEOPLAYER_H_
#define VIDEOPLAYER_H_

#include <gtkmm.h>
#include <cv.h>
#include <highgui.h>
#include <string>
#include <iostream>
#include "videodevice.h"


/**
 * @class VideoPlayer
 * @brief GTK based video player used for calibration
 */
class VideoPlayer: public Gtk::Window {

public:
	VideoPlayer(const std::string& videofile, bool capture, bool process, bool showCaptured, bool showProcessed);
	virtual ~VideoPlayer();

    //registers call-back function and calling
	void setProcessingCallback(void* pt2Object, void (*pt2Function)(void* pt2Object, cv::Mat&, cv::Mat&));

    void stopCapturing();
    void resumeCapturing();

    //stop processing of the video frame
    void stopProcessing();

    void showCapturedVideo(bool);
    void showProcessedVideo(bool);

//	void goToFrame(long int position);

    Gtk::Image& getLeftGTKImage() { return imageLeft; }
    Gtk::EventBox& getLeftGTKEventBox() { return eventBoxLeft; }
    Gtk::Image& getRightGTKImage() { return imageLeft; }
    Gtk::EventBox& getRightGTKEventBox() { return eventBoxRight; }

private:

    VideoIO *video;

	int fps, updateInterval; //frame rate and its inverse in milliseconds
	double t1, t2;	//start and stop times for calculating frame rate
	double duration; //elapsed time in milliseconds
	unsigned long delay; //delay video play-back (in milliseconds)

    bool captureFrame;	//stop capturing?
    bool processFrame;  //execute process call-back?

    bool showCapturedFrame;	//normal video
    bool showProcessedFrame; //processed video

	cv::Mat currentFrame; //current frame
	cv::Mat processedFrame; //resulting frame
	cv::Mat rgb; //dummy to convert grayscale<->rgb

	void* classInstance; //pointer to the calling class instance
    void (*processingCallback)(void* pt2Object, cv::Mat&, cv::Mat&); //pointer to the call-back function

protected:

	//Override default signal handlers:
	bool on_idle();
	bool updateVideo();

	//Child widgets:
	sigc::connection connection; //gtk signal
	Gtk::Image imageLeft; //widget where actual video frame is drawn
	Gtk::Image imageRight; //widget where actual video frame is drawn
	Gtk::EventBox eventBoxLeft; //gtk-event box to receive mouse clicks on left gtk-image
	Gtk::EventBox eventBoxRight; //gtk-event box to receive mouse clicks on gtk-image
	Glib::RefPtr<Gdk::Pixbuf> pixbufLeft; //gtk pixel buffer object to be drawn inside the image widget
	Glib::RefPtr<Gdk::Pixbuf> pixbufRight; //gtk pixel buffer object to be drawn inside the image widget

	Gtk::VBox m_VBox;
	Gtk::HBox m_HBoxVideo;
	Gtk::HBox m_HBoxNavigation;
	Gtk::VSeparator m_seperator;

	Gtk::Button m_buttonPlay;
	Gtk::Adjustment m_adjustment;
	Gtk::HScale m_slider;
	Gtk::Label m_label;

	bool on_slider_clicked(GdkEventButton* event);
	bool on_slider_released(GdkEventButton* event);
	bool on_slider_value_changed(Gtk::ScrollType type, double value);
	void on_buttonPlay_clicked();
};







#endif /* VIDEOPLAYER_H_ */
