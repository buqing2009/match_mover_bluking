#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <execinfo.h>
#include <signal.h>
#include <gtkmm.h> //GTKmm is needed only for calibration

#include "calibration.h"
#include "multiview.h"
#include "utils.h"
#include "params.h"

using namespace std;

Params *Params::instance;

void handler(int sig) {
  void *array[10];
  size_t size;

  //get void*'s for all entries on the stack
  size = backtrace(array, 10);

  //print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, 2);
  exit(1);
}

/**
  @mainpage
  @authors Manuel Lang, Rainer Breuss, Peter Kiechle
  @date February 2012
  @image html matchmover.jpg
  <center>Triangulated world coordinates and reconstructed camera movement</center>
 */
int main(int argc, char *argv[]) {
    	signal(SIGSEGV, handler);  //install our handler

    	if (!Params::get()->parse(argc, argv)) {
    		return EXIT_SUCCESS;
    	}


	const string videoFile = argv[1];

	try {

		if(Params::get()->getM() > 0 && Params::get()->getN() > 0) { //calibration
			Gtk::Main kit(argc, argv);	//GTK initialization
			Calibration *cal = new Calibration();
			cal->runCalibration();

		} else {
			Multiview *multi = new Multiview();
			multi->workflow();
			//multi->workflow_rendertest();
		}
	} catch (std::exception& e) {
		printf("\n\nException caught: %s", e.what());
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS; //won't reach this line anyway

}

