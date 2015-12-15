#ifndef MATCHMOVER_UTILS_H_
#define MATCHMOVER_UTILS_H_

#include <iostream>
#include <iomanip>
#include <string>
#include <stdio.h>
#include <time.h>
#include <cv.h>
#include <cvaux.h>

#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>

using namespace std;


/**
 * @class Utils
 * @brief Collection of useful static functions
 */
class Utils {
	public:

	static double getCurrentTimeMilliseconds() {
		timeval tv;
		gettimeofday(&tv, NULL);
		return tv.tv_sec * 1000. + tv.tv_usec / 1000.;
	}

	//sleep without busy waiting
	static int msleep(unsigned long milisec) {
	    struct timespec req={0};
	    time_t sec=(int)(milisec / 1000);
	    milisec = milisec-(sec * 1000);
	    req.tv_sec = sec;
	    req.tv_nsec = milisec * 1000000L;
	    while(nanosleep(&req, &req) == -1)
	         continue;
	    return 1;
	}

	//Pretty printing of openCV matrices
	static void printMatrix(cv::Mat &mat, string description) {
		string datatyp = "dont' know";
		if(mat.depth() == 5) {
			datatyp = "float";
		} else if(mat.depth() == 6) {
			datatyp = "double";
		}
		cout << endl << description << " (" << mat.rows <<"x" << mat.cols << " of type " << datatyp << ")" << endl;
		cout << setprecision(3) << right << fixed;

		if(mat.depth() == 5) { //data type = CV_32FC1
			for(int row = 0; row < mat.rows; row++) {
				for(int col = 0; col < mat.cols; col++) {
					cout << setw(12) << mat.at<float>(row, col);
				}
				cout << endl;
			}
		} else if (mat.depth() == 6) { //data type = CV_64FC1
			for(int row = 0; row < mat.rows; row++) {
				for(int col = 0; col < mat.cols; col++) {
					cout << setw(12) << mat.at<double>(row, col);
				}
				cout << endl;
			}
		} else {
			cout << "Matrix type not supported yet. Feel free to expand printMatrix()" << endl;
		}
		cout << endl;
	}

};


#endif /* MATCHMOVER_UTILS_H_ */
