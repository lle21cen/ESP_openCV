#include <iostream>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>

using namespace std;
using namespace cv;

void trackingCamShift(Mat& img, Rect search_window) {
	TermCriteria criteria(TermCriteria::COUNT | TermCriteria::EPS,10,1);

	RotatedRect found_object = CamShift(img, search_window, criteria);

	Rect found_rect = found_object.boundingRect();
	rectangle(img, found_rect, Scalar(0,255,0),3);
}

int main ( int argc,char **argv ) {

	raspicam::RaspiCam_Cv Camera;
	Mat frame;

	Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3);
	Camera.set( CV_CAP_PROP_FRAME_WIDTH, 640 );
	Camera.set( CV_CAP_PROP_FRAME_HEIGHT, 480 );

	if (!Camera.open()) {
		cerr<<"Error opening the camera"<<endl;
		return -1;
	}

	namedWindow("Video Camera");

	while(1){

		Camera.grab();
		Camera.retrieve (frame);

		cvtColor(frame, frame, COLOR_BGR2GRAY);

		Rect search_window(200,150,100,100);
		trackingCamShift(frame, search_window);

		imshow( "Video Camera", frame);
		if ( waitKey(20) == 27 ) break;

	}

	Camera.release();
	return 0;
}
