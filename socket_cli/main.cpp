#include "opencv2/opencv.hpp"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>

#define WIDTH 320
#define HEIGHT 320

using namespace cv;
using namespace std;

int main(int argc, char *argv[]) {
	int sokt;
	char serverIP[] = "192.168.0.7";
	int serverPort = 4097;

	struct sockaddr_in serverAddr;
	socklen_t addrLen = sizeof(struct sockaddr_in);

	if ((sokt = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
		cerr << "socket() failed" << endl;
	}

	serverAddr.sin_family = PF_INET;
	serverAddr.sin_addr.s_addr = inet_addr(serverIP);
	serverAddr.sin_port = htons(serverPort);

	if (connect(sokt, (sockaddr*)&serverAddr, addrLen) < 0) 
		cerr << "connect() failed!" << endl;

	Mat img_gray, img_bgrcolor;
	img_gray = Mat::zeros(WIDTH, HEIGHT, CV_8UC1);
	int imgSize = img_gray.total() * img_gray.elemSize();
	uchar *iptr = img_gray.data;
	int bytes = 0;
	int key;

	if (!img_gray.isContinuous() ) {
		img_gray = img_gray.clone();
	}

	cout << "Image Size: " << imgSize << endl;

	namedWindow("CV Video Client" , 1);
	
	while(key != 'q') {
		if ((bytes = recv(sokt, iptr, imgSize, MSG_WAITALL)) == -1) {
			cerr << "recv failed. received bytes = " << bytes << endl;
		}

		imshow("CV Video Client", img_gray);
		if( (key = waitKey(10)) >= 0) break;
	}
	close(sokt);
	return 0;
}
