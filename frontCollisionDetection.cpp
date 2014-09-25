//============================================================================
// Name        : frontCollisionDetection.cpp
// Author      : Murat AMBARKUTUK
// Version     : Beta
// Copyright   : CC
// Description : Front Collision Detection and Warning System
//============================================================================


#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <signal.h>
#include <string>

using namespace std;
using namespace cv;

unsigned int motionDetection(Mat frameOne, Mat frameTwo,int threshold=20);
void serial_port_close();
int serial_port_open();
int serial_port_read(char *read_buffer, size_t max_chars_to_read);
void serial_port_write(char *write_buffer);
void sigint_handler(int sig);
Mat parseData(string data);

#define MAX_COMMAND_LENGTH 44

Mat frameDiff;
struct termios options_original;
static const char *PORT_NAME = "/dev/ttyACM0";
int serial_port;

int main(){
	int chars_read;
	Mat origin(250,500, CV_8UC3, Scalar(255,255,255));
	Mat lastDistances = Mat::zeros(1,9,CV_8UC1);
	Mat distances = Mat::zeros(1,9,CV_8UC1);
	Mat diffDistances = Mat::zeros(1,9,CV_8UC1);
	int threshold = 20;
	Scalar color;
	int cap;
	namedWindow("VIDAR System",1);
	char read_buffer[MAX_COMMAND_LENGTH + 1] = {0};
	serial_port_open();
	for(int i=0;i<distances.cols;i++){
		double theta = CV_PI*(21-2*i)/24;
		Point xA = Point(200*cos(theta),200*sin(theta));
		Point xC = Point(250,250);
		line(origin,xC-xA,xC,Scalar(127,127,127),1,CV_AA);
	}

	while(true){
		Mat frame = origin.clone();
		if (serial_port != -1){
			chars_read = serial_port_read(read_buffer, MAX_COMMAND_LENGTH);
			string data = string(read_buffer);
			if (chars_read > 0){
	 			//cout<<"Data:"<<data;
				Mat temp = parseData(data);
				distances = temp.clone();
				cout<<"Distances    : "<<distances<<endl;
				cout<<"LastDistances: "<<lastDistances<<endl;
				if(distances.cols==9){
					absdiff(distances,lastDistances,diffDistances);
					cout<<"Diff: "<<diffDistances<<endl;
					Mat mask = diffDistances>threshold;

					for(int i=0;i<distances.cols;i++){
						int dist = (int)distances.at<unsigned char>(0,i);
						if(mask.at<unsigned char>(0,i)==255 || dist < 20){
							color = Scalar(0,0,255);
							cap = 10;
						}
						else{
							color = Scalar(0,255,0);
							cap = 5;
						}
						if(dist>=10){
							double theta = CV_PI*(21-2*i)/24;
							Point xP = Point(distances.at<unsigned char>(0,i)*cos(theta),distances.at<unsigned char>(0,i)*sin(theta));
							Point xC = Point(250,250);
							//cout<<"Sensor: "<<i<<" theta: "<<theta<<" xA: "<<xA<<" xC-xA: "<<xC-xA<<" xP: "<<xP<<endl;
							circle(frame, xC-xP, cap, color, -1);
						}
					}
					//cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
					lastDistances = distances.clone();
					//imwrite("/home/ezwei/Desktop/vidar.jpg", frame);
				}
			}
		}
		imshow("VIDAR System",frame);
		if(waitKey(30) >= 0) break;
	}
	return 0;
}


int main2() {
	int chars_read;
	char read_buffer[MAX_COMMAND_LENGTH + 1] = {0};
	serial_port_open();
	VideoCapture capture(1);

	Mat frameTwo = Mat::zeros(480,640,CV_8UC1);
	Mat frameOne,frameGray,frameCont;

	//time_t start,end;

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	namedWindow("frame",1);
	namedWindow("frameDiff",1);
	for(;;){
		if (serial_port != -1){
			chars_read = serial_port_read(read_buffer, MAX_COMMAND_LENGTH);
			string data = string(read_buffer);
			if (chars_read > 0){
				cout<<"Data:"<<data;
				Mat distances = parseData(data);
				cout<<"Mat: "<<distances<<endl;
			}

		}
		//start = getTickCount();
		capture >> frameGray;
		cvtColor(frameGray,frameGray,CV_BGR2GRAY);
		frameOne = frameGray.clone();
		cout<<"mP: "<<motionDetection(frameOne,frameTwo)<<endl;
		frameCont = frameDiff.clone();
		findContours(frameCont, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		vector<Rect> boundRect(contours.size());
		for(size_t i = 0; i< contours.size(); i++){
			boundRect[i] = boundingRect(Mat(contours[i]));
			rectangle(frameGray, boundRect[i].tl(), boundRect[i].br(), Scalar(255), 2, 8, 0);
		}
		frameTwo = frameOne.clone();
		line(frameGray, Point(frameGray.cols/2,0), Point(frameGray.cols/2,frameGray.rows), Scalar(255), 1, 4);
		imshow("frame", frameGray);
		imshow("frameDiff", frameDiff);
		if(waitKey(30) >= 0) break;
		//end = getTickCount();
		//cout<<"FPS: "<<getTickFrequency()/(end-start)<<endl;
	}
	return 0;
}

unsigned int motionDetection(Mat frameOne, Mat frameTwo,int threshold){
	unsigned int movingPixels = (unsigned int)frameOne.rows*frameOne.cols;
	absdiff(frameOne, frameTwo,frameDiff);
	frameDiff = frameDiff > threshold;
	Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
	morphologyEx(frameDiff,frameDiff, MORPH_OPEN,element);
	movingPixels = countNonZero(frameDiff);
	return movingPixels;
}

void serial_port_close(){
	tcsetattr(serial_port,TCSANOW,&options_original);
	close(serial_port);
}

int serial_port_open(void){
	struct termios options;

	serial_port = open(PORT_NAME, O_RDWR | O_NONBLOCK);

	if (serial_port != -1){
		printf("Serial Port open\n");
		tcgetattr(serial_port,&options_original);
		tcgetattr(serial_port, &options);
		cfsetispeed(&options, B115200);
		cfsetospeed(&options, B115200);
		options.c_cflag |= (CLOCAL | CREAD);
		options.c_lflag |= ICANON;
		tcsetattr(serial_port, TCSANOW, &options);
  }
  else
	  cout<<"Unable to open"<<PORT_NAME<<endl;
  return (serial_port);
}

void serial_port_write(char *write_buffer){
	int bytes_written;
	size_t len = 0;
	len = strlen(write_buffer);
	bytes_written = write(serial_port, write_buffer, len);
	if ((size_t)bytes_written < len){
		printf("Write failed \n");
	}
}

int serial_port_read(char *read_buffer, size_t max_chars_to_read){
	int chars_read = read(serial_port, read_buffer, max_chars_to_read);
	return chars_read;
}

void  sigint_handler(int sig){
	serial_port_close();
	exit (sig);
}

Mat parseData(string data){
	std::stringstream test(data);
	std::string segment;
	std::vector<std::string> seglist;
	vector<int> values;
	while(std::getline(test, segment, '|')){
		seglist.push_back(segment);
		values.push_back(atoi(segment.c_str()));
	}
	values.erase(values.begin());
	values.pop_back();

	Mat val = Mat::zeros(1,values.size(),CV_8UC1);
	for(unsigned int i=0;i<values.size();i++){
		val.at<unsigned char>(0,i) = values.at(i);
	}
	return val;
}
