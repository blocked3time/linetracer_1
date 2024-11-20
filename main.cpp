#include <iostream>
#include <unistd.h>
#include <signal.h>
#include "dxl.hpp"
#include "opencv2/opencv.hpp"
#include<math.h>
using namespace cv;
using namespace std;
bool ctrl_c_pressed;
void ctrlc(int)
{
	ctrl_c_pressed = true;
}
int main(void)
{	/*
    string src = "nvarguscamerasrc sensor-id=0 ! \
video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
format=(string)NV12, framerate=(fraction)30/1 ! \
nvvidconv flip-method=0 ! video/x-raw, \
width=(int)640, height=(int)360, format=(string)BGRx ! \
videoconvert ! video/x-raw, format=(string)BGR ! appsink";
	VideoCapture source(src, CAP_GSTREAMER);
*/
    VideoCapture source("5_lt_cw_100rpm_out.mp4");
	//VideoCapture source("7_lt_ccw_100rpm_in.mp4");
	if (!source.isOpened()) { cout << "Camera error" << endl; return -1; }
		VideoWriter vw("output(out).mp4", VideoWriter::fourcc('X', '2', '6', '4'), 30, Size(640, 360/4), true);
	if (!vw.isOpened())
	{
		std::cout << "Can't write video !!! check setting" << std::endl;
		return -1;
	}
	string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
h264parse ! rtph264pay pt=96 ! \
udpsink host=203.234.58.157 port=8001 sync=false";
	VideoWriter writer1(dst1, 0, (double)30, Size(640, 360/4), true);
	if (!writer1.isOpened()) {
		cerr << "Writer open failed!" << endl; return -1;
	}
	signal(SIGINT, ctrlc); //시그널 핸들러 지정
	Mat stats, centroids, labels,frame, gray;
	double err = 0;
	TickMeter tm;
	while (true)
	{
		tm.reset();
		tm.start();
		int index = 0;
		source >> frame;
		frame +=  Scalar(100,100,100) - mean(frame);
        if (frame.empty()) { cerr << "frame empty!" << endl; break; }
		frame = frame(Rect(Point(0,frame.rows/4*3),Point(frame.cols,frame.rows)));
        cvtColor(frame,frame,COLOR_RGB2GRAY);
		double mindistance =100;
		static Point po = Point(frame.cols/2,frame.rows/2);
		threshold(frame, frame, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
		int lable =  connectedComponentsWithStats(frame, labels, stats, centroids);
		cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

        for(int i = 1;i<lable;i++){
            cv::Scalar sc =stats.at<int>(i,4)<50 ?  cv::Scalar(0,255,255):cv::Scalar(255,0,0);
            cv::rectangle(frame,cv::Rect(stats.at<int>(i,0),stats.at<int>(i,1),stats.at<int>(i,2),stats.at<int>(i,3)),sc);
            cv::rectangle(frame,cv::Rect(centroids.at<double>(i,0),centroids.at<double>(i,1),3,3),sc);
        } 

		for(int i = 1;i<lable;i++){
            if(stats.at<int>(i,4)<50) continue;
			if(sqrt(pow((po.x-centroids.at<double>(i,0)),2)+pow((po.y- centroids.at<double>(i,1)),2))<mindistance){
                mindistance = sqrt(pow((po.x-centroids.at<double>(i,0)),2)+pow((po.y- centroids.at<double>(i,1)),2));
                index = i;
            }
         }
		if(mindistance != 100){ 
			po = Point(centroids.at<double>(index,0),centroids.at<double>(index,1));
			rectangle(frame,Rect(stats.at<int>(index,0),stats.at<int>(index,1),stats.at<int>(index,2),stats.at<int>(index,3)),Scalar(0,0,255));
        	rectangle(frame,Rect(centroids.at<double>(index,0),centroids.at<double>(index,1),3,3),Scalar(0,0,255));
		}
		err = frame.cols - po.x ;
		writer1 <<frame; 
		if (ctrl_c_pressed) break;
		usleep(25 * 1000);
		vw <<frame;
		tm.stop();
		cout<<"error : "<<err <<" lvel : " << 100 +err/5  <<" rvel : " << -100 + err /5 <<" time : " <<tm.getTimeSec() << "Sec" <<endl;
	}
	return 0;
}