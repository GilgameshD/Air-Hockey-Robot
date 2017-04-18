//
//  AHRobotLK.cpp
//  testopencv
//
//  Created by 温拓朴 on 16/2/1.
//  Copyright © 2016年 温拓朴. All rights reserved.
//
#define HEIGHT (480)
#define WIDTH (640)
#define ROIHEIGHT (285)
#define ROIWIDTH (550)
//#define TEST_SERIAL_PORT
#define HSV_CHECK
#define HSV_PUCK

#include "AHRobotLK.hpp"
#include <ctime>

// 图像处理变量
int MAXCOUNT = 500;
Mat frame;
Mat preframe = Mat(ROIWIDTH,ROIHEIGHT,CV_8UC3);
RotatedRect tmpRect;
Mat Res = Mat(ROIWIDTH,ROIHEIGHT,CV_64FC1);
int UpperLine = 15;
int LowerLine = ROIHEIGHT-15;
int RobotLine = 410;
int predict = 0;
int fp;
int slider;
int lowerH,upperH,lowerS,upperS, lowerV,upperV,lpuckH1,lpuckH2,upuckH1,upuckH2,lpuckS,upuckS,lpuckV,upuckV;

bool track = false;
bool learning = false;
bool update = false;
Point2f ContourTracking(Mat, RotatedRect&);
Point2f Predict(Mat&);
Point RobotTracking(Mat&);
void on_trackbar(int&, int);


vector<RotatedRect> Position;

static int first = 0;

// time stamp
DWORD frameTimestamp = 0;
DWORD firstTimestamp = 0;
DWORD oldFrameTimestamp;

// 串口变量
HANDLE serialPort;  // Serial port
BYTE message[16];   // BYTES buffer

// 存储坐标
Point2f pullPosition;
Point robotPosition;
Point2f predictPosition;

bool sendMessage();  // 串口发送函数
bool writeComPort(BYTE *message,int length);
bool openComPort(wchar_t* portSpecifier);
bool readComPort();


int main()
{
	lowerH = 18;
	upperH = 79;
	lowerS = 78;
	upperS = 172;
	lowerV = 80;
	upperV = 181;
	lpuckH1 = 0;
	lpuckH2 = 160;
	upuckH1 = 20;
	upuckH2 = 180;
	lpuckS = 0;
	upuckS = 255;
	lpuckV = 0;
	upuckV = 253;
	
    srand((unsigned)time(NULL));
    FileStorage fs1("Intrinsics.xml", FileStorage::READ);
    FileStorage fs2("Distortion.xml", FileStorage::READ);
    Mat cameraMatrix, distCoeffs2;
    fs1["Intrinsics"] >> cameraMatrix;
    fs2["Distortion"] >> distCoeffs2;
    Mat mapx,mapy,newCameraMatrix;
    initUndistortRectifyMap(cameraMatrix, distCoeffs2, Mat(), cameraMatrix, Size(640,480), CV_32FC1, mapx, mapy);
    //Rectify the image
    
    VideoCapture capture (0);
    Mat oriframe;
    Mat Ensembleframe;
    int count = 0;
	
	// number of COM
	wchar_t auxstr[10] = {'C', 'O', 'M', '4'};
	if(openComPort(auxstr))
		cout << "OPEN SUCCESSFUL" << endl;
	else
		cout << "OPEN FAILED" << endl;

	//------------------------- 串口调试部分 ----------------------------------
#ifdef TEST_SERIAL_PORT
	frameTimestamp = 200;
	firstTimestamp = 100;
	pullPosition = Point(100, 100);
	robotPosition = Point(200, 200);
	predictPosition = Point2f(300, 300);
#endif
	//------------------------------------------------------------------------

    while (1) 
	{
        if (first == 0)
		{
            capture >> oriframe;
            remap(oriframe, Ensembleframe, mapx, mapy, CV_INTER_CUBIC);
            getRectSubPix(Ensembleframe, Size(ROIWIDTH, ROIHEIGHT), Point(WIDTH/2-10,HEIGHT/2+25), frame);

            first++;
        }
        else
		{
            capture >> oriframe;
            remap(oriframe, Ensembleframe, mapx, mapy, CV_INTER_CUBIC);
            getRectSubPix(Ensembleframe, Size(ROIWIDTH, ROIHEIGHT), Point(WIDTH/2-10,HEIGHT/2+25), frame);
        }
		
        if (count == 1)
		{
			// LH=18 UH=79 LS=78 US=172 LV=80 UV=181


			
#ifdef HSV_CHECK
			cvCreateTrackbar("LowerH", "Robot", &lowerH, 180, NULL);
			cvCreateTrackbar("UpperH", "Robot", &upperH, 180, NULL);

			cvCreateTrackbar("LowerS", "Robot", &lowerS, 256, NULL);
			cvCreateTrackbar("UpperS", "Robot", &upperS, 256, NULL);

			cvCreateTrackbar("LowerV", "Robot", &lowerV, 256, NULL);
			cvCreateTrackbar("UpperV", "Robot", &upperV, 256, NULL); 
#endif

#ifdef HSV_PUCK
			cvCreateTrackbar("LowerH1", "Controls", &lpuckH1, 180, NULL);
			cvCreateTrackbar("UpperH1", "Controls", &upuckH1, 180, NULL);

			cvCreateTrackbar("LowerH2", "Controls", &lpuckH2, 256, NULL);
			cvCreateTrackbar("UpperH2", "Controls", &upuckH2, 256, NULL);

			cvCreateTrackbar("LowerS", "Controls", &lpuckS, 256, NULL);
			cvCreateTrackbar("UpperS", "Controls", &upuckS, 256, NULL); 

			cvCreateTrackbar("LowerV", "Controls", &lpuckV, 256, NULL);
			cvCreateTrackbar("UpperV", "Controls", &upuckV, 256, NULL); 
#endif
			
			//--------------------------------------------------------------
			double start = clock();
            robotPosition = RobotTracking(frame); //return Robot's position
			robotPosition.y = ROIHEIGHT - robotPosition.y;
            pullPosition = ContourTracking(frame, tmpRect);//return ball's position
			pullPosition.y = ROIHEIGHT - pullPosition.y;
			predictPosition = Predict(frame);
			predictPosition.y = ROIHEIGHT - predictPosition.y;
			double end = clock();
			cout << (end - start)/CLOCKS_PER_SEC << endl;
			/*predictPosition.x = 400;
			predictPosition.y = 120;*/
			//cout << predictPosition << endl;
			//---------------------------------------------------------------

            count = 0;
            frame.copyTo(preframe);
			line(frame, Point(RobotLine,0), Point(RobotLine, ROIHEIGHT),Scalar(255,255,255),2);
			//line(frame, Point(0,LowerLine), Point(ROIWIDTH, LowerLine),Scalar(255,255,255),2);
			//line(frame, Point(0,UpperLine), Point(ROIWIDTH, UpperLine),Scalar(255,255,255),2);
			//fp++;
			//stringstream os;
			//os << fp;
			//string filename = "video/"+os.str()+".jpg";
			//imwrite(filename, frame);
			imshow("img",frame);
        }
        
        char c = waitKey(1);
        count++;
		
		// 串口读写函数
        sendMessage();
		readComPort();
    }
}

Point2f ContourTracking(Mat frame, RotatedRect &tmpRect)
{
    int RposX = 0;
    int RposY = 0;
    Mat himg[3];
    Mat himgthresh;
    Mat frametmp;
    cvtColor(frame, frametmp, CV_BGR2HSV);
    //split(frametmp,himg);
	
    inRange(frametmp, Scalar(lpuckH1,lpuckS,lpuckV), Scalar(upuckH1,upuckS,upuckV), himg[1]);
    inRange(frametmp, Scalar(lpuckH2,lpuckS,lpuckV), Scalar(upuckH2,upuckS,upuckV), himg[2]);
    add(himg[1], himg[2], himgthresh);
    erode(himgthresh, himgthresh,getStructuringElement(MORPH_CROSS,Size(5,5),Point(-1,-1)),Point(-1,-1),3);
    dilate(himgthresh, himgthresh,getStructuringElement(MORPH_CROSS,Size(3,3),Point(-1,-1)),Point(-1,-1),1);
    //ContourTracking(himgthresh, tmpRect);
			//fp++;
			//stringstream os;
			//os << fp;
			//string filename = "hsv/"+os.str()+".bmp";
			//imwrite(filename,himgthresh);
	imshow("Controls", himgthresh);


    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
	
    findContours(himgthresh, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);
    
	if (!contours.empty())
	{
		for (int it = 0; it != -1; it = hierarchy[it][0])
		{
			double area = contourArea(contours[it]);
			double perimeter = arcLength(contours[it],true);
			double roundness = (perimeter*perimeter)/(6.28*area);

			if ((area > 300) && (area < 1200) && (roundness < 3.5))
			{
				Moments moment = moments(contours[it]);
				//init == 1drawContours(frame, contours, it, Scalar(100,255,255),1);
				RposX = int(double(moment.m10)/double(moment.m00));
				RposY = int(double(moment.m01)/double(moment.m00));
				//line(frame, Point(RposX,RposY), Point(RposX,RposY), Scalar(100,255,100),2);
				tmpRect =  minAreaRect(contours[it]);
				tmpRect.size = Size(35,35);
				rectangle(frame, Point(tmpRect.center.x - 17, tmpRect.center.y - 17), Point(tmpRect.center.x + 17, tmpRect.center.y + 17), Scalar(100,255,100), 2);
				//line(frame, Point(tmpRect.center.x, tmpRect.center.y), Point(tmpRect.center.x, tmpRect.center.y), Scalar(255,255,255), 3);
				//imshow("object", frame);

				/*string filename = "rect/"+os.str()+".jpg";
				imwrite(filename, frame);*/
				if (Position.size() > 5)
				{
					Position.erase(Position.begin());
					Position.push_back(tmpRect);
				}
				else
					Position.push_back(tmpRect);
				return tmpRect.center;
			}
		}
	}
				/*filename = "rect/"+os.str()+".jpg";
				imwrite(filename, frame);*/
    return tmpRect.center;
}

Point RobotTracking(Mat& tmpframe)
{
	//imshow("vec",tmpframe);
    Mat hsv;
    //getRectSubPix(tmpframe, Size(), Point(), hsv);
    tmpframe.copyTo(hsv);
    //Mat h[3];
    cvtColor(hsv, hsv, CV_BGR2HSV);
    //split(hsv, h);
    
    Mat Res;
    int x_pos = 0, y_pos = 0;
	int x_res = 0, y_res = 0;
	
    inRange(hsv, Scalar(lowerH,lowerS,lowerV), Scalar(upperH,upperS,upperV), Res);

    dilate(Res, Res, getStructuringElement(MORPH_CROSS,Size(5,5),Point(-1,-1)),Point(-1,-1),2);
	imshow("Robot", Res);

    vector<vector<Point> > contour;
    vector<Vec4i> hierarchy;
    findContours(Res, contour, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);
    
    double area = 0;
	if (!contour.empty())
	{
		for (int it = 0; it != -1; it = hierarchy[it][0])
		{
			Moments moment = moments(contour[it]);
			x_pos = int(double(moment.m10) / double(moment.m00));
			y_pos = int(double(moment.m01) / double(moment.m00));
			if (contourArea(contour[it]) > area && x_pos > ROIWIDTH/2)
			{
				x_res = x_pos;
				y_res = y_pos;
				area = contourArea(contour[it]);
			}
		}
	}

   // line(tmpframe, Point(x_pos, y_pos), Point(x_pos, y_pos), Scalar(255,255,255),2);
   // imshow("Robot", tmpframe);
    return Point(x_res, y_res);
}

Point2f Predict(Mat& tmpframe)
{
	fp++;
    unsigned long size = Position.size();
    //vector<Point2f> predictPos;
    float RobPosY = 0;
    int n;
    Point2f cur, pre;
    if (size > 3)
	{
        cur = Position[size - 1].center;
        pre = Position[size - 2].center;
        //predictPos.push_back(pre);
        //predictPos.push_back(cur);
        if ((cur.x - pre.x) > 10 || abs(cur.y - pre.y) > 10)
		{
            RobPosY = (cur.y - pre.y) * (RobotLine - cur.x) / (cur.x - pre.x) + cur.y;
//            n = int(RobPosY) / (LowerLine - UpperLine);
            //if (n % 2 == 0)
            //    RobPosY = RobPosY - n * (LowerLine - UpperLine);
            //else
            //    RobPosY = (n + 1) * (LowerLine - UpperLine) - RobPosY;
            int kcur = 0;
            int kpre = int(pre.y);
            for (int i = int(pre.x); i < RobotLine; i=i+2)
			{
                kcur = int( (cur.y - pre.y) * (i - cur.x) / (cur.x - pre.x) + cur.y );
                if (kcur < UpperLine || kcur > LowerLine)
				{
                    if (kcur < UpperLine) 
					{
                        double slope = (cur.y - pre.y) * (i - cur.x) / (cur.x - pre.x);
//                        p00 =      0.6449  (0.5055, 0.7844)
//                        p10 =   -0.004805  (-0.006096, -0.003515)
//                        p01 =      0.5293  (0.4283, 0.6302)
                        kcur = (UpperLine - kcur) * (0.6449 + (cur.y - pre.y) * (-0.004805) + slope * 0.5293) * 1.0 / slope + UpperLine;
                    }
                    if (kcur > LowerLine) 
					{
                        double slope = (cur.y - pre.y) * (i - cur.x) / (cur.x - pre.x);
                        kcur = (LowerLine - kcur) * (0.6449 + (cur.y - pre.y) * (-0.004805) + slope * 0.5293) * 1.0 / slope + LowerLine;
                    }
                }
                line(tmpframe, Point(i - 1,kpre), Point(i, kcur), Scalar(255,255,255),2);
                //predictPos.push_back(Point2f(i, kcur));
                kpre = kcur;
            }
            line(tmpframe, Point(RobotLine, int(kcur)), Point(RobotLine, int(kcur)), Scalar(255,100,100),2);

            imshow("predict", tmpframe);
			//stringstream os;
			//os << fp;
			//string filename = "predict/"+os.str()+".jpg";
			//imwrite(filename,tmpframe);;
            return Point2f(RobotLine,float(kcur));
        }
        else{
			//stringstream os;
			//os << fp;
			//string filename = "predict/"+os.str()+".jpg";
			//imwrite(filename,tmpframe);
            return Point2f(0,0);
		}
    }
    else{
		//stringstream os;
		//os << fp;
		//string filename = "predict/"+os.str()+".jpg";
		//imwrite(filename,tmpframe);;
        return Point2f(0,0);

	}
    
}

bool sendMessage()
{
	DWORD timestamp;

	// start signal
	message[0] = 0x6D;
	message[1] = 0x6D;

	// timestamp, two types to save low 8 bits and high 8 bits
	timestamp = frameTimestamp - firstTimestamp;
	message[2] = (timestamp >> 8) & 0xFF;
	message[3] = timestamp & 0xFF;

	// Pos_X  coordination of x axis of the ball
	message[4] = ((int)pullPosition.x >> 8) & 0xFF;  // high 8 bits
	message[5] = (int)pullPosition.x & 0xFF;         // low 8 bits

	// Pos_Y coordination of y axis of the ball
	message[6] = ((int)pullPosition.y >> 8) & 0xFF;
	message[7] = (int)pullPosition.y & 0xFF;

	// Robot Pos_X coordination of x axis of the puck
	message[8] = ((int)robotPosition.x >> 8) & 0xFF;
	message[9] = (int)robotPosition.x & 0xFF;

	// Robot Pos_Y coordination of y axis of the puck
	message[10] = ((int)robotPosition.y >> 8) & 0xFF;
	message[11] = (int)robotPosition.y & 0xFF;

	// Puck predict position x
	message[12] = ((int)predictPosition.x >> 8) & 0xFF;
	message[13] = (int)predictPosition.x & 0xFF;

	// Puck predict position x
	message[14] = ((int)predictPosition.y >> 8) & 0xFF;
	message[15] = (int)predictPosition.y & 0xFF;

	// Send message (16 bytes)
	return writeComPort(message, 16); 
}

bool writeComPort(BYTE *message,int length)
{
	DWORD byteswritten;
	bool retVal = WriteFile(serialPort, message, length, &byteswritten, NULL);
	return retVal;
}

// serialport operating
bool openComPort(wchar_t* portSpecifier)
{
	DCB dcb;
	serialPort = CreateFile(portSpecifier, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

	if(!GetCommState(serialPort, &dcb))
		return false;

	// Serial port configuration
	dcb.BaudRate = CBR_115200;
	dcb.ByteSize = 8;                       
	dcb.Parity = NOPARITY;                 
	dcb.StopBits = ONESTOPBIT;               
	dcb.fDtrControl = DTR_CONTROL_DISABLE;   

	if (!SetCommState(serialPort, &dcb))
		return false;
	return true;
}

// Read from COM PORT and output to console
bool readComPort()
{
	DWORD dwRead;
	char m_pDataBuf[10000];
	DWORD temp; 
    COMSTAT comstat;
	bool flag = false;
	// Get Serial states
	ClearCommError(serialPort, &temp, &comstat);

	// New bytes pending read or not 
	if (comstat.cbInQue > 0)
	{
		flag = ReadFile(serialPort, m_pDataBuf, comstat.cbInQue, &dwRead, NULL);
		m_pDataBuf[dwRead] = 0;
		cout << m_pDataBuf << endl;
	}
	return flag;
}