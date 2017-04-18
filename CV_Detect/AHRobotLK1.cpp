//
//  AHRobotLK.cpp
//  testopencv
//
//  Created by 温拓朴 on 16/2/1.
//  Copyright © 2016年 温拓朴. All rights reserved.
//
#define AHROBOTLK
#ifdef AHROBOTLK
#define HEIGHT (480)
#define WIDTH (640)
#define ROIHEIGHT (280)
#define ROIWIDTH (550)

#include "AHRobotLK.hpp"


int MAXCOUNT = 500;
Mat frame;
Mat preframe = Mat(ROIWIDTH,ROIHEIGHT,CV_8UC3);
RotatedRect tmpRect;
Mat Res = Mat(ROIWIDTH,ROIHEIGHT,CV_64FC1);
int N = 5;
int K = 4;
int fp = 0;
int init = 0;
int PRes = 0;
int NRes = 0;
int UpperLine = 70;
int LowerLine = 390;
int RobotLine = 520;
int fps = 0;
int predict=0;
String filename;
AdaBoost adaClassifier;

bool track =false;
bool learning = false;
bool update = false;
void trackObjectRobot(Mat );
bool LKTracking(Mat,Mat,RotatedRect&);
bool ContourTracking(Mat, RotatedRect&);
void trajectoryPredict(vector<RotatedRect> Position, RotatedRect &tmpRect);
float NCCComparison(RotatedRect tmpRect);
void initialize();
void Detection(Mat);
Point2f Predict(Mat&);
void Ada_Training(Point);
void Ada_Update(Point);
float SVMpredict(Point2f);
void EnsembleSVM();
void SavePResult(Point2f p,int fp);
void SaveNResult(Point2f p,int fp);
void InitSamples(RotatedRect rect, vector<Mat>& sampleQuery, Mat& labelQuery);
Mat patch2Vec(RotatedRect rect);
void RobotTracking(Mat);
void viewCurPositionVec(Mat frame);

vector<RotatedRect> Position;

float median(vector<float> v)
{
    if (v.size() == 0)
        return 10000000;
    else{
        int n = floor(v.size() / 2);
        nth_element(v.begin(), v.begin()+n, v.end());
        return v[n];
    }
}

static int first = 0;
int main(){
    srand((unsigned)time(NULL));
    FileStorage fs1("Intrinsics.xml", FileStorage::READ);
    FileStorage fs2("Distortion.xml", FileStorage::READ);
    Mat cameraMatrix, distCoeffs2;
    fs1["Intrinsics"] >> cameraMatrix;
    fs2["Distortion"] >> distCoeffs2;
    Mat mapx,mapy,newCameraMatrix;
    initUndistortRectifyMap(cameraMatrix, distCoeffs2, Mat(), cameraMatrix, Size(640,480), CV_32FC1, mapx, mapy);
    //Rectify the image
    
    
    VideoCapture capture (1);
    Mat frametmp = Mat(ROIWIDTH,ROIHEIGHT,CV_8UC3);
    vector<Mat> himg;//= Mat(ROIWIDTH,ROIHEIGHT,CV_8UC1);
    Mat himgthresh = Mat(ROIWIDTH,ROIHEIGHT,CV_8UC1);
    Mat himgthresh1 = Mat(ROIWIDTH,ROIHEIGHT,CV_8UC3);
    Mat himgthresh2 = Mat(ROIWIDTH,ROIHEIGHT,CV_8UC3);
    Mat oriframe;
    Mat Ensembleframe;
    int count = 0;
    const string winname = "befo";
    while (1) {
        if (first == 0){
            capture >> oriframe;
            remap(oriframe, Ensembleframe, mapx, mapy, CV_INTER_CUBIC);
            //imwrite("/Users/programath/Desktop/ori/1.jpg", oriframe);
            getRectSubPix(Ensembleframe, Size(ROIWIDTH, ROIHEIGHT), Point(WIDTH/2-10,HEIGHT/2+30),frame);
            
            first++;
        }
        else{
            //frame.copyTo(preframe);
            capture >> oriframe;
            remap(oriframe, Ensembleframe, mapx, mapy, CV_INTER_CUBIC);
            
            getRectSubPix(Ensembleframe, Size(ROIWIDTH, ROIHEIGHT), Point(WIDTH/2-10,HEIGHT/2+30), frame);
                    }
        //if (count == 4){
            //cvtColor(frame, frametmp, CV_BGR2HSV);
            //split(frametmp,himg);
            //inRange(himg[0], Scalar(160,0,0), Scalar(180,0,0), himgthresh1);
            //inRange(himg[0], Scalar(0,0,0), Scalar(20,0,0), himgthresh2);
            //add(himgthresh1, himgthresh2, himgthresh);
            //erode(himgthresh, himgthresh,getStructuringElement(MORPH_CROSS,Size(5,5),Point(-1,-1)),Point(-1,-1),3);
            //dilate(himgthresh, himgthresh,getStructuringElement(MORPH_CROSS,Size(3,3),Point(-1,-1)),Point(-1,-1),1);
            //imshow(winname, himgthresh);
            //imshow("origin", oriframe);
//            stringstream os;
//            os << fps;
//            imwrite("/Users/programath/Desktop/hsv/" + os.str() + ".jpg", himgthresh);
//            fps++;
            //cvDilate(himgthresh, himgthresh);
            //trackObjectRobot(himgthresh);
            //RobotTracking(frame);
            //Detection(frame);
            imshow("hh", frame);
            //cvShowImage("Thresh", himgthresh);
            count = 0;
            frame.copyTo(preframe);
        //}
        
        char c = waitKey(1);
        if (c=='p'){
            //cvWaitKey(0);
            track = true;
        }
        if (c=='l'){
            learning = true;
        }
        if (c == 'v'){
            viewCurPositionVec(frame);
        }
        //count++;
        
    }
}

void trackObjectRobot(Mat imgThresh){
    
    vector<vector<Point>> contours;  //hold the pointer to a contour in the memory block
    //CvSeq* result;   //hold sequence of points of a contour
    //CvMemStorage *storage = cvCreateMemStorage(0); //storage area for all contours
    
    // Position initialization
    RotatedRect rect1;
    RotatedRect rect2;
    RotatedRect rect3;
    //int RobjectSize = 0;
    vector<int> status(100);
    Mat tmpframe;
    frame.copyTo(tmpframe);
    
    vector<Mat> sampleQuery;
    Mat labelQuery;
    
    if (track){
        rect2 = tmpRect;
        rect1 = RotatedRect(Point(0,0),Size(0,0),0);
        LKTracking(preframe, frame, rect2);
        ContourTracking(imgThresh, rect1);
        trajectoryPredict(Position, rect3);
        Point2f Poscur = Point2f(Position[int(Position.size()) - 1].center);
        //Point2f Pospre = Point2f(Position[int(Position.size()) - 1].center);
        //double dist = abs(Poscur.x - Pospre.x) + abs(Poscur.y - Pospre.y);
        bool flag = false;
        if (init == 1 || (rect1.center.x != 0 && rect1.center.y != 0 && abs(rect1.center.x - Poscur.x) + abs(rect1.center.y - Poscur.y) < 300 && SVMpredict(rect1.center) == 1)){
            tmpRect = rect1;
            flag = true;
            //InitSamples(tmpRect, sampleQuery, labelQuery);
            //adaClassifier.AdaTrain(sampleQuery, labelQuery);
            init = 0;
        }
        else
            if (rect2.center.x != 0 && rect2.center.y != 0 && abs(rect2.center.x - rect3.center.x) + abs(rect2.center.y - rect3.center.y) < 75 && (SVMpredict(rect2.center) == 1)){
                tmpRect = rect2;
                flag = true;
            }
            else {
                vector<float> xpos;
                vector<float> ypos;
                for (int i = -35; i < 35; i = i + 5)
                    for (int j = -35; j < 35; j = j + 5){
                        if (SVMpredict(Point2f(rect3.center.x + i,rect3.center.y + j)) == 1) {
                            xpos.push_back(rect3.center.x + i);
                            ypos.push_back(rect3.center.y + j);
                            line(tmpframe, Point(rect3.center.x + i,rect3.center.y + j), Point(rect3.center.x + i,rect3.center.y + j), Scalar(255,255,255));
                        }
                    }
                if (xpos.size() != 0){
                    rect3.center.x = median(xpos);
                    rect3.center.y = median(ypos);
                    tmpRect = rect3;
                    flag = true;
                    xpos.resize(0);
                    ypos.resize(0);
                }
                else{
                    initialize();
                    rect2.center = Point2f(0,0);
                    rect3.center = Point2f(0,0);
                    flag = false;
                }
            }
        
        
        //if (init != 1){
            
            //cout << adaClassifier.predict(patch2Vec(rect1)) << endl;
            //cout << adaClassifier.predict(patch2Vec(rect2)) << endl;
            //cout << adaClassifier.predict(patch2Vec(rect3)) << endl;
            //InitSamples(tmpRect, sampleQuery, labelQuery);
            //adaClassifier.AdaUpdate(sampleQuery, labelQuery);
            
        //}
        
//        rectangle(tmpframe, Point(rect1.center.x - rect1.size.width / 2, rect1.center.y - rect1.size.height / 2), Point(rect1.center.x + rect1.size.width / 2, rect1.center.y + rect1.size.height / 2), Scalar(0,255,255),2);
//        rectangle(tmpframe, Point(rect2.center.x - rect2.size.width / 2, rect2.center.y - rect2.size.height / 2), Point(rect2.center.x + rect2.size.width / 2, rect2.center.y + rect2.size.height / 2), Scalar(255,0,255),2);
//        rectangle(tmpframe, Point(rect3.center.x - rect3.size.width / 2, rect3.center.y - rect3.size.height / 2), Point(rect3.center.x + rect3.size.width / 2, rect3.center.y + rect3.size.height / 2), Scalar(255,255,0),2);
        //rectangle(frame, Point(tmpRect.center.x - tmpRect.size.width / 2, tmpRect.center.y - tmpRect.size.height / 2), Point(tmpRect.center.x + tmpRect.size.width / 2, tmpRect.center.y + tmpRect.size.height / 2), Scalar(255,255,255),2);
        if (flag){
//            rectangle(tmpframe, Point(rect1.center.x - rect1.size.width / 2, rect1.center.y - rect1.size.height / 2), Point(rect1.center.x + rect1.size.width / 2, rect1.center.y + rect1.size.height / 2), Scalar(0,255,255),2);
//            rectangle(tmpframe, Point(rect2.center.x - rect2.size.width / 2, rect2.center.y - rect2.size.height / 2), Point(rect2.center.x + rect2.size.width / 2, rect2.center.y + rect2.size.height / 2), Scalar(255,0,255),2);
//            rectangle(tmpframe, Point(rect3.center.x - rect3.size.width / 2, rect3.center.y - rect3.size.height / 2), Point(rect3.center.x + rect3.size.width / 2, rect3.center.y + rect3.size.height / 2), Scalar(255,255,0),2);
            rectangle(tmpframe, Point(tmpRect.center.x - tmpRect.size.width / 2, tmpRect.center.y - tmpRect.size.height / 2), Point(tmpRect.center.x + tmpRect.size.width / 2, tmpRect.center.y + tmpRect.size.height / 2), Scalar(0,255,255),2);
            Position.push_back(tmpRect);
            cout << tmpRect.center << endl;
            
            //line(tmpframe, Point(0, 80), Point(640, 80), Scalar(255,255,255),2);
            //line(tmpframe, Point(0, 410), Point(640, 410), Scalar(255,255,255),2);
            //line(tmpframe, Point(520, 0), Point(520, ROIWIDTH), Scalar(255,255,255),2);
            Predict(tmpframe);
        }
        imshow("Contour", tmpframe);
        
//        stringstream os;
//        os << fps;
//        imwrite("/Users/programath/Desktop/video/" + os.str() + ".jpg", tmpframe);
//        fps++;
        //waitKey(0);
    }
    else{
        ContourTracking(imgThresh, tmpRect);
        Position.push_back(tmpRect);
        rectangle(tmpframe, tmpRect.center, tmpRect.center, Scalar(100,200,200));
        imshow("result", tmpframe);
        
        cout << tmpRect.center << endl;
        //waitKey(10);
    }
}

bool ContourTracking(Mat imgThresh, RotatedRect &tmpRect){
    int RposX = 0;
    int RposY = 0;
    Mat tmpframe;
    frame.copyTo(tmpframe);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(imgThresh, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);
    
    for (int it = 0; it != -1; it = hierarchy[it][0]){
        double area = contourArea(contours[it]);
        double perimeter = arcLength(contours[it],true);
        double roundness = (perimeter*perimeter)/(6.28*area);
        if ((area > 300) && (area < 1200) && (roundness < 3.5)){
            Moments moment = moments(contours[it]);
            //init == 1drawContours(frame, contours, it, Scalar(100,255,255),1);
            RposX = int(double(moment.m10)/double(moment.m00));
            RposY = int(double(moment.m01)/double(moment.m00));
            //line(frame, Point(RposX,RposY), Point(RposX,RposY), Scalar(100,255,100),2);
            tmpRect =  minAreaRect(contours[it]);
            tmpRect.size = Size(35,35);
            
//            rectangle(tmpframe, Point(tmpRect.center.x - tmpRect.size.width / 2, tmpRect.center.y - tmpRect.size.height / 2), Point(tmpRect.center.x + tmpRect.size.width / 2, tmpRect.center.y + tmpRect.size.height / 2), Scalar(0,255,255),2);
//            imshow("Contour", tmpframe);
            return true;
        }
    }
    return false;
}

void InitSamples(RotatedRect rect, vector<Mat>& sampleQuery, Mat& labelQuery){
    float xoff,yoff,angle;
    int Postotal = 200;
    int Negtotal = 300;
    labelQuery = Mat(1, Postotal + Negtotal, CV_32FC1);
    for (int i = 0; i < 200; ++i){
        xoff = rand() % 12 - 6;
        yoff = rand() % 12 - 6;
        angle = rand() % 720 - 360;
        Mat M, rotated, cropped;
        rect.angle = angle;
        rect.center.x += xoff;
        rect.center.y += yoff;
        if (rect.angle < -45. ){
            angle += 90.0;
        }
        M = getRotationMatrix2D(rect.center, angle, 1.0);
        warpAffine(frame, rotated, M, frame.size(), INTER_CUBIC);
        getRectSubPix(rotated, rect.size, rect.center, cropped, CV_32FC1);
        sampleQuery.push_back(cropped);
        labelQuery.at<float>(0, i) = 1.0;
        
    }
    
    Mat patch;
    for (int i = 0; i < 300; i = i + 4){
        xoff = rand() % 35 + 20;
        yoff = rand() % 35 + 20;
        getRectSubPix(frame, rect.size, Point2f(rect.center.x + xoff,rect.center.y + yoff), patch, CV_32FC1);
        sampleQuery.push_back(patch);
        labelQuery.at<float>(0, i + 0 + Postotal) = -1.0;
        getRectSubPix(frame, rect.size, Point2f(rect.center.x + xoff,rect.center.y - yoff), patch, CV_32FC1);
        sampleQuery.push_back(patch);
        labelQuery.at<float>(0, i + 1 + Postotal) = -1.0;
        getRectSubPix(frame, rect.size, Point2f(rect.center.x - xoff,rect.center.y - yoff), patch, CV_32FC1);
        sampleQuery.push_back(patch);
        labelQuery.at<float>(0, i + 2 + Postotal) = -1.0;
        getRectSubPix(frame, rect.size, Point2f(rect.center.x - xoff,rect.center.y + yoff), patch, CV_32FC1);
        sampleQuery.push_back(patch);
        labelQuery.at<float>(0, i + 3 + Postotal) = -1.0;
    }
}

bool LKTracking(Mat pre, Mat cur, RotatedRect &Box){
    Mat ptmp;
    Mat ctmp;
    pre.copyTo(ptmp);
    cur.copyTo(ctmp);
    int width = Box.size.width;
    int height = Box.size.height;
    //int count = (1 + (height - 1) / 6) * (1 + (width - 1) / 6);
    Point lefttop = Point(Box.center.x - Box.size.width/2, Box.center.y - Box.size.height/2);
    vector<Point2f> Point1;
    
    for (int i = 0; i < 6; ++i ){
        for (int j = 0; j < 6; ++j){
            Point1.push_back(Point2f(double(lefttop.x + j * width / 6),double(lefttop.y + i * height / 6)));
            //line(frame, *Point1.end(), *Point1.end(), Scalar(255,255,255),2);
        }
        
    }
    vector<Point2f> Point2(Point1.size());
    vector<Point2f> PointFB(Point1.size());
    vector<uchar> status(Point1.size());
    vector<uchar> FB_status(Point1.size());
    vector<float> similarity(Point1.size());
    vector<float> FB_error(Point1.size());
    
    calcOpticalFlowPyrLK(pre, cur, Point1, Point2, status, similarity, Size(6,6), 5, TermCriteria( TermCriteria::COUNT+TermCriteria::EPS, 20, 0.03), 0.5, 0);
    calcOpticalFlowPyrLK(cur, pre, Point2, PointFB, FB_status, FB_error, Size(6,6), 5, TermCriteria( TermCriteria::COUNT+TermCriteria::EPS, 20, 0.03), 0.5, 0);
    
    for (int i = 0; i < Point1.size(); ++i){
        FB_error[i] = norm(Point1[i]-PointFB[i]);
    }
    
    Mat rec0(10,10,CV_8U);
    Mat rec1(10,10,CV_8U);
    Mat res(1,1,CV_32F);
    
    for (int i = 0; i < Point1.size(); ++i){
        if (status[i] == 1){
            getRectSubPix( pre, Size(4,4), Point1[i],rec0 );
            getRectSubPix( cur, Size(4,4), Point2[i],rec1 );
            matchTemplate( rec0,rec1, res, CV_TM_CCOEFF_NORMED);
            similarity[i] = ((float *)(res.data))[0];
        }
        else
            similarity[i] = 0.0;
    }
    
    rec0.release();
    rec1.release();
    res.release();
    
    int k;
    float Thresherror = median(similarity);
    float Thresh = median(FB_error);
    
    for (int i = k = 0; i < Point1.size(); ++i){
        if (FB_error[i] < Thresh && similarity[i] > Thresherror){
            Point1[k] = Point1[i];
            Point2[k] = Point2[i];
            ++k;
        }
    }
    cout << Point1.size() <<" "<< k << endl;
    
    if (k == 0){
        Box.center = Point2f(0,0);
        Box.size = Size(0,0);
        return false;
    }
    
    Point1.resize(k);
    Point2.resize(k);
    for (int i = 0; i < Point1.size(); ++i){
        line(ptmp, Point(int(Point1[i].x), int(Point1[i].y)), cvPoint(int(Point1[i].x), int(Point1[i].y)), Scalar(255,100,100),2);
        line(ctmp, Point(int(Point2[i].x), int(Point2[i].y)), cvPoint(int(Point2[i].x), int(Point2[i].y)), Scalar(100,255,100),2);
        //line(frame, Point(int(PointFB[i].x), int(PointFB[i].y)), cvPoint(int(PointFB[i].x), int(PointFB[i].y)), Scalar(0,0,255),2);
    }
    
    vector<float> xoff;
    vector<float> yoff;
    for (int i = 0; i < Point1.size(); ++i){
        xoff.push_back(Point2[i].x - Point1[i].x);
        yoff.push_back(Point2[i].y - Point1[i].y);
    }
    Box.center.x += median(xoff);
    Box.center.y += median(yoff);
    Box.size = Size(32,32);
    
//    rectangle(ctmp, Point(Box.center.x - Box.size.width / 2, Box.center.y - Box.size.height / 2), Point(Box.center.x + Box.size.width / 2, Box.center.y + Box.size.height / 2), Scalar(255,0,255),2);
    
    //namedWindow("L_Kc");
    //imshow("L_Kp", ctmp);
    //namedWindow("L_Kp");
    //imshow("L_Kp", ptmp);
    //cvWaitKey(0);
    return true;
    
}

void trajectoryPredict(vector<RotatedRect> Position, RotatedRect &tmpRect){
    if (Position.size() < 3)
        return;
    Mat tmpframe;
    frame.copyTo(tmpframe);
    int n = int(Position.size());
    RotatedRect poscur = Position[n - 1];
    RotatedRect pospre = Position[n - 2];
    tmpRect = RotatedRect(poscur);
    tmpRect.center = Point2f(2 * poscur.center.x - pospre.center.x, 2 * poscur.center.y - pospre.center.y);
    if (tmpRect.center.x < 0) tmpRect.center.x *= -1;
    if (tmpRect.center.y > LowerLine) tmpRect.center.y = 2 * LowerLine - tmpRect.center.y;
    if (tmpRect.center.x > ROIHEIGHT) tmpRect.center.x = 2 * ROIHEIGHT - tmpRect.center.x;
    if (tmpRect.center.y < UpperLine) tmpRect.center.y = 2 * UpperLine - tmpRect.center.y;
    tmpRect.size = Size(32,32);
//    rectangle(tmpframe, Point(tmpRect.center.x - tmpRect.size.width / 2, tmpRect.center.y - tmpRect.size.height / 2), Point(tmpRect.center.x + tmpRect.size.width / 2, tmpRect.center.y + tmpRect.size.height / 2), Scalar(255,255,0),2);
//    imshow("Contour", tmpframe);
}

float NCCComparison(RotatedRect tmpRect){
    Mat res(1,1,CV_32F);
    Mat rec0;
    Mat rec1;
    int n = int(Position.size());
    int count = 0;
    float similarity;
    for (int i = 1; i < 6; ++i){
        int k = rand() % 10 + 1;
        getRectSubPix( frame, Size(32,32), Position[n - k].center,rec0 );
        getRectSubPix( frame, Size(32,32), tmpRect.center,rec1 );
        matchTemplate( rec0,rec1, res, CV_TM_CCOEFF_NORMED);
        similarity = ((float *)(res.data))[0];
        if (similarity > 0.5){
            count++;
        }
    }
    return (count > 2 ? 1 : 0);
}

void initialize(){
    Position.resize(0);
    init = 1;
    
}

void RobotTracking(Mat tmpframe){
    Mat hsv;
    tmpframe.copyTo(hsv);
    imshow("win", tmpframe);
    Mat h[3];
    cvtColor(hsv, hsv, CV_BGR2HSV);
    split(hsv, h);
    //imshow("tmpframe", hsv);
    Mat Res;
    int x_pos = 0, y_pos = 0;
    int cnt =0;
    inRange(hsv, Scalar(40,120,50), Scalar(70,200,140), Res);
    for (int i = 0; i < Res.rows; ++i){
        for (int j = 0; j < Res.cols; ++j){
            if (Res.at<uchar>(i, j) != 0){
                x_pos += j;
                y_pos += i;
                cnt++;
            }
        }
    }
    x_pos /= (cnt+1);
    y_pos /= (cnt+1);
    line(tmpframe, Point(x_pos, y_pos), Point(x_pos, y_pos), Scalar(100,100,100),2);
    imshow("Robot", tmpframe);
}

Point2f Predict(Mat& tmpframe){
    unsigned long size = Position.size();
    vector<Point2f> predictPos;
    float RobPosY = 0;
    int n;
    Point2f cur, pre;
    if (size > 3){
        cur = Position[size - 1].center;
        pre = Position[size - 2].center;
        predictPos.push_back(pre);
        predictPos.push_back(cur);
        if (cur.x - pre.x > 10 && abs(cur.x - pre.x) + abs(cur.y - pre.y) > 7){
            RobPosY = (cur.y - pre.y) * (RobotLine - cur.x) / (cur.x - pre.x) + cur.y;
            n = int(RobPosY) / (LowerLine - UpperLine);
            if (n % 2 == 0)
                RobPosY = RobPosY - n * (LowerLine - UpperLine);
            else
                RobPosY = (n + 1) * (LowerLine - UpperLine) - RobPosY;
            int kcur = 0;
            int kpre = int(pre.y);
            for (int i = int(pre.x); i < RobotLine; ++i){
                kcur = int( (cur.y - pre.y) * (i - cur.x) / (cur.x - pre.x) + cur.y );
                while (kcur < UpperLine || kcur > LowerLine){
                    if (kcur < UpperLine ) {
                        double slope = (cur.y - pre.y) * (i - cur.x) / (cur.x - pre.x);
//                        p00 =      0.6449  (0.5055, 0.7844)
//                        p10 =   -0.004805  (-0.006096, -0.003515)
//                        p01 =      0.5293  (0.4283, 0.6302)
                        kcur = (UpperLine - kcur) * (0.6449 + (cur.y - pre.y) * (-0.004805) + slope * 0.5293) * 1.0 / slope + UpperLine;
                    }
                    if (kcur > LowerLine ) {
                        double slope = (cur.y - pre.y) * (i - cur.x) / (cur.x - pre.x);
                        kcur = (LowerLine - kcur) * (0.6449 + (cur.y - pre.y) * (-0.004805) + slope * 0.5293) * 1.0 / slope + LowerLine;
                    }
                }
                line(tmpframe, Point(i - 1,kpre), Point(i, kcur), Scalar(255,255,255),2);
                predictPos.push_back(Point2f(i, kcur));
                kpre = kcur;
            }
            line(tmpframe, Point(RobotLine, int(kcur)), Point(RobotLine, int(kcur)), Scalar(255,100,100),2);
            imshow("predict", tmpframe);
            //waitKey(10);
            return Point2f(RobotLine,float(kcur));
        }
        else
            return Point2f(0,0);
    }
    else
        return Point2f(0,0);
    
}

void viewCurPositionVec(Mat tmpframe){
    for (int i = Position.size() - 20; i < Position.size(); ++i){
        line(tmpframe, Point(int(Position[i].center.x),int(Position[i].center.y)),Point(int(Position[i].center.x),int(Position[i].center.y)),Scalar(100, 200, 150), 2);
    }
    imshow("view", tmpframe);
    waitKey(0);
}


void EnsembleSVM(){
    Mat tmpframe;
    frame.copyTo(tmpframe);
    for (int i = 18; i < frame.rows - 18; ++i)
        for (int j = 18; j < frame.cols - 18; ++j){
            if (SVMpredict(Point2f(j, i)) == 1){
                line(tmpframe, Point(j,i), Point(j,i), Scalar(255,0,0));
            }
            else
                line(tmpframe, Point(j,i), Point(j,i), Scalar(0,0,255));
        }
    imshow("SVM", tmpframe);
    waitKey(0);
}

float SVMpredict(Point2f p){
    Mat patch;
    Mat oripatch;
    Mat hsv[3];
    hsv[0] = Mat(35,35,CV_8UC1);
    hsv[1] = Mat(35,35,CV_8UC1);
    hsv[2] = Mat(35,35,CV_8UC1);

    getRectSubPix(frame, Size(35,35), p, oripatch);
    CvSVM svm;
    svm.load("SVM_DATA.xml");
    Mat Sample = Mat(1, 35*35*2, CV_32FC1);
    Mat graypatch = Mat(35,35,CV_8UC1);
    //Mat normpatch = Mat(Size(35,35), CV_32FC1);
    cvtColor(oripatch, patch, CV_BGR2HSV);
    split(patch, hsv);
    hsv[0].copyTo(graypatch);
    graypatch.convertTo(graypatch, CV_32FC1);
    //normalize(graypatch, graypatch,1.0,0.0,CV_MINMAX);
    for (int i = 0; i < graypatch.rows; ++i)
        for (int j = 0; j < graypatch.cols; ++j){
            Sample.at<float>(0, i * graypatch.cols + j) = graypatch.at<float>(i, j);
        }
    hsv[1].copyTo(graypatch);
    graypatch.convertTo(graypatch, CV_32FC1);
    //normalize(graypatch, graypatch,1.0,0.0,CV_MINMAX);
    for (int i = 0; i < graypatch.rows; ++i)
        for (int j = 0; j < graypatch.cols; ++j){
            Sample.at<float>(0, 35 * 35 + i * graypatch.cols + j) = graypatch.at<float>(i, j);
        }
    return svm.predict(Sample);
//    stringstream os;
//    os << predict;
//    if (svm.predict(Sample) == 1 ){
//        string file = "/Users/programath/Desktop/psvm/" + os.str() + ".jpg";
//        imwrite(file, oripatch);
//        predict++;
//        return 1;
//    }
//    else {
//        string file = "/Users/programath/Desktop/nsvm/" + os.str() + ".jpg";
//        imwrite(file, oripatch);
//        predict++;
//        return -1;
//    }
}

void SavePResult(Point2f p,int fp){
    Mat roi;
    stringstream os;
    getRectSubPix(frame, Size(35,35), p, roi);
    os << fp;
    filename = "/Users/programath/Desktop/PResult/" + os.str() + ".jpg";
    cout << filename << endl;
    imwrite(filename, roi);
}
void SaveNResult(Point2f p,int fp){
    Mat roi;
    stringstream os;
    getRectSubPix(frame, Size(35,35), p, roi);
    os << fp;
    filename = "/Users/programath/Desktop/NResult/" + os.str() + ".jpg";
    cout << filename << endl;
    imwrite(filename, roi);
}

Mat patch2Vec(RotatedRect rect){
    Mat patch;
    getRectSubPix(frame, rect.size, rect.center, patch, CV_32FC1);
    Mat Sample = Mat(1, patch.cols * patch.rows, CV_32FC1);
    for (int i = 0; i < patch.rows; ++i)
        for (int j = 0; j < patch.cols; ++j)
            Sample.at<float>(0, i * patch.cols + j) = patch.at<float>(i,j);
    return Sample;
}


AdaBoost::AdaBoost(){
    Samples = Mat(SAMPLECOUNT, COL * ROW, CV_32FC1);
    labels = Mat(1, SAMPLECOUNT, CV_32FC1);
    reslabels = Mat(1, SAMPLECOUNT, CV_32FC1);
}

void vecNorm(vector<float>& weight){
    float sum = 0;
    for (int i = 0; i < weight.size(); ++i){
        sum += weight[i];
    }
    for (int i = 0; i < weight.size(); ++i){
        weight[i] /= sum;
    }
}

void AdaBoost::Sampling(vector<Mat> sampleQuery, Mat labelQuery){
    for (int i = 0; i < Samples.rows; ++i){
        float choice = rand();
        int k = 0;
        while (choice > 0){
            choice -= SampleWeight[k];
            ++k;
        }
        for (int j = 0; j < sampleQuery[0].rows; ++j)
            for (int l = 0; l < sampleQuery[0].cols; ++l)
                Samples.at<float>( i,j * sampleQuery[0].cols + l ) = sampleQuery[k].at<float>(j, l);
        
        labels.at<float>( 0, i ) = labelQuery.at<float>(0, i);
    }
    
}

void AdaBoost::CalcuEpsilon(CvDTree* tree){
    epsilon = 0;
    reslabels = Mat(1,labels.cols,CV_32FC1);
    for (int i = 0; i < Samples.rows; ++i){
        Mat patch = Mat(1, Samples.cols, CV_32FC1);
        for (int j = 0; j < Samples.cols; ++j){
            patch.at<float>(0, j) = Samples.at<float>( i ,j);
            cout << tree->predict(patch)->value << endl;
            if (tree->predict(patch)->value < 0){
                
                reslabels.at<float>(0, j) = 1.0;
                epsilon += SampleWeight[i];
            }
            else {
                reslabels.at<float>(0, j) = -1.0;
            }
        }
    }
    alpha = 0.5 * log( (1 - epsilon) / epsilon );
}

void AdaBoost::weightUpdate(){
    for (int i = 0; i < SampleWeight.size(); ++i)
        SampleWeight[i] *= exp(-alpha * labels.at<float>(0,i) * reslabels.at<float>(0,i));
    vecNorm(SampleWeight);
}

int AdaBoost::predict(Mat Sample){
    float score = 0;
    for (int i = 0; i < gtree.size(); ++i)
        score += gtree[i].weight * gtree[i].dtree->predict(Sample)->value;
    return score > 0 ? 1 : -1;
}


void AdaBoost::AdaTrain(vector<Mat> InitSamples, Mat labelQuery){
    int SampleCount = Samples.rows;
    int SampleSize = Samples.cols;
    int TreeCount = 30;
    CvDTree *dt;
    
    for (int i = 0; i < SampleCount; ++i){
        SampleWeight.push_back(1.0/SampleCount);
    }
    for (int i = 0; i < 30; ++i ){
        dt = CreateTree(Samples, labels);
        CalcuEpsilon(dt);
        weightUpdate();
        Sampling(InitSamples, labelQuery);
        gtree.push_back(WeakDTree(alpha,dt));
    }
    sort(gtree.begin(), gtree.end(), Cmp);
}

void AdaBoost::AdaUpdate(vector<Mat> NewSamples, Mat labelQuery){
    int SampleCount = Samples.rows;
    int SampleSize = Samples.cols;
    CvDTree *dt;
    gtree.resize(30 - 20);
    for (int i = 0; i < SampleCount; ++i){
        SampleWeight.push_back(1.0/SampleCount);
    }
    
    for (int i = 0; i < gtree.size(); ++i){
        CalcuEpsilon(gtree[i].dtree);
        weightUpdate();
        Sampling(NewSamples, labelQuery);
        gtree[i].weight = alpha;
    }
    for (int i = 0; i < 20; ++i){
        dt = CreateTree(Samples, labels);
        CalcuEpsilon(dt);
        weightUpdate();
        Sampling(NewSamples, labelQuery);
        gtree.push_back(WeakDTree(alpha,dt));
    }
    sort(gtree.begin(), gtree.end(), Cmp);
}

CvDTree* CreateTree(const Mat& data, const Mat& response){
    int maxDepth = 2;
    int minsampleCount = 30;
    //float pweight = 1;
    //float priors[] = {1, pweight};
    Mat var_type = Mat(data.cols + 1, 1, CV_8UC1, Scalar::all(0));
    Mat missing = Mat(data.rows, data.cols, CV_8UC1, 0);
    Mat sampleidx = Mat(data.rows, 1, CV_8UC1, 1);
    Mat varidx = Mat(data.cols, 1, CV_8UC1, 1);
    CvDTree* dtree;
    dtree = new CvDTree;
    
    dtree->train( data, CV_ROW_SAMPLE, response, Mat(), Mat(), var_type, Mat(), CvDTreeParams(maxDepth, minsampleCount, 0.01, false, 2, 1, true, true, 0));
    //Mat var_importance = dtree->getVarImportance();
    //cout << var_importance << endl;
    //var_importance.convertTo(var_importance, CV_32FC1);
    return dtree;
}

void AdaBoost::SampleUpdate(vector<Mat> sampleQuery, Mat labelQuery){
    for (int i = 0; i < sampleQuery.size(); ++i){
        for (int j = 0; j < 35; ++j)
            for (int k = 0; k < 35; ++k){
                Samples.at<float>(i, j * 35 + k) = sampleQuery[i].at<uchar>(j, k);
            }
        labels.at<float>(0,i) = labelQuery.at<float>(0,i);
    }
}



#endif