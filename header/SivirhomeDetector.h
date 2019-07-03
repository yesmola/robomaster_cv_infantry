#ifndef _SIVIRHOMEDETECTOR_H_
#define _SIVIRHOMEDETECTOR_H_
#include<opencv2/opencv.hpp>
#include<vector>
#include"config.h"

using namespace cv;
using namespace std;

typedef struct ARMOR3
{
    Point2f center;
    Point2f rect[4];
}Armor3;

typedef struct ANGLE3
{
   // float x,y,z;
   float yaw,pitch;
}Angle3;

typedef struct SCIRCLE
{
    Point2f center;
    float r;
}Scircle;

class SivirhomeDetector
{
public:
    Mat src;//source image
    Mat binary;//binary image
    Mat outline;//outline image
    bool islost;
    vector< vector<Point> > contours;
    int num;
    Point2f small[10];
    Point2f big[10];
    int cntsmall;
    int cntbig;
    Armor3 target;
    Angle3 pnpresult;
    Point2f allrect[100][4];
    
    Point2f predict[400];
    int prep;
    Scircle heart;

public: 
    SivirhomeDetector();
    SivirhomeDetector(Mat src0);
    void getResult(Mat src0);
    void getSrcImage(Mat src0);
    void getBinaryImage();
    void getContours();
    void getTarget();
    
    void getPnP();

    Scircle getCpoint(Point2f p1,Point2f p2,Point2f p3);
};
#endif