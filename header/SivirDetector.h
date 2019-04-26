#ifndef _SIVIRDETECTOR_H_
#define _SIVIRDETECTOR_H_
#include<opencv2/opencv.hpp>
#include<vector>
#include<math.h>
#include"config.h"

using namespace cv;
using namespace std;

typedef struct ARMOR2
{
    Point2f center;
    Point2f rect[4];
}Armor2;

typedef struct ANGLE2
{
    float yaw;
    float pitch;
}Angle2;

class SivirDetector
{
public:
    Mat src;//source image
    Mat binary;//binary image
    Mat outline;//outline image
    bool islost;
    vector< vector<Point> > contours;
    int num;
    Point2f small[100];
    Point2f big[100];
    int cntsmall;
    int cntbig;
    Armor2 target;
    Angle2 pnpresult;
    Point2f allrect[100][4];
    
public: 
    SivirDetector();
    SivirDetector(Mat src0);
    void getResult(Mat src0);
    void getSrcImage(Mat src0);
    void getBinaryImage();
    void getContours();
    void getTarget();
    void getPnP();
};
#endif