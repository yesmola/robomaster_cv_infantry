#ifndef _ARMORDETECTOR_H_
#define _ARMORDETECTOR_H_
#include<opencv2/opencv.hpp>
#include<vector>
#include"config.h"

using namespace cv;
using namespace std;

typedef struct ARMOR
{
    Point2f center;
    Point2f rect[4];
}Armor;

typedef struct ROI
{
    Point2f lefttop;
    int rwidth;
    int rheight;
}Roi;

typedef struct ANGLE
{
    float yaw;
    float pitch;
    float dist;
}Angle;

class ArmorDetector
{
public:
    Mat src;//source image
    Mat binary;//binary image
    Mat outline;//outline image
    bool islost;//1代表丢失
    Armor target;
    Roi roi;
    Mat roiimg;
    Angle pnpresult;
    vector< vector<Point> > contours;
    int num;
    float matchrank[1500][1500];
    //float dis[1500][1500];
    bool isbig;

public: 
    ArmorDetector();
    ArmorDetector(Mat src0);
    void getResult(Mat src0);
    void getSrcImage(Mat src0);
    void getBinaryImage();
    void getContours();
    void getTarget();
    void getPnP();
};
#endif