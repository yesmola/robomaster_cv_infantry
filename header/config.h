#ifndef _CONFIG_H_
#define _CONFIG_H_

#define XSIZE 640
#define YSIZE 480
#define BIN_VALUE 50//110
#define AREA1 1500//200   //大装甲板与其他
#define AREA2 3500//450   //大装甲板和下面的大灯条
#define MIND 130//40     //对应的大的和小的距离应当小于该值，其他都大于
#define ARMOR_MODE  1
#define SIVIR_MODE  2

#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Console {
public:
    int ArmorMode;
};
#endif