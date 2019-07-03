#include <iostream>
#include <sys/time.h>
#include <string.h>
#include <string>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include"../header/config.h"
#include"../header/ArmorDetector.h"
#include"../header/Serialport.h"
#include"../header/SivirDetector.h"
#include"../header/SivirhomeDetector.h"
#include"../header/watchdog.h"
#include"../header/GxIAPI.h"
#include"../header/DxImageProc.h"
#define PI 3.14159265
using namespace cv;
using namespace std;

int mode=1;
int findmode=1;
VideoCapture cap;//打开小相机
int getcaprsc=0;//0代表装甲识别在使用，1代表能量机关在使用
ArmorDetector detector;//装甲识别类
Mat srcframe;

Mat m_image;
bool      is_implemented = false;
int64_t   m_pixel_color = 0;              ///< Bayer格式
char*m_rgb_image = NULL;

#define MEMORY_ALLOT_ERROR -1 

GX_DEV_HANDLE g_device = NULL;              ///< 设备句柄
GX_FRAME_DATA g_frame_data = { 0 };         ///< 采集图像参数
pthread_t g_acquire_thread = 0;             ///< 采集线程ID
bool g_get_image = false;                   ///< 采集线程是否结束的标志：true 运行；false 退出

int64_t m_roi_offset_x = 320;		///< 水平偏移量设置 1280-640 
int64_t m_roi_offset_y = 277;		///< 竖直偏移量设置 1024-480
int64_t m_roi_width = 0;		///< 感兴趣区域宽			
int64_t m_roi_height = 0;		///< 感兴趣区域高
bool b_is_set_roi = false;		///< 是否设置roi标志

WatchDog dog("/home/ubuntu1/Desktop/RM/YY/watchdog_fifo");

//获取图像大小并申请图像数据空间
int PreForImage();

//释放资源
int UnPreForImage();

//采集线程函数
void *ProcGetImage(void* param);

//获取错误信息描述
void GetErrorString(GX_STATUS error_status);


void setCap()
{
    cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,960);
}
int main()
{
    //串口通信
    Serialport ser("/dev/ttyUSB0"); 

    ser.set_opt(115200,8,'N',1);
    //看门狗
    
    dog.feed();

    //工业相机
    //----------------------------------------------------------------
    uid_t user = 0;
    user = geteuid();
    if(user != 0)
    {
        printf("\n");  
        printf("Please run this application with 'sudo -E ./GxAcquireContinuous' or"
                              " Start with root !\n");
        printf("\n");
        return 0;
    }
    printf("Press [x] or [X] and then press [Enter] to Exit the Program\n");
    printf("Initializing......"); 
    printf("\n\n");

    // usleep(2000000);

    //API接口函数返回值 
    GX_STATUS status = GX_STATUS_SUCCESS;

    uint32_t device_num = 0;
    uint32_t ret = 0;
    GX_OPEN_PARAM open_param;

    //初始化设备打开参数，默认打开序号为1的设备
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    open_param.openMode = GX_OPEN_INDEX;
    open_param.pszContent = "1";

    //初始化库
    status = GXInitLib(); 
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return 0;
    }

    //获取枚举设备个数
    status = GXUpdateDeviceList(&device_num, 1000);
    if(status != GX_STATUS_SUCCESS)
    { 
        GetErrorString(status);
        status = GXCloseLib();
        return 0;
    }

    if(device_num <= 0)
    {
        printf("<No device>\n");
        status = GXCloseLib();
        return 0;
    }
    else
    {
        //默认打开第1个设备
        status = GXOpenDevice(&open_param, &g_device);
        if(status == GX_STATUS_SUCCESS)
	{
            printf("<Open device success>\n");
	    int64_t width,height;
        b_is_set_roi = true;
	    //设置roi区域，设置时相机必须时停采状态
	    if(b_is_set_roi)
		{
		    m_roi_width = 640;
		    m_roi_height = 480;
		    m_roi_offset_x = 320;
		    m_roi_offset_y = 320;
		    status = GXSetInt(g_device,GX_INT_WIDTH,m_roi_width);
		    status = GXSetInt(g_device,GX_INT_HEIGHT,m_roi_height);
		    status = GXSetInt(g_device,GX_INT_OFFSET_X,m_roi_offset_x);
		    status = GXSetInt(g_device,GX_INT_OFFSET_Y,m_roi_offset_y);
		}
    	    status = GXGetInt(g_device,GX_INT_WIDTH,&width);
    	    status = GXGetInt(g_device,GX_INT_HEIGHT,&height);
	    // 查询当前相机是否支持GX_ENUM_PIXEL_COLOR_FILTER
    	    status = GXIsImplemented(g_device, GX_ENUM_PIXEL_COLOR_FILTER, &is_implemented);

   	   //支持彩色图像
    	   if(is_implemented)
    	   {
       		status = GXGetEnum(g_device, GX_ENUM_PIXEL_COLOR_FILTER, &m_pixel_color);
		m_image.create(height,width,CV_8UC3);
		m_rgb_image = new char[width*height*3];
   	   }else{
	    
 	    	m_image.create(height,width,CV_8UC1);
	   }
        }
        else
        {
            printf("<Open device fail>\n");
            status = GXCloseLib();
            return 0;			
        }
    }

    //设置采集模式为连续采集
    status = GXSetEnum(g_device, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //设置触发开关为OFF
    status = GXSetEnum(g_device, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }
	
    //为采集做准备    
    ret = PreForImage();
    if(ret != 0)    
    {
        printf("<Failed to prepare for acquire image>\n");
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //启动接收线程
    ret = pthread_create(&g_acquire_thread, 0, ProcGetImage, 0);
    if(ret != 0)
    {
        printf("<Failed to create the collection thread>\n");
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }
    dog.feed();
    while (srcframe.empty())
    {
        cout<<"ready for get frame..."<<endl;
    }
  
    int cntframe=0;
    Mat frame;
    SivirDetector Sivir;
   // SivirhomeDetector Sivir2;
  //  Sivir2.islost=true;
    cout<<"now mode:"<<mode<<endl;
    dog.feed();
    bool run=true;
    
    int cntSivir=0;
    // videorate值有待商榷
    double videoRate = 120;
    Size videoSize(640,480);
    stringstream ss;
    string filename="";
    time_t nowtime;
    nowtime = time(NULL);
    //cap.open("/dev/video0");
    //srand((int)time(0));
    //ss << rand()%1000;
    ss << nowtime;
    filename+=ss.str();
    //VideoWriter videoWrtL(filename+".avi",CV_FOURCC('M', 'J', 'P', 'G'), videoRate, videoSize);
    //VideoWriter videoWrtLYY(filename+"YY.avi",CV_FOURCC('M', 'J', 'P', 'G'), videoRate, videoSize);

    while (run)
    { 
        double now=clock();
        double getImgtime=clock();
        
        dog.feed();

        cout<<"------------------Frame:"<<++cntframe<<"------------------"<<endl;
        cout<<"get image time:="<<(getImgtime-now)/CLOCKS_PER_SEC * 1000<<"ms"<<endl;
        ser.readMode(mode);
        cout<<"mode:"<<mode<<endl;
        if (mode == 1)
        {
            detector.isbig = false;
        }
        if (mode == 2)
        {
            detector.isbig = true;
        }
        if (findmode==ARMOR_MODE)
        {
            if (getcaprsc==1)//相机切换
            {
                getcaprsc=0;
               // cap.release();
                cout<<"now mode:"<<mode<<endl;
            }
            frame=srcframe.clone();
            //videoWrtL << frame;
            detector.getResult(frame);
            circle(frame,detector.target.center,8,Scalar(255,0,0),2,8,0);
            //videoWrtLYY << frame;

            dog.feed();

            if (detector.islost==false) cout<<"find the armor successfully!"<<endl;
            float yaw,pitch,dist;
            yaw=detector.pnpresult.yaw;
            pitch=detector.pnpresult.pitch;
            dist=detector.pnpresult.dist/1000;
            cout<<"yaw:"<<yaw<<"  pitch:"<<pitch<<" dist:"<<dist<<endl;
            float flag;
            if (detector.islost == true) flag=0.0;
            else flag=1.0;
            ser.sendAngleDist(yaw,pitch,dist,flag);
            cout<<"flag:"<<flag<<endl;
            double afterfind=clock();
            cout<<"find armor time:="<<(afterfind-now)/CLOCKS_PER_SEC * 1000<<"ms"<<endl;
        }
        else if (findmode==2)
        {
            if (getcaprsc==0)//相机切换
            {
                getcaprsc=1;
                cout<<"now mode:"<<mode<<endl;
                cap.open("/dev/video0");
		        waitKey(200);
                //cap.open("../../Sivir2.avi");
            }
            cap >> frame;

            //frame=srcframe.clone();
            resize(frame,frame,Size(640,480));
            //videoWrtL << frame;
            Sivir.getResult(frame);
            circle(frame,Sivir.target.center,8,Scalar(255,0,0),2,8,0); 
            //videoWrtLYY << frame;         
            dog.feed();

            if (Sivir.islost==false) cout<<"find the armor successfully!"<<endl;
            float yaw,pitch;
            yaw=Sivir.pnpresult.yaw;
            pitch=Sivir.pnpresult.pitch;
            cout<<"yaw:"<<yaw<<"  pitch:"<<pitch<<endl;
            ser.sendAngle(yaw,pitch);
            double afterfind=clock();
            cout<<"find sivir time:="<<(afterfind-now)/CLOCKS_PER_SEC * 1000<<"ms"<<endl;
        }
    /*    else if (findmode==2)
        {
            if (getcaprsc==0)//相机切换
            {
                getcaprsc=1;
                cout<<"now mode:"<<mode<<endl;
                //dog.feed();
                
                //cap.open("../../Sivir.avi");
		        waitKey(20);
            }
            dog.feed();
            cap >> frame;
            Sivir2.getResult(frame);
            float yaw,pitch;
            yaw=Sivir2.pnpresult.yaw;
            pitch=Sivir2.pnpresult.pitch;
            cout<<"yaw:"<<yaw<<"  pitch:"<<pitch<<endl;
            
            ser.sendAngle(yaw,pitch);
            double afterfind=clock();
            cout<<"find sivir time:="<<(afterfind-now)/CLOCKS_PER_SEC * 1000<<"ms"<<endl;
        }*/
        waitKey(1);
    }

    //为停止采集做准备
    ret = UnPreForImage();
    if(ret != 0)
    {
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //关闭设备
    status = GXCloseDevice(g_device);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //释放库
    status = GXCloseLib();
    return 0;
}


//-------------------------------------------------
/**
\brief 获取图像大小并申请图像数据空间
\return void
*/
//-------------------------------------------------
int PreForImage()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    int64_t payload_size = 0;
	
    status = GXGetInt(g_device, GX_INT_PAYLOAD_SIZE, &payload_size);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return status;
    }
	
    g_frame_data.pImgBuf = malloc(payload_size);
    if(g_frame_data.pImgBuf == NULL)
    {
        printf("<Failed to allot memory>\n");
        return MEMORY_ALLOT_ERROR;
    }
 
    return 0;
}

//-------------------------------------------------
/**
\brief 释放资源
\return void
*/
//-------------------------------------------------
int UnPreForImage()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t ret = 0;
   
    //发送停采命令
    status = GXSendCommand(g_device, GX_COMMAND_ACQUISITION_STOP);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return status;
    }

    g_get_image = false;
    ret = pthread_join(g_acquire_thread,NULL);
    if(ret != 0)
    {
        printf("<Failed to release resources>\n");
        return ret;
    }
	

    //释放buffer
    if(g_frame_data.pImgBuf != NULL)
    {
        free(g_frame_data.pImgBuf);
        g_frame_data.pImgBuf = NULL;
    }

    return 0;
}

//-------------------------------------------------
/**
\brief 采集线程函数
\param pParam 线程传入参数
\return void*
*/
//-------------------------------------------------
void *ProcGetImage(void* pParam)
{
    GX_STATUS status = GX_STATUS_SUCCESS;

    //接收线程启动标志
    g_get_image = true;

    //发送开采命令
    status = GXSendCommand(g_device, GX_COMMAND_ACQUISITION_START);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
    }
	
    while(g_get_image)
    {
        if(g_frame_data.pImgBuf == NULL)
        {
            continue;
        }

        status = GXGetImage(g_device, &g_frame_data, 100);
        if(status == GX_STATUS_SUCCESS)
        {
            if(g_frame_data.nStatus == 0)
            {
                //printf("<Successful acquisition : Width: %d Height: %d >\n", g_frame_data.nWidth, g_frame_data.nHeight);
		        if(is_implemented)
    	   	    {
                    DxRaw8toRGB24(g_frame_data.pImgBuf, m_rgb_image, g_frame_data.nWidth, g_frame_data.nHeight,RAW2RGB_NEIGHBOUR,DX_PIXEL_COLOR_FILTER(BAYERBG),false);
       			    memcpy(m_image.data,m_rgb_image,g_frame_data.nHeight*g_frame_data.nWidth*3);
   	   	        }
                else
                {
 	    		    memcpy(m_image.data,g_frame_data.pImgBuf,g_frame_data.nHeight*g_frame_data.nWidth);
	   	        }
                // cout<<"test m_image"<<endl;
           	   // imshow("test", m_image);
                //detector.getResult(m_image);
                srcframe = m_image.clone();
               // resize(frame,frame,Size(XSIZE,YSIZE));
               // imshow("test",frame);
               // waitKey(3);
            }
        }
    }
}

//----------------------------------------------------------------------------------
/**
\brief  获取错误信息描述
\param  emErrorStatus  错误码

\return void
*/
//----------------------------------------------------------------------------------
void GetErrorString(GX_STATUS error_status)
{
    char *error_info = NULL;
    size_t size = 0;
    GX_STATUS status = GX_STATUS_SUCCESS;
	
    // 获取错误描述信息长度
    status = GXGetLastError(&error_status, NULL, &size);
    if(status != GX_STATUS_SUCCESS)
    {
           GetErrorString(status);
	   return;
    }
	
    error_info = new char[size];
    if (error_info == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return ;
    }
	
    // 获取错误信息描述
    status = GXGetLastError(&error_status, error_info, &size);
    if (status != GX_STATUS_SUCCESS)
    {
        printf("<GXGetLastError call fail>\n");
    }
    else
    {
        printf("%s\n", (char*)error_info);
    }

    // 释放资源
    if (error_info != NULL)
    {
        delete []error_info;
        error_info = NULL;
    }
}

