#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp> 
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <fstream>
#include <unistd.h>
#include <pthread.h> 
#include <iostream>
#include "HCNetSDK.h"
#include "PlayM4.h"
#include "LinuxPlayM4.h"
#include "CapPicture.h"
#include "playback.h"

#include <termios.h> 
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <pthread.h>

#define HPR_ERROR       -1
#define HPR_OK           0
#define USECOLOR         0
#define key_ESC 27

using namespace std;

static cv::Mat dst;
cv::Mat image;
cv::Mat dst_img;
cv::Mat imgResize;

string cam_ip;    //用于获取launch传递函数
string user_name;    //用于获取launch传递函数
string password;    //用于获取launch传递函数
string user_log;    //用于获取launch传递函数
string record_video;    //用于获取launch传递函数
string record_dat;    //用于获取launch传递函数
string image_name;
string image_type;

int Init_runing = 1;
int Camera_runing = 1;
int video_recording = 0; 

char key = 0; // 云台控制
int key_runing = 1;
static int num = 0;
static struct termios initial_settings, new_settings;
static int peek_character = -1;         /* 用于测试一个按键是否被按下 */
static int peek_char = -1;         /* 用于测试一个按键是否被按下 */

string dir;
FILE *g_pFile = NULL;
HWND h = NULL;
LONG nPort=-1;
LONG lUserID;
 
pthread_mutex_t mutex_cam;
std::list<cv::Mat> g_frameList;

image_transport::Publisher pub; //  发布图像话题
//　发布图像数据
sensor_msgs::ImagePtr msg;
ros::Subscriber Camera_Control_Sub; 

void init_keyboard();
void close_keyboard();
int kbhit();
int readch(); /* 相关函数声明 */
// 读取视频流回调函数
static void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize,void* dwUser);
// 预览重连回调函数
static void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser); 

/*
功能：写入头数据
备注：云台、API
*/
void CALLBACK PsDataCallBack(LONG lRealHandle, DWORD dwDataType,BYTE *pPacketBuffer,DWORD nPacketSize, char* pUser)
{  
  char * DAT_data = (char *)record_dat.data();
  if (dwDataType  == NET_DVR_SYSHEAD)
  {
    //写入头数据
    g_pFile = fopen(DAT_data, "wb");
    if (g_pFile == NULL)
    {
      printf("CreateFileHead fail\n");
      return;
    }
    //写入头数据
    fwrite(pPacketBuffer, sizeof(unsigned char), nPacketSize, g_pFile);
    printf("write head len=%d\n", nPacketSize);
  }
  else
  {
    if(g_pFile != NULL)
    {
      fwrite(pPacketBuffer, sizeof(unsigned char), nPacketSize, g_pFile);
      printf("write data len=%d\n", nPacketSize);
    }
  }
}
 
/*
功能：格式转换
备注：云台、API
*/
//void CALLBACK DecCBFun(LONG nPort, char *pBuf, LONG nSize, FRAME_INFO *pFrameInfo, LONG nReserved1, LONG nReserved2)
void CALLBACK DecCBFun(LONG nPort, char *pBuf, LONG nSize, FRAME_INFO *pFrameInfo, void* nReserved1, LONG nReserved2)
{
  long lFrameType = pFrameInfo->nType;
  if (lFrameType == T_YV12)
  {
    dst.create(pFrameInfo->nHeight, pFrameInfo->nWidth,CV_8UC3);
    // printf("hheihg: %d wwidht: %d size: %d\n",pFrameInfo->nHeight,  pFrameInfo->nWidth, nSize );
    cv::Mat src(pFrameInfo->nHeight + pFrameInfo->nHeight/2, pFrameInfo->nWidth, CV_8UC1, (uchar *)pBuf);
	  cv::cvtColor(src, dst, CV_YUV2BGR_YV12);
    pthread_mutex_lock(&mutex_cam);
    g_frameList.push_back(dst);
    pthread_mutex_unlock(&mutex_cam);
  }
  usleep(10);
}
 
/*
功能：读取视频流回调
备注：云台、API
*/
void CALLBACK g_RealDataCallBack_V30(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize,void* dwUser)
{
  DWORD dRet;
  switch (dwDataType)
  {
    case NET_DVR_SYSHEAD:           //系统头
    if (!PlayM4_GetPort(&nPort))  //获取播放库未使用的通道号
    {
      break;
    }
    //第一次回调的是系统头，将获取的播放库port号赋值给全局port，下次回调数据时即使用此port号播放
    if (dwBufSize > 0) 
    {
      //设置实时流播放模式
      if (!PlayM4_SetStreamOpenMode(nPort, STREAME_REALTIME)) 
      {
        dRet = PlayM4_GetLastError(nPort);
        break;
      }
      //打开流接口
      if (!PlayM4_OpenStream(nPort, pBuffer, dwBufSize, 1024 * 1024))
      {
        dRet = PlayM4_GetLastError(nPort);
        break;
      }
      //设置解码回调函数 解码且显示
      if (!PlayM4_SetDecCallBackEx(nPort, DecCBFun, NULL, NULL))
      {
        dRet = PlayM4_GetLastError(nPort);
        break;
      }
      //打开视频解码，播放开始
      if (!PlayM4_Play(nPort, h))
      {
        dRet = PlayM4_GetLastError(nPort);
        break;
      }
      //打开音频解码, 需要码流是复合流
      if (!PlayM4_PlaySound(nPort)) 
      {
        dRet = PlayM4_GetLastError(nPort);
        break;
      }
    }
    break;
    //usleep(500);
    case NET_DVR_STREAMDATA:  //码流数据
       if (dwBufSize > 0 && nPort != -1) 
       {
         BOOL inData = PlayM4_InputData(nPort, pBuffer, dwBufSize);
         while (!inData) 
         {
           sleep(10);
           inData = PlayM4_InputData(nPort, pBuffer, dwBufSize);
           std::cerr << "PlayM4_InputData failed \n" << std::endl;
         }
       }
       break;
   }
}
 
/*
功能：预览时重连回调
备注：无
*/ 
void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser)
{
  char tempbuf[256] = {0};
  std::cout << "EXCEPTION_RECONNECT = " << EXCEPTION_RECONNECT << std::endl;
  switch(dwType)
  {
    case EXCEPTION_RECONNECT:	//预览时重连
                              printf("pyd reconnect %d\n", time(NULL));
                              break;
    default:
                              break;
  }
}

/****************************************************************
函数功能：初始化SDK，用户注册设备
****************************************************************/
void *CameraInit(void *)
{
	char * LOG   = (char *)user_log.data(); //离线语法识别资源路径
	char * RECORD   = (char *)record_video.data(); //离线语法识别资源路径
  char *IP     = (char *)cam_ip.data();   //海康威视网络摄像头的ip
  char *UName  = (char *)user_name.data();     //海康威视网络摄像头的用户名
  char *PSW    = (char *)password.data();      //海康威视网络摄像头的密码
  //  将自己设置为分离状态
  pthread_detach(pthread_self());  

  NET_DVR_Init(); //  初始化SDK
  //  设置连接时间与重连时间
  NET_DVR_SetConnectTime(2000, 1);
  NET_DVR_SetReconnect(1000, true);
  //  设置日志保存路径
  NET_DVR_SetLogToFile(3, LOG);

  NET_DVR_DEVICEINFO_V30 struDeviceInfo = {0};
  NET_DVR_SetRecvTimeOut(5000);
  lUserID = NET_DVR_Login_V30(IP, 8000, UName, PSW, &struDeviceInfo); //  用户注册设备
  if (lUserID < 0)
  {
    printf("###########<<Login error>>########## erro code: %d\n", NET_DVR_GetLastError());
    NET_DVR_Cleanup();
  }

  NET_DVR_SetExceptionCallBack_V30(0, NULL, g_ExceptionCallBack, NULL);
 
  long lRealPlayHandle;
  NET_DVR_CLIENTINFO ClientInfo = {0};
 
  ClientInfo.lChannel       = 1;  //预览通道号
  ClientInfo.lLinkMode     = 0;   //0-TCP方式，1-UDP方式，2-多播方式，3-RTP方式，4-RTP/RTSP，5-RSTP/HTTP
  ClientInfo.hPlayWnd     = 0;    //需要SDK解码时句柄设为有效值，仅取流不解码时可设为空
  ClientInfo.sMultiCastIP = NULL;
 
  // 启动预览
  lRealPlayHandle = NET_DVR_RealPlay_V30(lUserID, &ClientInfo, g_RealDataCallBack_V30, NULL, 0);
  if (video_recording)
  {
	  NET_DVR_SaveRealData(lRealPlayHandle, RECORD);
  }

  if (lRealPlayHandle < 0)
  {
    printf("pyd1---NET_DVR_RealPlay_V30 error\n");
  }
  while(Init_runing)
  {
    int a;
  }
  // sleep(-1);  //sleep 无限时间
  // 释放SDK资源
  NET_DVR_Cleanup();
  pthread_exit(NULL);
}
 
/*
功能：相机图像发布线程
备注：相机图像
*/ 
void *RunIPCameraInfo(void*)
{
  // 将自己设置为分离状态
  pthread_detach(pthread_self());   

  while(Camera_runing)
  {
    pthread_mutex_lock(&mutex_cam);
    if(g_frameList.size())
    {
      std::list<cv::Mat>::iterator it;
      it = g_frameList.end();
      it--;
      image = (*(it));
      if (!image.empty())
      {
        //  修改图片尺寸大小（像素为640*480）
        cv::resize(image, dst_img,  cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
        imshow("image", dst_img);  

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst_img).toImageMsg();
        if(msg)
        {
          pub.publish(msg); 
        }
        cv::waitKey(1);
      }
      g_frameList.pop_front();
    }
    g_frameList.clear(); // 丢掉旧的帧
    pthread_mutex_unlock(&mutex_cam); 
  }
	pthread_exit(NULL);
}

/*
功能：键盘输入获取线程
备注：键盘
*/ 
void *capture_keyvalue(void*)
{
  // 将自己设置为分离状态
  pthread_detach(pthread_self());  
  while(key_runing)
  {
    if (kbhit())
    {
      key = readch(); 
      // printf("test ch %d \n", key);
	    if(key == 3)
	    {
        close_keyboard();
        key_runing = 0;
        Camera_runing = 0;  
  	  }
    }
  }	
  close_keyboard();	
  pthread_exit(NULL);
}

/*
功能：判断是否有键盘按下
备注：键盘
*/ 
int kbhit()
{
  char ch = NULL;
  int nread = 0;
  if ( peek_character != -1 )
  {
    peek_character = -1;
    return(1);
  }
    
  new_settings.c_cc[VMIN] = 0;
  tcsetattr(0, TCSANOW, &new_settings);
  nread = read(0, &ch, 1);
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(0, TCSANOW, &new_settings);
  if ( nread == 1 )
  {
    peek_character = ch;
    // printf("test nread %d \n", nread);
    return(1);
  }
  return(0);
}

/*
功能：捕获键盘输入
备注：键盘
*/
/* 用来接收按下的按键，并peek_character = -1恢复状态 */
int readch()
{
  char ch = NULL;
  if ( peek_character != -1 )
  {
    ch = peek_character;
    peek_character = -1;
    // printf("test ch %d \n", ch);
    return(ch);
  }
  read( 0, &ch, 1 );
  return(ch);
}

/*
功能：终端配置
备注：键盘
*/
void init_keyboard()
{
  tcgetattr( 0, &initial_settings );
  new_settings = initial_settings;
  new_settings.c_lflag &= ~ICANON;
  new_settings.c_lflag &= ~ECHO;
  new_settings.c_lflag &= ~ISIG;
  new_settings.c_cc[VMIN] = 1;
  new_settings.c_cc[VTIME] = 0;
  tcsetattr( 0, TCSANOW, &new_settings );
}

/*
功能：关闭键盘
备注：键盘
*/
void close_keyboard()
{
  tcsetattr( 0, TCSANOW, &initial_settings );
}

/*
功能：话题控制
备注：话题回调
*/
void Camera_Control_Callback(const std_msgs::String::ConstPtr& cmd)
{
  string temp = cmd -> data;
  // cout << temp[0] << endl;
  key = temp[0];
}


int main(int argc,char **argv)
{
  ros::init(argc, argv, "Pan_Tilt_Camera_NODE");
  ros::NodeHandle n;		//创建句柄	
  ros::NodeHandle nh("~");    //用于launch文件传递参数

	nh.param("cam_ip", cam_ip, std::string("192.168.2.64"));    //从launch文件获取appid参数
	nh.param("user_name", user_name, std::string("admin"));    //从launch文件获取参数
	nh.param("password", password, std::string("abcd1234"));    //从launch文件获取参数
	nh.param("user_log", user_log, std::string("./record/sdkLog"));    //从launch文件获取参数
	nh.param("record_video", record_video, std::string("./yuntai.mp4"));    //从launch文件获取参数
	nh.param("record_dat", record_dat, std::string("./record/ps.dat"));    //从launch文件获取参数

	nh.param("image_name", image_name, std::string("./img/image_"));    //从launch文件获取参数
	nh.param("image_type", image_type, std::string(".jpeg"));    //从launch文件获取参数

  Camera_Control_Sub = n.subscribe("/Camera_Control_TOPIC", 1, &Camera_Control_Callback);	 
  image_transport::ImageTransport it(n);
  pub = it.advertise("camera/image", 10);
  pthread_t camerainit;
  pthread_t getframe; 
  pthread_t input_key;
	
  // char  name[]= "./img/image_";
  // char  jpeg[]=".jpeg";	
  char * name = (char *)image_name.data();
  char * jpeg = (char *)image_type.data();

  int ret;
  init_keyboard();

  pthread_mutex_init(&mutex_cam, NULL); 
  ret = pthread_create(&camerainit, NULL, CameraInit, NULL);  //  创建摄像头初始化线程
  if(ret!=0)
  {
    printf("Create pthread error!\n");
  }
  
  ret = pthread_create(&camerainit, NULL, RunIPCameraInfo, NULL); //  创建图像显示与发布线程
  if(ret!=0)
  {
    printf("Create pthread error!\n");
  }

  ret = pthread_create(&input_key, NULL, capture_keyvalue, NULL); //  创建键盘获取线程
  if(ret!=0)
  {
    printf("Create pthread error!\n");
  }  

  bool sw = true;
  
  
  //ros::Rate loop_rate(100);	
  while(ros::ok())
  {
    switch(key)
    {
      // 焦距变大(倍率变大)
      case '1': 	NET_DVR_PTZControl_Other(lUserID, 1, ZOOM_IN, 0);	
                  usleep(10);
                  key = 117;
                  // NET_DVR_PTZControl_Other(lUserID, 1, ZOOM_IN, 1);	
                  // printf(" wait %s\n", key);
                  break;
      case 'u': 	NET_DVR_PTZControl_Other(lUserID, 1, ZOOM_IN, 1);	
                  break;

      // 焦距变小(倍率变小)
      case '2':   NET_DVR_PTZControl_Other(lUserID, 1, ZOOM_OUT, 0); 
                  usleep(10);
                  key = 105; 	
			            break;  
      case 'i':   NET_DVR_PTZControl_Other(lUserID, 1, ZOOM_OUT, 1);  	
			            break;  

      // 接通灯光电源
      case '3':   
                  NET_DVR_PTZControl_Other(lUserID, 1, LIGHT_PWRON, 0);
			            break;
      //  焦点前调
      case '4':   
                  NET_DVR_PTZControl_Other(lUserID, 1, FOCUS_NEAR, 0); 
                  usleep(10);
                  key = 111;  
		              break;
      case 'o':   
                  NET_DVR_PTZControl_Other(lUserID, 1, FOCUS_NEAR, 1);  
		              break;
      //  焦点后调
      case '5':   
                  NET_DVR_PTZControl_Other(lUserID, 1, FOCUS_FAR, 0);
                  usleep(10);
                  key = 112;  
		              break;
      case 'p':   
                  NET_DVR_PTZControl_Other(lUserID, 1, FOCUS_FAR, 1);  
		              break;

      //  光圈扩大
      case '6':   
                  NET_DVR_PTZControl_Other(lUserID, 1, IRIS_OPEN, 0); 
                  usleep(10);
                  key = 106;  
		              break;
      case 'j':   
                  NET_DVR_PTZControl_Other(lUserID, 1, IRIS_OPEN, 1);  
		              break;

      //  光圈缩小
      case '7':   
                  NET_DVR_PTZControl_Other(lUserID, 1, IRIS_CLOSE, 0);
                  usleep(10); 
                  key = 108; 
		              break;
      case 'l':   
                  NET_DVR_PTZControl_Other(lUserID, 1, IRIS_CLOSE, 1); 
		              break;

      //接通雨刷开关
      case '8':   
                  NET_DVR_PTZControl_Other(lUserID, 1, WIPER_PWRON, 0); 
                  // usleep(10);
                  // key = 107; 
		              break;

      //关闭雨刷开关
      case '9':   
                  NET_DVR_PTZControl_Other(lUserID, 1, WIPER_PWRON, 1); 
                  // usleep(10);
                  // key = 107; 
		              break;

      //云台上仰
      case 'w':   
                  NET_DVR_PTZControlWithSpeed_Other(lUserID, 1, TILT_UP, 0, 7); 
                  usleep(10);
                  key = 107; 
			            break;
      //云台下俯
      case 's':   
                  NET_DVR_PTZControlWithSpeed_Other(lUserID, 1, TILT_DOWN, 0, 7);
                  usleep(10);
                  key = 98; 
			            break;

      // 云台左转
      case 'a':   
                  NET_DVR_PTZControlWithSpeed_Other(lUserID, 1, PAN_LEFT, 0, 7); 
                  usleep(10);
                  key = 110; 
			            break;
      // 云台右转
      case 'd':   
                  NET_DVR_PTZControlWithSpeed_Other(lUserID, 1, PAN_RIGHT, 0, 7);
                  usleep(10);
                  key = 109; 
			            break;

      case 'f':  
                  if(!image.empty())
                  {
                    num += 1;
                    dir = image_name + std::to_string(num) + image_type;
                    // sprintf(dir,"%s%d%s",name,num,jpeg);
                    cv::imwrite(dir, image);
                    printf("save current image!");
                  }
                  key = 0;
			            break;

      case 'g':  Demo_PlayBack(lUserID);
			          break;
	  
	    case 'v':   video_recording = !video_recording;		//录制视频
			            if(video_recording)	printf(" start video recording\n");
			            else 			printf(" stop video recording\n");
                  // NET_DVR_Cleanup();
			            break;

      // 停止云台上仰
      case 'k':   
                  NET_DVR_PTZControlWithSpeed_Other(lUserID, 1, TILT_UP, 1, 3); 
			            break;  

      // 停止云台下俯
      case 'b':   
                  NET_DVR_PTZControlWithSpeed_Other(lUserID, 1, TILT_DOWN, 1, 3);
			            break; 
      // 停止云台左转
      case 'n':   
                  NET_DVR_PTZControlWithSpeed_Other(lUserID, 1, PAN_LEFT, 1, 3);
			            break; 
      // 停止云台右转
      case 'm':   
                  NET_DVR_PTZControlWithSpeed_Other(lUserID, 1, PAN_RIGHT, 1, 3);
			            break; 
	    case  3:    goto exit;break;
	    default : 	
                  // printf(" wait\n");
                  // key = !key;
                  break;

	  }	
	  ros::spinOnce();//循环等待回调函数
  }

  close_keyboard();	
  Init_runing = 0;
  key_runing = 0;
  Camera_runing = 0; 
  pthread_exit(NULL);
  return 0;
 
exit:
  close_keyboard();	
  Init_runing = 0;
  key_runing = 0;
  Camera_runing = 0; 
  pthread_exit(NULL);
  return 0;
}
