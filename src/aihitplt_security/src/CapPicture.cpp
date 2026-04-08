/*
* Copyright(C) 2010,Hikvision Digital Technology Co., Ltd 
* 
* File   name��CapPicture.cpp
* Discription��
* Version    ��1.0
* Author     ��panyd
* Create Date��2010_3_25
* Modification History��
*/

#include "public.h"
#include "CapPicture.h"
#include <stdio.h>
#include <string.h>
/*******************************************************************
      Function:   Demo_Capture
   Description:   Capture picture.
     Parameter:   (IN)   none 
        Return:   0--success��-1--fail.   
**********************************************************************/
int Demo_Capture()
{
    NET_DVR_Init();
    long lUserID;
    //login
    NET_DVR_USER_LOGIN_INFO struLoginInfo = {0};
    NET_DVR_DEVICEINFO_V40 struDeviceInfoV40 = {0};
    struLoginInfo.bUseAsynLogin = false;

    struLoginInfo.wPort =80;
    memcpy(struLoginInfo.sDeviceAddress, "192.168.5.64", NET_DVR_DEV_ADDRESS_MAX_LEN);
    memcpy(struLoginInfo.sUserName, "admin", NAME_LEN);
    memcpy(struLoginInfo.sPassword, "abcd1234", NAME_LEN);

    lUserID = NET_DVR_Login_V40(&struLoginInfo, &struDeviceInfoV40);
    static int num = 0;
    char  name[]= "./img/image_";
    char  jpeg[]=".jpeg";	
 
    char dir[50]={0};
    num += 1;
    sprintf(dir,"%s%d%s",name,num,jpeg);	
	
    if (lUserID < 0)
    {
        printf("pyd1---Login error, %d\n", NET_DVR_GetLastError());
        return HPR_ERROR;
    }

    //
    NET_DVR_JPEGPARA strPicPara = {0};
    strPicPara.wPicQuality = 2;
    strPicPara.wPicSize = 5;
    int iRet;
    iRet = NET_DVR_CaptureJPEGPicture(lUserID, 1, &strPicPara, dir);//struDeviceInfoV40.struDeviceV30.byStartChan
    if (!iRet)
    {
        printf("pyd1---NET_DVR_CaptureJPEGPicture error, %d\n", NET_DVR_GetLastError());
        return HPR_ERROR;
    }

    //logout
    NET_DVR_Logout_V30(lUserID);
    NET_DVR_Cleanup();

    return HPR_OK;

}
