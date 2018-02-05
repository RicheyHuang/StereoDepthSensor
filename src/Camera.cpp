#include "Camera.h"
#include <string.h>
using namespace cv;

Camera::Camera()
{
    // this->iCameraCounts = CamNum;
    if (CameraSdkInit(1) != CAMERA_STATUS_SUCCESS){
        printf("fail to CameraSdkInit\n");
    }
    this->iCameraCounts = 12;

    for(int i=0; i<12; ++i)
    {
        this->g_pRgbBuffer[i] = NULL;
        // this->thread_t[i] = NULL;
    }

    // printf("Camera Initialize Done\n");
}


bool Camera::EnumerateDevice() //枚举设备，并建立设备列表
{
    int flag = -1;
    // this->iCameraCounts = 1;
    // std::cout<<"EnumerateDevice"<<std::endl;
    flag = CameraEnumerateDevice(tCameraEnumList,&this->iCameraCounts);
    // std::cout<<this->tCameraEnumList[0].acFriendlyName<<std::endl;
    // std::cout<<this->tCameraEnumList[1].acFriendlyName<<std::endl;
    // std::cout<<this->tCameraEnumList[2].acFriendlyName<<std::endl;
    // std::cout<<this->tCameraEnumList[3].acFriendlyName<<std::endl;
    // std::cout<<this->iCameraCounts<<std::endl;
    // std::cout<<flag<<std::endl;
    //连接设备
    if (flag != CAMERA_STATUS_SUCCESS || this->iCameraCounts == 0){
        //return -1;
        printf("Cannot find any Camera\n");
        return false;
    }
    return true;
}

std::string Camera::Getname()   //得到相机昵称
{
    std::string name;
    char * cam_name;
    for (int i=0; i<this->iCameraCounts; i++){
        cam_name = this->tCameraEnumList[i].acFriendlyName;
        if (i > 0){
            name = name + "," + std::string(cam_name);
        }
        else{
            name = std::string(cam_name);
        }
    }
    if (0 < this->iCameraCounts){
        // return -1;
        return name;
    }
    return NULL;
}

bool Camera::SdkInit(int iLanguageSel) //Language init 0:English,1:Chinese
{
    int flag = -1;
    flag = CameraSdkInit(iLanguageSel);
    if (0 == flag){
        return true;
    }
    else{
        // printf("fail to CameraSdkInit\n");
        return false;
    }
}

int Camera::Init(const char* Cam)	//枚举后检查名字初始化,返回句柄号(相机ID);分配内存;设置1/3通道
{
    int   flag = -1;
    int   hCam = -1;
    for (int i = 0; i < this->iCameraCounts; ++i)
    {
        if ( 0 == strcmp(Cam,tCameraEnumList[i].acFriendlyName)){
            flag = CameraInit(&tCameraEnumList[i],-1,-1,&hCam);
            flag |= CameraGetCapability(hCam,&tCapability[hCam]);
            break;
        }
    }

    if (0 == flag){
        this->iplImage[hCam] = 0;
        this->g_pRgbBuffer[hCam] = (unsigned char*)malloc(tCapability[hCam].sResolutionRange.iHeightMax*tCapability[hCam].sResolutionRange.iWidthMax*3);
        if(tCapability[hCam].sIspCapacity.bMonoSensor){
            channel[hCam]=1;
            CameraSetIspOutFormat(hCam,CAMERA_MEDIA_TYPE_MONO8);
        }else{
            channel[hCam]=3;
            CameraSetIspOutFormat(hCam,CAMERA_MEDIA_TYPE_BGR8);
        }
        return hCam;
    }
    else{
        return -1;
    }
}

int Camera::getMVNum()  //得到枚举的相机数量
{
    return this->iCameraCounts;
}

int Camera::getMVunuseNum() //Not available
{
    int    flag = -1;
    int    piNums = 12;
    tSdkCameraDevInfo       tCameraEnumList[12];
    flag = CameraIdleStateDevice(tCameraEnumList, &piNums);
    // std::cout<<this->tCameraEnumList[0].acFriendlyName<<std::endl;
    // std::cout<<this->tCameraEnumList[1].acFriendlyName<<std::endl;
    // std::cout<<this->tCameraEnumList[2].acFriendlyName<<std::endl;
    if (0 == flag){
        return piNums;
    }
    else{
        // std::cout<<piNums<<std::endl;
        return -1;
    }
}

bool Camera::isOpen(char* Cam)  //Not available
{
    int i = 0;
    BOOL Bopen = false;
    for (i = 0; i < this->iCameraCounts; ++i)
    {
        if ( 0 == strcmp(Cam,tCameraEnumList[i].acFriendlyName)){
            CameraIsOpened(&tCameraEnumList[i],&Bopen);
        }
    }
    return Bopen;

}

bool Camera::UnInit(int hCam)   //反初始化
{
    if (CameraUnInit(hCam) == 0){
        return true;
    }
    else{
        return false;
    }
    // free(g_pRgbBuffer);
}

Mat Camera::getImage(int hCam, UINT wTimes_ms)
{
    // CameraGetCapability(hCam,&tCapability);
    // g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
    int flag = CameraGetImageBuffer(hCam,&sFrameInfo[hCam],&pbyBuffer[hCam],wTimes_ms);
    if( flag == 0){
        CameraImageProcess(hCam, pbyBuffer[hCam], g_pRgbBuffer[hCam], &sFrameInfo[hCam]);
        if(iplImage[hCam])
        {
            cvReleaseImageHeader(&iplImage[hCam]);
        }
        iplImage[hCam] = cvCreateImageHeader(cvSize(sFrameInfo[hCam].iWidth,sFrameInfo[hCam].iHeight),IPL_DEPTH_8U,channel[hCam]);
        cvSetData(iplImage[hCam],g_pRgbBuffer[hCam],sFrameInfo[hCam].iWidth*channel[hCam]);//此处只是设置指针，无图像块数据拷贝，不需担心转换效率
        //以下两种方式都可以显示图像或者处理图像

        Mat Iimag = cvarrToMat(iplImage[hCam]);//这里只是进行指针转换，将IplImage转换成Mat类型
//        Mat Iimag(iplImage[hCam]);//这里只是进行指针转换，将IplImage转换成Mat类型
        CameraReleaseImageBuffer(hCam,pbyBuffer[hCam]);
        return Iimag;
    }
    else{
        // printf("CameraGetImageBufferNo\n");
        Mat Iimag ;
        return Iimag;
    }
}


bool Camera::ReleaseImageBuffer(int hCam) //释放由CameraGetImageBuffer获得的缓冲区(pbyBuffer)
{
    if (CameraReleaseImageBuffer(hCam,pbyBuffer[hCam]) == 0){
        return true;
    }
    else{
        return false;
    }
}

bool Camera::Play(int hCam)	//开始接收来自相机发送的图像
{
    if (CameraPlay(hCam) == 0){
        return true;
    }
    else{
        return false;
    }
}

bool Camera::Pause(int hCam)	//暂停相机，可设置相机
{
    if (CameraPause(hCam) == 0){
        return true;
    }
    else{
        return false;
    }
}

bool Camera::Stop(int hCam)	//停止相机,一般是反初始化时调用该函数
{
    if (CameraStop(hCam) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetImageResolutionWidth(int hCam)
{
    tSdkImageResolution sResolution;
    if (CameraGetImageResolution(hCam,&sResolution) == 0){
        return sResolution.iWidth;
    }
    return -1;
}

int Camera::GetImageResolutionHeight(int hCam)
{
    tSdkImageResolution sResolution;
    if (CameraGetImageResolution(hCam,&sResolution) == 0){
        return sResolution.iHeight;
    }
    return -1;
}

bool Camera::SetImageResolution(int hCam,int Width,int Height)	//分辨率
{
    tSdkImageResolution sResolution;
    if (CameraGetImageResolution(hCam,&sResolution) != 0){
        return false;
    }
    sResolution.iIndex = 0xff;
    sResolution.iWidth = Width;
    sResolution.iWidthFOV = Width;
    sResolution.iHeight = Height;
    sResolution.iHeightFOV = Height;
    if (CameraSetImageResolution(hCam,&sResolution) == 0){
        return true;
    }
    else{
        return false;
    }
}

bool Camera::SetOffsetFOV(int hCam,int HOffsetFOV,int VOffsetFOV)  //
{
    tSdkImageResolution sResolution;
    if (CameraGetImageResolution(hCam,&sResolution) != 0){
        return false;
    }
    sResolution.iHOffsetFOV = HOffsetFOV;
    sResolution.iVOffsetFOV = VOffsetFOV;
    if (CameraSetImageResolution(hCam,&sResolution) == 0){
        return true;
    }
    else{
        return false;
    }
}

bool Camera::SetAeState(int hCam, bool State)	//自动曝光
{
    if (CameraSetAeState(hCam,State) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetAeState(int hCam)
{
    BOOL State;
    if (CameraGetAeState(hCam,&State) != 0)
    {
        return -1;
    }
    if (State)
    {
        return 1;
    }
    return 0;
}

bool Camera::SetSharpness(int hCam, int iSharpness)	//锐化
{
    //锐化范围具体参照tCapability.sSharpnessRange.iMax,
    //tCapability.sSharpnessRange.iMin,一般为0-100
    // CameraGetCapability(hCam,&tCapability);
    // std::cout<<tCapability.pClrTempDesc<<std::endl;

    if (CameraSetSharpness(hCam,iSharpness) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetSharpness(int hCam)
{
    int iSharpness;
    if (CameraGetSharpness(hCam,&iSharpness) == 0){
        return iSharpness;
    }
    else{
        return -1;
    }
}

bool Camera::SetLutMode(int hCam, int emLutMode)
{
    // emLutMode  LUTMODE_PARAM_GEN(0) 表示由伽马和对比度参数动态生成LUT表。
//             LUTMODE_PRESET(1)    表示使用预设的LUT表。
//             LUTMODE_USER_DEF(2)  表示使用用户自定的LUT表。
    if (CameraSetLutMode(hCam,emLutMode) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetLutMode(int hCam)
{
    int emLutMode;
    if (CameraGetLutMode(hCam,&emLutMode) == 0){
        return emLutMode;
    }
    else{
        return -1;
    }
}

bool Camera::SelectLutPreset(int hCam, int iSel)	//选择预设LUT模式
{
    if (CameraSelectLutPreset(hCam,iSel) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetLutPresetSel(int hCam)
{
    int iSel;
    if (CameraGetLutPresetSel(hCam,&iSel) == 0){
        return iSel;
    }
    else{
        return -1;
    }
}

bool Camera::SetWbMode(int hCam, bool bAuto)	//设置白平衡模式
{
    // bAuto      TRUE，则表示使能自动模式。
//             FALSE，则表示使用手动模式，通过调用
//                 CameraSetOnceWB来进行一次白平衡。
    // CameraGetCapability(hCam,&tCapability);
    // printf(tCapability.pClrTempDesc);
    //std::cout<<tCapability.pClrTempDesc.acDescription<<std::endl;
    if (CameraSetWbMode(hCam,bAuto) == 0){
        return true;
    }
    else{
        // std::cout<<CameraSetWbMode(hCam,bAuto)<<std::endl;
        return false;
    }
}

int Camera::GetWbMode(int hCam)
{
    BOOL bAuto;
    if (CameraGetWbMode(hCam,&bAuto) != 0){
        return -1;
    }
    if (bAuto)
    {
        return 1;
    }
    return 0;
}

bool Camera::SetPresetClrTemp(int hCam, int iSel)	//设预设色温模式
{
    if (CameraSetPresetClrTemp(hCam,iSel) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetPresetClrTemp(int hCam)
{
    int iSel;
    if (CameraGetPresetClrTemp(hCam,&iSel) == 0){
        return iSel;
    }
    else{
        return -1;
    }
}

bool Camera::SetUserClrTempGain(int hCam,int iRgain,int iGgain,int iBgain)	//红绿蓝增益（0-400,0-4倍）
{
    if (CameraSetUserClrTempGain(hCam,iRgain,iGgain,iBgain) == 0){
        return true;
    }
    else{
        return false;
    }
}


int Camera::GetUserClrTempGain(int hCam,int Colorsel)
{
    int iRgain;
    int iGgain;
    int iBgain;
    if (CameraGetUserClrTempGain(hCam,&iRgain,&iGgain,&iBgain) != 0){
        return -1;
    }
    if (0 == Colorsel)
        return iRgain;
    else if (1 == Colorsel)
        return iGgain;
    else if (2 == Colorsel)
        return iBgain;
    else
        return -1;
}

bool Camera::SetClrTempMode(int hCam,int iMode)		//
{
    // 自动模式下，会自动选择合适的色温模式
//              预设模式下，会使用用户指定的色温模式
//              自定义模式下，使用用户自定义的色温数字增益和矩阵
    if (CameraSetClrTempMode(hCam,iMode) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetClrTempMode(int hCam)
{
    int iMode;
    if (CameraGetClrTempMode(hCam,&iMode) == 0){
        return iMode;
    }
    else{
        return -1;
    }
}

bool Camera::SetOnceWB(int hCam)	//一次白平衡
{
    if (CameraSetOnceWB(hCam) == 0){
        return true;
    }
    else{
        return false;
    }
}

bool Camera::SetAeTarget(int hCam,int iAeTarget)	//设定自动曝光的亮度目标值
{
    if (CameraSetAeTarget(hCam,iAeTarget) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetAeTarget(int hCam)
{
    int iAeTarget;
    if (CameraGetAeTarget(hCam,&iAeTarget) == 0){
        return iAeTarget;
    }
    else{
        return -1;
    }
}

bool Camera::SetExposureTime(int hCam,int fExposureTime)	//设定曝光时间微秒
{
    if (CameraSetExposureTime(hCam,fExposureTime) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetExposureTime(int hCam)
{
    double fExposureTime;
    if (CameraGetExposureTime(hCam,&fExposureTime) == 0){
        return fExposureTime;
    }
    else{
        return -1;
    }
}

bool Camera::SetAnalogGain(int hCam,int iAnalogGain)	//图像模拟增益
{
    if (CameraSetAnalogGain(hCam,iAnalogGain) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetAnalogGain(int hCam)
{
    int iAnalogGain;
    if (CameraGetAnalogGain(hCam,&iAnalogGain) == 0){
        return iAnalogGain;
    }
    else{
        return -1;
    }
}

bool Camera::SetGain(int hCam,int iRgain,int iGgain,int iBgain)	//设置图像的数字增益0-399
{
    if (CameraSetGain(hCam,iRgain,iGgain,iBgain) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetGain(int hCam,int Colorsel)
{
    int iRgain;
    int iGgain;
    int iBgain;
    if (CameraGetGain(hCam,&iRgain,&iGgain,&iBgain) != 0){
        return -1;
    }
    if (0 == Colorsel)
        return iRgain;
    else if (1 == Colorsel)
        return iGgain;
    else if (2 == Colorsel)
        return iBgain;
    else
        return -1;

}

bool Camera::SetGamma(int hCam,int iGamma)	//SetGamma
{
    if (CameraSetGamma(hCam,iGamma) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetGamma(int hCam)
{
    int iGamma;
    if (CameraGetGamma(hCam,&iGamma) == 0){
        return iGamma;
    }
    else{
        return -1;
    }
}

bool Camera::SetContrast(int hCam,int iContrast)	//对比度
{
    if (CameraSetContrast(hCam,iContrast) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetContrast(int hCam)
{
    int iContrast;
    if (CameraGetContrast(hCam,&iContrast) == 0){
        return iContrast;
    }
    else{
        return -1;
    }
}

bool Camera::SetSaturation(int hCam,int iSaturation)	//饱和度
{
    if (CameraSetSaturation(hCam,iSaturation) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetSaturation(int hCam)
{
    int iSaturation;
    if (CameraGetSaturation(hCam,&iSaturation) == 0){
        return iSaturation;
    }
    else{
        return -1;
    }
}

// std::string Camera::getchar()
// {
//     char name[6] = "pName";
//     string NameStr = std::string(name);
//     return NameStr;
// }

bool Camera::SetMonochrome(int hCam,bool pbEnable)	//彩色转黑白
{
    if (CameraSetMonochrome(hCam,pbEnable) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetMonochrome(int hCam)
{
    BOOL pbEnable;
    if (CameraGetMonochrome(hCam,&pbEnable) != 0){
        return -1;
    }
    if (pbEnable)
    {
        return 1;
    }
    return 0;
}

bool Camera::SetInverse(int hCam,bool pbEnable)		//颜色反转
{
    if (CameraSetInverse(hCam,pbEnable) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetInverse(int hCam)
{
    BOOL pbEnable;
    if (CameraGetInverse(hCam,&pbEnable) != 0){
        return -1;
    }
    if (pbEnable)
    {
        return 1;
    }
    return 0;
}

bool Camera::SetAntiFlick(int hCam,bool pbEnable)	//自动曝光抗频闪使能，手动曝光模式下无效，没测试
{
    if (CameraSetAntiFlick(hCam,pbEnable) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetAntiFlick(int hCam)
{
    BOOL pbEnable;
    if (CameraGetAntiFlick(hCam,&pbEnable) != 0){
        return -1;
    }
    if (pbEnable)
    {
        return 1;
    }
    return 0;
}

bool Camera::SetLightFrequency(int hCam,int iFrequencySel)	//自动曝光时消频闪的频率，没测试
{
    if (CameraSetLightFrequency(hCam,iFrequencySel) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetLightFrequency(int hCam)
{
    int iFrequencySel;
    if (CameraGetLightFrequency(hCam,&iFrequencySel) != 0){
        return -1;
    }
    return iFrequencySel;
}

bool Camera::SetFrameSpeed(int hCam,int piFrameSpeed)	//相机输出图像的帧率，没测试
{
    if (CameraSetFrameSpeed(hCam,piFrameSpeed) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetFrameSpeed(int hCam)
{
    int piFrameSpeed;
    if (CameraGetFrameSpeed(hCam,&piFrameSpeed) != 0){
        return -1;
    }
    return piFrameSpeed;
}

bool Camera::SaveParameterToFile(int hCam,char* sFileName)	//It's not recommended
{
    if (CameraSaveParameterToFile(hCam,sFileName) == 0){
        return true;
    }
    else{

        return false;
    }
}

bool Camera::ReadParameterFromFile(int hCam,char* sFileName)	//It's not recommended
{
    if (CameraReadParameterFromFile(hCam,sFileName) == 0){
        return true;
    }
    else{
        std::cout<<CameraReadParameterFromFile(hCam,sFileName)<<std::endl;
        return false;
    }
}


bool Camera::WriteSN(int hCam,BYTE* pbySN, int iLevel)		//Not available
{
    // if (CameraWriteSN(hCam,pbySN,iLevel) == 0){
    //     return true;
    // }
    // else{
    //     return false;
    // }
}

BYTE Camera::ReadSN(int hCam,int iLevel)	//Not available
{
    BYTE pbySN;
    // int intSN;
    if (CameraReadSN(hCam,&pbySN,iLevel) != 0){
        return BYTE(-1);
    }
    // memcpy(&intSN, &pbySN, 4);
    // printf(intSN);
    // std::cout<<intSN<<std::endl;
    return pbySN;
}

bool Camera::SetTriggerDelayTime(int hCam,UINT uDelayTimeUs)
{
    if (CameraSetTriggerDelayTime(hCam,uDelayTimeUs) == 0){
        return true;
    }
    else{
        return false;
    }
}

UINT Camera::GetTriggerDelayTime(int hCam)
{
    UINT uDelayTimeUs;
    if (CameraGetTriggerDelayTime(hCam,&uDelayTimeUs) != 0){
        return -1;
    }
    return uDelayTimeUs;
}


bool Camera::SetTriggerCount(int hCam,INT iCount)
{
    if (CameraSetTriggerCount(hCam,iCount) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetTriggerCount(int hCam)
{
    INT iCount;
    if (CameraGetTriggerCount(hCam,&iCount) != 0){
        return -1;
    }
    return iCount;
}

bool Camera::SoftTrigger(int hCam)
{
    if (CameraSoftTrigger(hCam) != 0){
        return false;
    }
    return true;
}


bool Camera::SetTriggerMode(int hCam,INT iModeSel)
{
    if (CameraSetTriggerMode(hCam,iModeSel) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetTriggerMode(int hCam)
{
    INT iModeSel;
    if (CameraGetTriggerMode(hCam,&iModeSel) != 0){
        return -1;
    }
    return iModeSel;
}

bool Camera::SetStrobeMode(int hCam,INT iMode)
{
    if (CameraSetStrobeMode(hCam,iMode) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetStrobeMode(int hCam)
{
    INT iMode;
    if (CameraGetStrobeMode(hCam,&iMode) != 0){
        return -1;
    }
    return iMode;
}

bool Camera::SetStrobeDelayTime(int hCam,UINT uDelayTimeUs)
{
    if (CameraSetStrobeDelayTime(hCam,uDelayTimeUs) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetStrobeDelayTime(int hCam)
{
    UINT uDelayTimeUs;
    if (CameraGetStrobeDelayTime(hCam,&uDelayTimeUs) != 0){
        return -1;
    }
    return uDelayTimeUs;
}

bool Camera::SetStrobePulseWidth(int hCam,UINT uDelayTimeUs)
{
    if (CameraSetStrobePulseWidth(hCam,uDelayTimeUs) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetStrobePulseWidth(int hCam)
{
    UINT uDelayTimeUs;
    if (CameraGetStrobePulseWidth(hCam,&uDelayTimeUs) != 0){
        return -1;
    }
    return uDelayTimeUs;
}


bool Camera::SetStrobePolarity(int hCam,INT uPolarity)
{
    if (CameraSetStrobePolarity(hCam,uPolarity) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetStrobePolarity(int hCam)
{
    INT uPolarity;
    if (CameraGetStrobePolarity(hCam,&uPolarity) != 0){
        return -1;
    }
    return uPolarity;
}

bool Camera::SetExtTrigSignalType(int hCam,INT iType)
{
    if (CameraSetExtTrigSignalType(hCam,iType) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetExtTrigSignalType(int hCam)
{
    INT iType;
    if (CameraGetExtTrigSignalType(hCam,&iType) != 0){
        return -1;
    }
    return iType;
}

bool Camera::SetExtTrigShutterType(int hCam,INT iType)
{
    if (CameraSetExtTrigShutterType(hCam,iType) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetExtTrigShutterType(int hCam)
{
    INT iType;
    if (CameraGetExtTrigShutterType(hCam,&iType) != 0){
        return -1;
    }
    return iType;
}


bool Camera::SetExtTrigDelayTime(int hCam,UINT uDelayTimeUs)
{
    if (CameraSetExtTrigDelayTime(hCam,uDelayTimeUs) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetExtTrigDelayTime(int hCam)
{
    UINT uDelayTimeUs;
    if (CameraGetExtTrigDelayTime(hCam,&uDelayTimeUs) != 0){
        return -1;
    }
    return uDelayTimeUs;
}


bool Camera::SetExtTrigJitterTime(int hCam,UINT uDelayTimeUs)
{
    if (CameraSetExtTrigJitterTime(hCam,uDelayTimeUs) == 0){
        return true;
    }
    else{
        return false;
    }
}

UINT Camera::GetExtTrigJitterTime(int hCam)
{
    UINT uDelayTimeUs;
    if (CameraGetExtTrigJitterTime(hCam,&uDelayTimeUs) != 0){
        return -1;
    }
    return uDelayTimeUs;
}

// bool Camera::SetResolutionForSnap(int hCam,tSdkImageResolution* pImageResolution)

//     if (CameraSetResolutionForSnap(hCam,pImageResolution) == 0){
//         return true;
//     }
//     else{
//         return false;
//     }
// }

UINT Camera::GetExtTrigCapability(int hCam)
{
    UINT puCapabilityMask;
    if (CameraGetExtTrigCapability(hCam,&puCapabilityMask) != 0){
        return -1;
    }
    return puCapabilityMask;
}

bool Camera::SetNoiseFilter(int hCam,BOOL bEnable)	//降噪模块的使能，Not available
{
    if (CameraSetNoiseFilter(hCam,bEnable) == 0){
        return true;
    }
    else{
        return false;
    }
}

int Camera::GetNoiseFilterState(int hCam)
{
    BOOL bEnable;
    if (CameraGetNoiseFilterState(hCam,&bEnable) != 0){
        return -1;
    }
    if (bEnable)
    {
        return 1;
    }
    return 0;
}

bool Camera::SetFriendlyName(int hCam,char* pName)	//设置相机昵称
{
    if (CameraSetFriendlyName(hCam,pName) == 0){
        return true;
    }
    else{
        return false;
    }
}

std::string Camera::GetFriendlyName(int hCam)	//得到相机昵称
{
    char pName[32];
    if (CameraGetFriendlyName(hCam,pName) == 0){
        // return -1;
        std::string Name = std::string(pName);
        return Name;
    }
}

bool Camera::ReConnect(int hCam)	//Not available
{
    if (CameraReConnect(hCam) == 0){
        return true;
    }
    else{
        return false;
    }
}

bool Camera::ConnectTest(int hCam)	//Not available
{
    if (CameraConnectTest(hCam) == 0){
        return true;
    }
    else{
        return false;
    }
}









Camera::~Camera()
{
    std::cout<<"++++++++++++++++++++++++++++delete Camera\n";
    //CameraSaveParameter(hCamera, 0);

    for (int i = 0; i < 12; ++i)
    {
        if (NULL != this->g_pRgbBuffer[i]){
            // std::cout<<"g_pRgbBuffer"<<std::endl;
            // CameraUnInit(i);
            free(this->g_pRgbBuffer[i]);
            this->g_pRgbBuffer[i] = NULL;
        }
    }

    // free(&tCapability);

}
