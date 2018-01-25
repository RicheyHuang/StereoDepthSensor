#pragma once

#include <iostream>
#include "CameraApi.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <string.h>
#include <string>
#include <stdio.h>


class Camera
{
public:
    unsigned char           * g_pRgbBuffer[12];     //处理后数据缓存区
    int                     iCameraCounts;
    tSdkCameraDevInfo       tCameraEnumList[12];
    int                     hCamera_Fast;
    tSdkCameraCapbility     tCapability[12];      //设备描述信息
    tSdkFrameHead           sFrameInfo[12];
    BYTE*                   pbyBuffer[12];
    int                     channel[12];
    // char*                   cam_list[12];
    // int                     iDisplayFrames[12];
    // bool FastPhotoModel;
    IplImage *iplImage[12];

    Camera();
    ~Camera();
    bool EnumerateDevice();
    std::string Getname();
    bool SdkInit(int iLanguageSel);
    int Init(const char* Cam);
    int getMVNum();
    int getMVunuseNum();
    bool isOpen(char* Cam);
    bool UnInit(int hCam);
    cv::Mat getImage(int hCam, UINT wTimes_ms);
    bool ReleaseImageBuffer(int hCam);
    bool Play(int hCam);
    bool Pause(int hCam);
    bool Stop(int hCam);
    int GetImageResolutionWidth(int hCam);
    int GetImageResolutionHeight(int hCam);
    bool SetImageResolution(int hCam,int Width,int Height);
    bool SetOffsetFOV(int hCam,int HOffsetFOV,int VOffsetFOV);
    bool SetAeState(int hCam, bool State);
    int GetAeState(int hCam);
    bool SetSharpness(int hCam, int iSharpness);
    int GetSharpness(int hCam);
    bool SetLutMode(int hCam, int emLutMode);
    int GetLutMode(int hCam);
    bool SelectLutPreset(int hCam, int iSel);
    int GetLutPresetSel(int hCam);
    bool SetWbMode(int hCam, bool bAuto);
    int GetWbMode(int hCam);
    bool SetPresetClrTemp(int hCam, int iSel);
    int GetPresetClrTemp(int hCam);
    bool SetUserClrTempGain(int hCam,int iRgain,int iGgain,int iBgain);
    int GetUserClrTempGain(int hCam,int Colorsel);
    bool SetClrTempMode(int hCam,int iMode);
    int GetClrTempMode(int hCam);
    bool SetOnceWB(int hCam);
    bool SetAeTarget(int hCam,int iAeTarget);
    int GetAeTarget(int hCam);
    bool SetExposureTime(int hCam,int fExposureTime);
    int GetExposureTime(int hCam);
    bool SetAnalogGain(int hCam,int iAnalogGain);
    int GetAnalogGain(int hCam);
    bool SetGain(int hCam,int iRgain,int iGgain,int iBgain);
    int GetGain(int hCam,int Colorsel);
    bool SetGamma(int hCam,int iGamma);
    int GetGamma(int hCam);
    bool SetContrast(int hCam,int iContrast);
    int GetContrast(int hCam);
    bool SetSaturation(int hCam,int iSaturation);
    int GetSaturation(int hCam);
    // std::string getchar();
    bool SetMonochrome(int hCam,bool pbEnable);
    int GetMonochrome(int hCam);
    bool SetInverse(int hCam,bool pbEnable);
    int GetInverse(int hCam);
    bool SetAntiFlick(int hCam,bool pbEnable);
    int GetAntiFlick(int hCam);
    bool SetLightFrequency(int hCam,int iFrequencySel);
    int GetLightFrequency(int hCam);
    bool SetFrameSpeed(int hCam,int piFrameSpeed);
    int GetFrameSpeed(int hCam);
    bool SaveParameterToFile(int hCam,char* sFileName);
    bool ReadParameterFromFile(int hCam,char* sFileName);
    bool WriteSN(int hCam,BYTE* pbySN, int iLevel);
    BYTE ReadSN(int hCam,int iLevel);
    bool SetTriggerDelayTime(int hCam,UINT uDelayTimeUs);
    UINT GetTriggerDelayTime(int hCam);
    bool SetTriggerCount(int hCam,INT iCount);
    int GetTriggerCount(int hCam);
    bool SoftTrigger(int hCam);
    bool SetTriggerMode(int hCam,INT iModeSel);
    int GetTriggerMode(int hCam);
    bool SetStrobeMode(int hCam,INT iMode);
    int GetStrobeMode(int hCam);
    bool SetStrobeDelayTime(int hCam,UINT uDelayTimeUs);
    int GetStrobeDelayTime(int hCam);
    bool SetStrobePulseWidth(int hCam,UINT uDelayTimeUs);
    int GetStrobePulseWidth(int hCam);
    bool SetStrobePolarity(int hCam,INT uPolarity);
    int GetStrobePolarity(int hCam);
    bool SetExtTrigSignalType(int hCam,INT iType);
    int GetExtTrigSignalType(int hCam);
    bool SetExtTrigShutterType(int hCam,INT iType);
    int GetExtTrigShutterType(int hCam);
    bool SetExtTrigDelayTime(int hCam,UINT uDelayTimeUs);
    int GetExtTrigDelayTime(int hCam);
    bool SetExtTrigJitterTime(int hCam,UINT uDelayTimeUs);
    UINT GetExtTrigJitterTime(int hCam);
    UINT GetExtTrigCapability(int hCam);
    // bool SetResolutionForSnap(int hCam,tSdkImageResolution* pImageResolution);
    bool SetNoiseFilter(int hCam,BOOL bEnable);
    int GetNoiseFilterState(int hCam);
    bool SetFriendlyName(int hCam,char* pName);
    std::string GetFriendlyName(int hCam);
    bool ReConnect(int hCam);
    bool ConnectTest(int hCam);


};
