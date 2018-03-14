//
// Created by root on 17-8-23.
//
#pragma once
#include"CamSys.h"
#include"Bm.h"
#include<time.h>
#include "ShowImgGui.h"

class StereoSystem
{
public:
    CamSys* CamLeft;
    CamSys* CamRight;
    std::string ImgPath;
    std::string ImgPathMatch;
    std::string ImgFormat;
    cv::Mat R;       //Output rotation matrix between the 1st and the 2nd camera coordinate systems.
    cv::Mat T;       //Output translation vector between the coordinate systems of the cameras.
    cv::Mat E;       //Output essential matrix.
    cv::Mat F;
    cv::Mat RL, RR, PL, PR, Q;
    cv::Size ImgSize;

    /// cv::StereoBM is a pure abstract class, so it cannot be used as a class member. The solution is to use pointer
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create();

    int CamWorkingRange;
    int WorkingDistance[2];
    int MaxSADWindowSize;
    int MaxValForMinDist;
    int MaxWorkingRange;
    int MaxTextureThreshold;
    int MaxUniquenessRatio;

    int blockSize;
    int textureThreshold;
    int uniquenessRatio;

    cv::Mat ReMapLeftX;
    cv::Mat ReMapLeftY;
    cv::Mat ReMapRightX;
    cv::Mat ReMapRightY;
    cv::Mat RectifyLeft;
    cv::Mat RectifyRight;
    cv::Size DepthMapSize;
    std::string HandleLeft;
    std::string HandleRight;
    cv::Mat dispsbm;
    cv::Mat dispsbm8;
    cv::Mat Pts3D;
    double minVal, maxVal;
    std::string HandleDepthMap;

    static void onChangeLeftCam(int ExposureTime, void *ptr);
    static void onChangeRightCam(int ExposureTime, void *ptr);

    static void onChangeSADWinSize(int SADWindowsSize, void *ptr);
    static void onChangeMinDist(int MinDist,void *ptr);
    static void onChangeWorkRange(int WorkingRange, void *ptr);
    static void onChangeTextureThresh(int TextureThresh, void *ptr);
    static void onChangeUniquenessRatio(int UniquenessRatio, void *ptr);

public:
    StereoSystem(CamSys* CamLeft,CamSys* CamRight,std::string ImgPath,std::string ImgPathMatch,std::string ImgFormat,
                 int MaxSADWindowSize, int MaxValForMinDist, int MaxWorkingRange, int MaxTextureThreshold, int MaxUniquenessRatio);
    StereoSystem();
    bool SelectCamDirection();
    void AdjustCameraFocus();
    void StereoCalibration(CalibrationBoard Board,int CaliImgNum,bool CaputureImgs);
    void LoadStereoCamInfo(bool LeftRight);
    void RectifyPreview();
    void Compute3DMap(int* WorkingDist,bool LeftRight,bool DebugMode, int SADWindowSize, int TextureThreshold);

    void AdjustLeftCamExposure(int, void *);
    void AdjustRightCamExposure(int, void *);

    void UpdateSADWindowSize(int SADWindowSize, void*);
    void UpdateMinDist(int MinDist, void*);
    void UpdateWorkingRange(int WorkingRange, void*);
    void UpdateTextureThresh(int TextureThresh, void*);
    void UpdateUniquenessRatio(int UniquenessRatio, void*);

};

