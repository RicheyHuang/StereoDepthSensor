#include <iostream>
#include <string.h>
#include "opencv2/opencv.hpp"
#include "ToolBox.h"
#include "CalibrationBoard.h"
#include "CamSys.h"
#include "Camera.h"
#include "StereoSystem.h"

#define CAMERA_GROUP_1  1;
#define CAMERA_GROUP_2  2;
#define CAMERA_GROUP_3  3;
#define CAMERA_CALIB_CAM1_TO_CAM2 4;
#define CAMERA_CALIB_CAM3_TO_CAM2 5;
#define CAMERA_CALIB_CAM1_TO_CAM3 6;

/// The code below uses left cam as the reference ///

int main()
{
    int UsingCamGroup = CAMERA_CALIB_CAM1_TO_CAM3;

    /// Parameters adjusted by trackbar ///
    int ImageWidth  = 1280;
    int ImageHeight = 720;

    int ExposureTimeLeftCam  = 18000;
    int ExposureTimeRightCam = 18000;
    int SADWindowSize        = 17;
    int MinDist              = 330;
    int WorkingRange         = 50;

    int MaxExposureTime      = 500000;
    int MaxSADWindowSize     = 99;
    int MaxValForMinDist     = 500;
    int MaxWorkingRange      = 80;

    int TextureThreshold     = 5000;
    int MaxTextureThreshold  = 10000;

    /// Cam Group1 ///
    char *CamName1L = "Cam130_1L";
    char *CamName1R = "Cam130_1R";
    std::string CalibrateImgPathCam1L         = "..//Calibration//Cam130_1//CalibrateA//";
    std::string CalibrateImgPathCam1R         = "..//Calibration//Cam130_1//CalibrateB//";
    std::string StereoCalibrateImgPathCam1    = "..//Calibration//Cam130_1//CalibrateStereo//";
    std::string StereoMatchingImgPathCam1     = "..//Calibration//Cam130_1//StereoMatching//";

    /// Cam Group2 ///
    char *CamName2L = "Cam130_2L";
    char *CamName2R = "Cam130_2R";
    std::string CalibrateImgPathCam2L         = "..//Calibration//Cam130_2//CalibrateA//";
    std::string CalibrateImgPathCam2R         = "..//Calibration//Cam130_2//CalibrateB//";
    std::string StereoCalibrateImgPathCam2    = "..//Calibration//Cam130_2//CalibrateStereo//";
    std::string StereoMatchingImgPathCam2     = "..//Calibration//Cam130_2//StereoMatching//";

    /// Cam Group3 ///
    char *CamName3L = "Cam130_3L";
    char *CamName3R = "Cam130_3R";
    std::string CalibrateImgPathCam3L         = "..//Calibration//Cam130_3//CalibrateA//";
    std::string CalibrateImgPathCam3R         = "..//Calibration//Cam130_3//CalibrateB//";
    std::string StereoCalibrateImgPathCam3    = "..//Calibration//Cam130_3//CalibrateStereo//";
    std::string StereoMatchingImgPathCam3     = "..//Calibration//Cam130_3//StereoMatching//";

    /// Calib Cam1 to Cam2 ///
    std::string CalibrateImgPathCalib1Cam2    = "..//Calibration//CamCalib1to2//CalibrateA//";
    std::string CalibrateImgPathCalib1Cam1    = "..//Calibration//CamCalib1to2//CalibrateB//";
    std::string StereoCalibrateImgPathCam1to2 = "..//Calibration//CamCalib1to2//CalibrateStereo//";
    std::string StereoMatchingImgPathCam1to2  = "..//Calibration//CamCalib1to2//StereoMatching//";

    /// Calib Cam3 to Cam2 ///
    std::string CalibrateImgPathCalib2Cam2    = "..//Calibration//CamCalib3to2//CalibrateA//";
    std::string CalibrateImgPathCalib2Cam3    = "..//Calibration//CamCalib3to2//CalibrateB//";
    std::string StereoCalibrateImgPathCam3to2 = "..//Calibration//CamCalib3to2//CalibrateStereo//";
    std::string StereoMatchingImgPathCam3to2  = "..//Calibration//CamCalib3to2//StereoMatching//";

    /// Calib Cam1 to Cam3 ///
    std::string CalibrateImgPathCalib3Cam3    = "..//Calibration//CamCalib1to3//CalibrateA//";
    std::string CalibrateImgPathCalib3Cam1    = "..//Calibration//CamCalib1to3//CalibrateB//";
    std::string StereoCalibrateImgPathCam1to3 = "..//Calibration//CamCalib1to3//CalibrateStereo//";
    std::string StereoMatchingImgPathCam1to3  = "..//Calibration//CamCalib1to3//StereoMatching//";


    std::string ImgFormat = ".png";

//// chessboard parameter ////
    int   PointsCols        = 7;
    int   PointsRows        = 7;
    float CircleDistWidth   = 2.5;      /// mm
    float CircleDistHeight  = 2.5;      /// mm

    float CircleDiameter    = 1;        /// mm
    float BoardWidth        = 20;       /// mm
    float BoardHeight       = 20;       /// mm

///////////////////////////////
    bool LeftRight          = false;
    /// true:  map coordinate sys of right cam to left cam (left cam as reference)
    /// false: map coordinate sys of left cam to right cam (right cam as reference)

    bool OpenCamera         = true;
    bool DebugMode          = false;
    bool DoCamerCalibration = true;
    bool CaputureImgs       = false;

    CalibrationBoard CircleBoard(PointsCols,PointsRows,CircleDistWidth,CircleDistHeight,BoardWidth,BoardHeight,CircleDiameter);

    Camera *cam = new Camera();
    if (OpenCamera)
    {
        bool FindCam = cam->EnumerateDevice();
        if (!FindCam)
        {
            std::cout << "Terminate program" << std::endl;
            exit(0);
        }
    }

    switch (UsingCamGroup)
    {
        case 1:
        {
            /// Cam Group1 ///
            CamSys Cam1L(cam, CamName1L, ImageWidth, ImageHeight, ExposureTimeLeftCam,  MaxExposureTime, CalibrateImgPathCam1L, ImgFormat);    ///initialize camera1
            CamSys Cam1R(cam, CamName1R, ImageWidth, ImageHeight, ExposureTimeRightCam, MaxExposureTime, CalibrateImgPathCam1R, ImgFormat);    ///initialize camera1
            Cam1L.Initialize();
            Cam1R.Initialize();

            StereoSystem StereoCam1LR(&Cam1L, &Cam1R, StereoCalibrateImgPathCam1, StereoMatchingImgPathCam1, ImgFormat,
                                      MaxSADWindowSize, MaxValForMinDist, MaxWorkingRange, MaxTextureThreshold);

            if (DoCamerCalibration)
            {
                if  (CaputureImgs)
                {
                    bool ChangeDirection = StereoCam1LR.SelectCamDirection();
                    if (ChangeDirection)
                    {
                        std::cout<<"Need To Change Camera Position!!!"<<std::endl;
                        exit(0);
                    }
                    //////////// adjust focus ////////////
                    StereoCam1LR.AdjustCameraFocus();
                }
                ////  Single Camera Calibration  /////
                StereoCam1LR.CamLeft->CalibrateCamera(CircleBoard,15,CaputureImgs);
                StereoCam1LR.CamRight->CalibrateCamera(CircleBoard,15,CaputureImgs);
                ////  Stereo Camera Calibration  /////
                StereoCam1LR.StereoCalibration(CircleBoard,15,CaputureImgs);
            }

            StereoCam1LR.LoadStereoCamInfo(LeftRight);
            int WorkingDistCam1[2] = {MinDist,MinDist+WorkingRange};
            StereoCam1LR.Compute3DMap(WorkingDistCam1,LeftRight,DebugMode, SADWindowSize, TextureThreshold);
        }
            break;

        case 2:
        {
            /// Cam Group2 ///
            CamSys Cam2L(cam, CamName2L, ImageWidth, ImageHeight, ExposureTimeLeftCam,  MaxExposureTime, CalibrateImgPathCam2L, ImgFormat);    ///initialize camera2
            CamSys Cam2R(cam, CamName2R, ImageWidth, ImageHeight, ExposureTimeRightCam, MaxExposureTime, CalibrateImgPathCam2R, ImgFormat);    ///initialize camera2
            Cam2L.Initialize();
            Cam2R.Initialize();
            StereoSystem StereoCam2LR(&Cam2L, &Cam2R, StereoCalibrateImgPathCam2, StereoMatchingImgPathCam2, ImgFormat,
                                      MaxSADWindowSize, MaxValForMinDist, MaxWorkingRange, MaxTextureThreshold);

            if (DoCamerCalibration)
            {
                if  (CaputureImgs)
                {
                    bool ChangeDirection = StereoCam2LR.SelectCamDirection();
                    if (ChangeDirection)
                    {
                        std::cout<<"Need To Change Camera Position!!!"<<std::endl;
                        exit(0);
                    }
                    //////////// adjust focus ////////////
                    StereoCam2LR.AdjustCameraFocus();
                }
                ////  Single Camera Calibration  /////
                StereoCam2LR.CamLeft->CalibrateCamera(CircleBoard,15,CaputureImgs);
                StereoCam2LR.CamRight->CalibrateCamera(CircleBoard,15,CaputureImgs);
                ////  Stereo Camera Calibration  /////
                StereoCam2LR.StereoCalibration(CircleBoard,15,CaputureImgs);
            }

            StereoCam2LR.LoadStereoCamInfo(LeftRight);
            int WorkingDistCam2[2] = {MinDist,MinDist+WorkingRange};
            StereoCam2LR.Compute3DMap(WorkingDistCam2,LeftRight,DebugMode, SADWindowSize, TextureThreshold);
        }
            break;

        case 3:
        {
            /// Cam Group3 ///
            CamSys Cam3L(cam, CamName3L, ImageWidth, ImageHeight, ExposureTimeLeftCam,  MaxExposureTime, CalibrateImgPathCam3L, ImgFormat);    ///initialize camera3
            CamSys Cam3R(cam, CamName3R, ImageWidth, ImageHeight, ExposureTimeRightCam, MaxExposureTime, CalibrateImgPathCam3R, ImgFormat);    ///initialize camera3
            Cam3L.Initialize();
            Cam3R.Initialize();

            StereoSystem StereoCam3LR(&Cam3L, &Cam3R, StereoCalibrateImgPathCam3, StereoMatchingImgPathCam3, ImgFormat,
                                      MaxSADWindowSize, MaxValForMinDist, MaxWorkingRange, MaxTextureThreshold);

            if (DoCamerCalibration)
            {
                if  (CaputureImgs)
                {
                    bool ChangeDirection = StereoCam3LR.SelectCamDirection();
                    if (ChangeDirection)
                    {
                        std::cout<<"Need To Change Camera Position!!!"<<std::endl;
                        exit(0);
                    }
                    //////////// adjust focus ////////////
                    StereoCam3LR.AdjustCameraFocus();
                }
                ////  Single Camera Calibration  /////
                StereoCam3LR.CamLeft->CalibrateCamera(CircleBoard,15,CaputureImgs);
                StereoCam3LR.CamRight->CalibrateCamera(CircleBoard,15,CaputureImgs);
                ////  Stereo Camera Calibration  /////
                StereoCam3LR.StereoCalibration(CircleBoard,15,CaputureImgs);
            }

            StereoCam3LR.LoadStereoCamInfo(LeftRight);
            int WorkingDistCam3[2] = {MinDist,MinDist+WorkingRange};
            StereoCam3LR.Compute3DMap(WorkingDistCam3,LeftRight,DebugMode, SADWindowSize, TextureThreshold);
        }
            break;

        case 4:
        {
            /// Calib Cam1 to Cam2 ///
            CamSys Calib1Cam1R(cam, CamName1R, ImageWidth, ImageHeight, ExposureTimeLeftCam,  MaxExposureTime,
                               CalibrateImgPathCalib1Cam1, ImgFormat);    ///initialize camera1
            CamSys Calib1Cam2R(cam, CamName2R, ImageWidth, ImageHeight, ExposureTimeRightCam, MaxExposureTime,
                               CalibrateImgPathCalib1Cam2, ImgFormat);    ///initialize camera2
            Calib1Cam1R.Initialize();
            Calib1Cam2R.Initialize();

            StereoSystem StereoCam1Cam2(&Calib1Cam1R, &Calib1Cam2R, StereoCalibrateImgPathCam1to2,
                                        StereoMatchingImgPathCam1to2, ImgFormat,
                                        MaxSADWindowSize, MaxValForMinDist, MaxWorkingRange, MaxTextureThreshold);

            if (DoCamerCalibration)
            {
                if (CaputureImgs) {
                    bool ChangeDirection = StereoCam1Cam2.SelectCamDirection();
                    if (ChangeDirection) {
                        std::cout << "Need To Change Camera Position!!!" << std::endl;
                        exit(0);
                    }
                    //////////// adjust focus ////////////
                    StereoCam1Cam2.AdjustCameraFocus();
                }
                ////  Single Camera Calibration  /////
                StereoCam1Cam2.CamLeft->CalibrateCamera(CircleBoard, 15, CaputureImgs);
                StereoCam1Cam2.CamRight->CalibrateCamera(CircleBoard, 15, CaputureImgs);
                ////  Stereo Camera Calibration  /////
                StereoCam1Cam2.StereoCalibration(CircleBoard, 15, CaputureImgs);
            }

            StereoCam1Cam2.LoadStereoCamInfo(LeftRight);
            int WorkingDistCam1Cam2[2] = {MinDist, MinDist + WorkingRange};
            StereoCam1Cam2.Compute3DMap(WorkingDistCam1Cam2, LeftRight, DebugMode, SADWindowSize, TextureThreshold);
        }
            break;

        case 5:
        {
            /// Calib Cam3 to Cam2 ///
            CamSys Calib2Cam3R(cam, CamName3R, ImageWidth, ImageHeight, ExposureTimeLeftCam, MaxExposureTime,
                               CalibrateImgPathCalib2Cam3, ImgFormat);    ///initialize camera3
            CamSys Calib2Cam2R(cam, CamName2R, ImageWidth, ImageHeight, ExposureTimeRightCam, MaxExposureTime,
                               CalibrateImgPathCalib2Cam2, ImgFormat);    ///initialize camera2
            Calib2Cam3R.Initialize();
            Calib2Cam2R.Initialize();

            StereoSystem StereoCam3Cam2(&Calib2Cam3R, &Calib2Cam2R, StereoCalibrateImgPathCam3to2,
                                        StereoMatchingImgPathCam3to2, ImgFormat,
                                        MaxSADWindowSize, MaxValForMinDist, MaxWorkingRange, MaxTextureThreshold);

            if (DoCamerCalibration)
            {
                if (CaputureImgs)
                {
                    bool ChangeDirection = StereoCam3Cam2.SelectCamDirection();
                    if (ChangeDirection) {
                        std::cout << "Need To Change Camera Position!!!" << std::endl;
                        exit(0);
                    }
                    //////////// adjust focus ////////////
                    StereoCam3Cam2.AdjustCameraFocus();
                }
                ////  Single Camera Calibration  /////
                StereoCam3Cam2.CamLeft->CalibrateCamera(CircleBoard, 15, CaputureImgs);
                StereoCam3Cam2.CamRight->CalibrateCamera(CircleBoard, 15, CaputureImgs);
                ////  Stereo Camera Calibration  /////
                StereoCam3Cam2.StereoCalibration(CircleBoard, 15, CaputureImgs);
            }

            StereoCam3Cam2.LoadStereoCamInfo(LeftRight);
            int WorkingDistCam3Cam2[2] = {MinDist, MinDist + WorkingRange};
            StereoCam3Cam2.Compute3DMap(WorkingDistCam3Cam2, LeftRight, DebugMode, SADWindowSize, TextureThreshold);
        }
            break;

        case 6:
        {
            /// Calib Cam1 to Cam3 ///
            CamSys Calib3Cam1R(cam, CamName1R, ImageWidth, ImageHeight, ExposureTimeLeftCam, MaxExposureTime,
                               CalibrateImgPathCalib3Cam1, ImgFormat);    ///initialize camera1
            CamSys Calib3Cam3R(cam, CamName3R, ImageWidth, ImageHeight, ExposureTimeRightCam, MaxExposureTime,
                               CalibrateImgPathCalib3Cam3, ImgFormat);    ///initialize camera3
            Calib3Cam1R.Initialize();
            Calib3Cam3R.Initialize();

            StereoSystem StereoCam1Cam3(&Calib3Cam1R, &Calib3Cam3R, StereoCalibrateImgPathCam1to3,
                                        StereoMatchingImgPathCam1to3, ImgFormat,
                                        MaxSADWindowSize, MaxValForMinDist, MaxWorkingRange, MaxTextureThreshold);

            if (DoCamerCalibration)
            {
                if (CaputureImgs)
                {
                    bool ChangeDirection = StereoCam1Cam3.SelectCamDirection();
                    if (ChangeDirection) {
                        std::cout << "Need To Change Camera Position!!!" << std::endl;
                        exit(0);
                    }
                    //////////// adjust focus ////////////
                    StereoCam1Cam3.AdjustCameraFocus();
                }
                ////  Single Camera Calibration  /////
                StereoCam1Cam3.CamLeft->CalibrateCamera(CircleBoard, 15, CaputureImgs);
                StereoCam1Cam3.CamRight->CalibrateCamera(CircleBoard, 15, CaputureImgs);
                ////  Stereo Camera Calibration  /////
                StereoCam1Cam3.StereoCalibration(CircleBoard, 15, CaputureImgs);
            }

            StereoCam1Cam3.LoadStereoCamInfo(LeftRight);
            int WorkingDistCam1Cam3[2] = {MinDist, MinDist + WorkingRange};
            StereoCam1Cam3.Compute3DMap(WorkingDistCam1Cam3, LeftRight, DebugMode, SADWindowSize, TextureThreshold);
        }
            break;

        default:
            break;
    }

    return 0;
}
