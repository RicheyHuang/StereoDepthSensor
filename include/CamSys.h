#pragma once
#include "Camera.h"
#include "CalibrationBoard.h"

class CamSys
{
public:
	Camera* cam;
	char* CamName;
	int CamHandle;
	cv::Mat frame;
	cv::Mat intrinsicMatrix;
	cv::Mat distortionMatrix;
	int height;
	int width;
	int ExposureTime;
	int MaxExposureTime;
    std::string ImgPath;    ///
    std::string ImgFormat;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

public:
    CamSys(Camera* cam,char* CamName,int width,int height,int ExposureTime, int MaxExposureTime, std::string ImgPath,std::string ImgFormat);
    void Initialize();
    void PreviewImage();
    void Trigger();
    void CapImage(UINT TimeOut=1000);
    void SaveFrame(std::string ImgName);
    void CalibrateCamera(CalibrationBoard CircleBoard,int CaliImgNum,bool CaputuredImgs);
	void LoadCamInfo();
};
