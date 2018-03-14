//
// Created by samuel on 17-8-4.
//

#pragma once
#include "opencv2/opencv.hpp"
#include <fcntl.h>
#include <linux/fb.h>
#include<sys/ioctl.h>
#include<pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
namespace cv
{
    using std::vector;
}

void DrawPoints(cv::Mat Img,cv::vector<cv::Point> Points);
void DrawPoints(cv::Mat Img,cv::vector<cv::Point2f> Points);
void DrawPoints(cv::Mat Img,cv::Point2f Points);


void ComputeContourCentroid(cv::vector<cv::Point> ContourPoints,cv::Point2f& CircleCentroid);

void ComputeImgBoldEdge(cv::Mat Img,cv::Mat& EdgeBoldU8);

void ComputeMinDistPoint(cv::vector<cv::Point2f> PointsGroup,cv::Point2f ComparedPoint,cv::Point2f& PointMinDist,int& MinIndex);

void saveMat(cv::Mat img, std::string folderPath, int imgNo, std::string suffix);

void loadMat(cv::Mat& img, std::string folderPath, int imgNo, std::string suffix);

void ComputePointsCenter(cv::vector<cv::Point> Points,cv::Point2f& PointCenter);

void GetScreenResolution(int& w,int& h);

void CreateImgWindow(std::string WindowName,cv::Size ImgSize,int mode=1,double para=0.85);

void CreateImgWindow(std::string WindowNameA,cv::Size ImgSizeA,std::string WindowNameB,cv::Size ImgSizeB,double para=0.85);

void CreateImgWindow(std::string WindowNameA,cv::Size ImgSizeA,std::string WindowNameB,cv::Size ImgSizeB,std::string WindowNameC,cv::Size ImgSizeC,double para=0.85);

bool JudgeFivePosition(cv::vector<cv::Point2f>FivePointsSort,cv::Point2f BoardMarker,int ImgWidth,int ImgHeight);

void saveXmlFile(cv::Mat Matrix,std::string Name,std::string Format,std::string savePath);

void loadXmlFile(cv::Mat& Matrix,std::string Name,std::string Format,std::string path);

void saveXYZ(std::string filenameString,const cv::Mat& mat);

void savePCD(std::string filename, const cv::Mat& mat);

void savePLY(std::string filename, const cv::Mat& mat);