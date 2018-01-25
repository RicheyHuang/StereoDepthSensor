//
// Created by root on 17-9-13.
//
#pragma once
#include "opencv2/opencv.hpp"

static cv::Size SizeResize;
static cv::Size SizeOri;
static cv::Point StartPoint;
static bool MoveMode0;
static cv::Point RefPoint;

static void OnMouse(int Event,int x,int y,int flags,void* param);

void ShowImgGui(std::string WinName,cv::Mat Img);


