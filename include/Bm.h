/**************************************************************************************
from opencv calib3d module stereobm.cpp
****************************************************************************************/
#pragma once
#ifndef BM_H
#define BM_H

#include <opencv2/opencv.hpp>
//#include "precomp.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
//#undef CV_SSE2
//#define CV_SSE2 0
//#include "emmintrin.h"
#include <limits>




namespace cv
{



//void Bm_prefilterNorm( const cv::Mat& src, cv::Mat& dst, int winsize, int ftzero, uchar* buf );

//void Bm_prefilterXSobel( const cv::Mat& src, cv::Mat& dst, int ftzero );

//void Bm_findStereoCorrespondenceBM( const cv::Mat& left0, const cv::Mat& right0, cv::Mat& disp0, cv::Mat& diff0, CvStereoBMState* state, int nstripes,bool useShorts);


    static const int DISPARITY_SHIFT = 4;

#if CV_SSE2
    void Bm_findStereoCorrespondenceBM_SSE2( const cv::Mat& left, const cv::Mat& right,
                                             cv::Mat& disp, cv::Mat& cost, CvStereoBMState& state,
                                             uchar* buf, int _dy0, int _dy1 );
#endif

    void Bm_findStereoCorrespondenceBM( const cv::Mat& left, const cv::Mat& right,
                                        cv::Mat& disp, cv::Mat& cost, const CvStereoBMState& state,
                                        uchar* buf, int _dy0, int _dy1 );


}


class CV_EXPORTS_W MyBM: public cv::StereoBM
{
public:
    CV_WRAP MyBM();

    CV_WRAP MyBM(int preset, int ndisparities=0, int SADWindowSize=21) ;

    void operator()( cv::InputArray _left, cv::InputArray _right,
                     cv::OutputArray _disparity, int disptype, int nstripes,bool useShorts );
};



#endif