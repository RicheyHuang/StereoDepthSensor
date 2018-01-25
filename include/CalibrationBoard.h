//
// Created by samuel on 17-8-10.
//
#pragma once
#include "opencv2/opencv.hpp"
#include "ToolBox.h"


class CalibrationBoard
{
    private:
        int PointsCols;
        int PointsRows;
        float CircleDistWidth;
        float CircleDistHeight;
        float BoardWidth;
        float BoardHeight;
        float CircleDiameter;
        int PointsNum;

    public:
        std::vector<cv::Point3f> ObjVector;
        cv::Size BoardSize;



        //cv::vector<cv::vector<cv::Point>> contours;
        //cv::Point2f CornerCentroid;
        //cv::vector<cv::Vec4i> hierarchy;
        //int idx;
        //int idxson;
        //cv::vector<cv::Point2f> CircleCoordinate;
        //cv::vector<cv::Point2f> CircleCoordinateSort;
        //cv::vector<cv::Point2f> FourCornerPointsUnSort;
        //cv::Point2f Point0;
        //cv::Point2f Point1;
        //cv::Point2f Point2;
        //cv::Point2f Point3;
        //cv::Mat Img;
        //cv::Mat ImgHomo;
        //cv::Mat HomoMatrix;
        //float SquareDimension;





    public:
    bool ExtractBoardCoordinate(cv::Mat Img,cv::Point2f& CornerCentroid,cv::vector<cv::Point2f>& CirclePointsSort);

    CalibrationBoard(int PointsCols,int PointsRows,float CircleDistWidth,float CircleDistHeight,float BoardWidth,float BoardHeight,float CircleDiameter);
    //cv::Mat GenerateChessboard(int offset_cols, int offset_rows,int resolution_x, int resolution_y);
    void GetObjVec();
    bool FindCalibrationBoard(cv::Mat Img,cv::vector<cv::vector<cv::Point>>& contours,cv::vector<cv::Vec4i>& hierarchy,int& idxson,cv::vector<cv::Point2f>& CircleCoordinate,int& i,int StartTh,int& UsedTh);
    bool FindUnSymmetricCorner(cv::vector<cv::Point2f> FourCornerPointsUnSort,cv::vector<cv::Point> FittingCurve,cv::Point2f& CornerCentroid);
    bool FindFourCornerPoints(cv::Mat Img,cv::vector<cv::vector<cv::Point>> contours,int idxson,cv::vector<cv::Point2f> CircleCoordinate,cv::vector<cv::Point2f>& FittingFourPoints,bool FivePointJudge,cv::Point2f& BoardMarker);
    void SortFourCornerPoints(cv::Point2f CornerCentroid, cv::vector<cv::Point2f> FourCornerPointsUnSort,cv::vector<cv::Point2f>& FourCornerPoints);
    void BoardRectification(cv::Mat Img,cv::vector<cv::Point2f> FourCornerPoints,cv::Mat& HomoMatrix,cv::Mat& ImgHomo);
    void SortCirclePoints(cv::Mat Img,cv::Mat HomoMatrix,cv::vector<cv::Point2f> CircleCoordinate,cv::vector<cv::Point2f>& CirclePointsSort);
    void CombineFivePointsIntoFourPoints(cv::vector<cv::Point> FivePoints,cv::vector<cv::Point2f>& FourPoints,cv::vector<cv::Point2f>& FivePointsSort,cv::Point2f& BoardMarker);

    //void FindRightPoint();
    //void FindDownPoint();
};
