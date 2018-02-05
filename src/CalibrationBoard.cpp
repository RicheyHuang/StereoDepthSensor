//
// Created by samuel on 17-8-10.
//
#include "CalibrationBoard.h"

namespace cv
{
    using std::vector;
}

CalibrationBoard::CalibrationBoard(int PointsCols,int PointsRows,float CircleDistWidth,float CircleDistHeight,float BoardWidth,float BoardHeight,float CircleDiameter):
        PointsCols(PointsCols), PointsRows(PointsRows),PointsNum(PointsCols*PointsRows),CircleDistWidth(CircleDistWidth),CircleDistHeight(CircleDistHeight), BoardWidth(BoardWidth), BoardHeight(BoardHeight),BoardSize(cv::Size(PointsCols,PointsRows)),CircleDiameter(CircleDiameter)
{
    CalibrationBoard::GetObjVec();
}

bool CalibrationBoard::ExtractBoardCoordinate(cv::Mat Img,cv::Point2f& BoardMarker,cv::vector<cv::Point2f>& CirclePointsSort)
{
    cv::vector<cv::vector<cv::Point>> contours;
    cv::vector<cv::Vec4i> hierarchy;
    int idxson;
    cv::vector<cv::Point2f> CircleCoordinate;
    cv::vector<cv::Point2f> FourCornerPointsUnSort;
    cv::vector<cv::Point2f> FourCornerPoints;
    cv::Mat HomoMatrix;
    cv::Mat HomoMatrix2;
    cv::Mat ImgGray;
    cv::Mat ImgHomo;
    cv::Mat BinImg;
    bool FindBoard;
    int StartTh;
    int ThLastTime;


    /////////////////////////////////////    convert image to gray image     //////////////////////////////
    if (Img.channels()==3)
    {
        cv::cvtColor(Img,ImgGray, cv::COLOR_RGB2GRAY);
    }
    else
    {
        Img.copyTo(ImgGray);
    }

    StartTh = cv::threshold(ImgGray, BinImg, 0, 255, CV_THRESH_OTSU+CV_THRESH_BINARY_INV);
    /////////////////////////////////////    first time, rectify calibration board, but the rotation may be wrong  ////////////////////////////////////
    int ThLoopTime;

    ThLoopTime = 0;
    while(true)
    {
        FindBoard = CalibrationBoard::FindCalibrationBoard(ImgGray, contours, hierarchy, idxson, CircleCoordinate, ThLoopTime,StartTh,ThLastTime);
        if (!FindBoard)
        {
            return FindBoard;
        }
        FindBoard = CalibrationBoard::FindFourCornerPoints(ImgGray, contours, idxson, CircleCoordinate, FourCornerPointsUnSort,false,BoardMarker);
        if (FindBoard)
        {
            break;
        }
    }

    /////////////////BoardMarker = OutLinerFittingPoints[1];  for debug
    CalibrationBoard::SortFourCornerPoints(BoardMarker, FourCornerPointsUnSort,FourCornerPoints);
    CalibrationBoard::BoardRectification(ImgGray,FourCornerPoints,HomoMatrix,ImgHomo);
    //cv::imshow("ImgHomo",ImgHomo);
    //cv::waitKey(0);
    //DrawPoints(ImgGray,CircleCoordinate);

    ////////////////////////////////////////////  at the first time ,the rotation may be wrong,so at second time,adjust the rotation  ///////////////////////////////////////////////////////
    ThLoopTime = 0;
    StartTh = ThLastTime;
    while (true)
    {
        FindBoard = CalibrationBoard::FindCalibrationBoard(ImgHomo, contours, hierarchy, idxson, CircleCoordinate, ThLoopTime,StartTh,ThLastTime);
        //DrawPoints(ImgHomo,CircleCoordinate);
        //DrawPoints(ImgHomo,contours[idxson]);
        if (!FindBoard)
        {
            return FindBoard;
        }
        FindBoard = CalibrationBoard::FindFourCornerPoints(ImgHomo, contours, idxson, CircleCoordinate, FourCornerPointsUnSort, true,BoardMarker);
        if (FindBoard)
        {
            break;
        }
    }
   // cv::imshow("ImgHomo",ImgHomo);
   // cv::waitKey(0);

   // DrawPoints(ImgHomo,CircleCoordinate);

    ////////CalibrationBoard::FindUnSymmetricCorner(FourCornerPointsUnSort,OutLinerFittingPoints,BoardMarker);  /// useless now
    //////BoardMarker = OutLinerFittingPoints[2];   for debug
    CalibrationBoard::SortFourCornerPoints(BoardMarker, FourCornerPointsUnSort,FourCornerPoints);
    CalibrationBoard::BoardRectification(ImgHomo,FourCornerPoints,HomoMatrix2,ImgHomo);
    ///// apply bilateral filter ///////
    cv::Mat ImgHomoFilter;
    cv::bilateralFilter(ImgHomo,ImgHomoFilter,9,75,75);
    //cv::imshow("ImgHomoFilter",ImgHomoFilter);
    //cv::waitKey(0);

    /////////////////////////////////// At last on the final image(right rotation and rectified),extract circle center data  ///////////////////////////
    StartTh = ThLastTime;
    FindBoard = CalibrationBoard::FindCalibrationBoard(ImgHomoFilter,contours,hierarchy,idxson,CircleCoordinate,ThLoopTime=0,StartTh,ThLastTime);

    if (!FindBoard)
    {
        return FindBoard;
    }

    //// Compute  original CornerCentroid  /////
    cv::vector<cv::Point2f> BoardMarkerVecotr;
    BoardMarkerVecotr.push_back(BoardMarker);
    cv::perspectiveTransform(BoardMarkerVecotr,BoardMarkerVecotr,HomoMatrix.inv());
    BoardMarker = BoardMarkerVecotr[0];
    HomoMatrix = HomoMatrix2*HomoMatrix;

    CalibrationBoard::SortCirclePoints(ImgHomoFilter,HomoMatrix,CircleCoordinate,CirclePointsSort);


    return FindBoard;
}


void CalibrationBoard::GetObjVec()
{
    // for real chessobard
    ObjVector.clear();
    for(int num_loop=0;num_loop<PointsNum;num_loop++)
    { ObjVector.emplace_back(cv::Point3f((num_loop% PointsCols)*(CircleDistWidth), (num_loop/ PointsCols)* (CircleDistHeight), 0.0f));}
}

bool CalibrationBoard::FindCalibrationBoard(cv::Mat Img,cv::vector<cv::vector<cv::Point>>& contours,cv::vector<cv::Vec4i>& hierarchy,int& idxson,cv::vector<cv::Point2f>& CircleCoordinate,int& i,int StartTh,int& UsedTh)
{
    ///////////////////////////////////////////////////////  the input image is gray image  //////////
    int idx;
    int idxgrandson;
    bool FindBoard = false;
    double ContourArea;
    double ContourAreaSon;
    double ContourAreaGrandSon;

    cv::vector<cv::vector<cv::Point>> SubContours;
    cv::vector<cv::Vec4i> SubHierarchy;

    cv::Mat BinImg;
    //this->Img=Img;

    cv::Mat dst = cv::Mat::zeros(Img.rows, Img.cols, CV_8UC3);
    cv::Mat Background = cv::Mat::zeros(Img.rows, Img.cols, CV_8UC1);
    cv::Mat SubEdge = cv::Mat::zeros(Img.rows, Img.cols, CV_8UC1);

    cv::Mat EdgeBold;
    ComputeImgBoldEdge(Img,EdgeBold);

    int OutLinerTh = (Img.rows/10)*(Img.rows/10);    ////////// if the chessboard is too small, may adjust this parameter

    //
    int StepTh = 1;
    int Th = Th=StartTh+StepTh*i*pow(-1,i);
    //int i=0;
    for (;(Th>0)&&(Th<255);Th=StartTh+StepTh*i*pow(-1,i))
    {
        cv::threshold(Img, BinImg, Th, 255, CV_THRESH_BINARY_INV);
        //cv::imshow("BinImg",BinImg);
        //cv::waitKey(0);
        cv::findContours(BinImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

        i++;
        for (idx = 0; idx < contours.size(); idx++) //idx = hierarchy[idx][0]
        {
            ContourArea = contourArea(contours[idx]);
            if (ContourArea > OutLinerTh)
            {
                idxson = hierarchy[idx][2];
                for (; idxson >= 0; idxson = hierarchy[idxson][0])
                {
                    ContourAreaSon = contourArea(contours[idxson]);
                    if (ContourAreaSon > ContourArea * 0.4)
                    {
                        //DrawPoints(Img,contours[idx]);
                        //DrawPoints(Img,contours[idxson]);
                        idxgrandson = hierarchy[idxson][2];
                        int CountCircleNum = 0;
                        double ContourAreaTh0 = ContourArea*(3.14159*CircleDiameter*CircleDiameter/4)/(BoardHeight*BoardHeight)/6;
                        double ContourAreaTh1 = ContourArea*(3.14159*CircleDiameter*CircleDiameter/4)/(BoardHeight*BoardHeight)*6;
                        CircleCoordinate.clear();
                        for (; idxgrandson >= 0; idxgrandson = hierarchy[idxgrandson][0])
                        {
                            ContourAreaGrandSon = contourArea(contours[idxgrandson]);
                            if ((ContourAreaGrandSon > ContourAreaTh0)&&(ContourAreaGrandSon < ContourAreaTh1))    //////////////////////////   ContourAreaTh
                            {
                                //std::cout<<ContourAreaGrandSon<<std::endl;
                                int CW = sqrt(ContourAreaGrandSon/3.14);
                                //std::cout<<CW<<std::endl;
                                //std::cout<<ContourAreaTh<<std::endl;
                                //DrawPoints(Img,contours[idxgrandson]);
                                //////////   compute circle centroid ///////////
                                cv::drawContours( Background, contours, idxgrandson, 255, CW, 8, hierarchy,0);  //CV_FILLED   //////////////////
                                //cv::imshow("Background",Background);
                                //cv::waitKey(0);
                                SubEdge = EdgeBold.mul(Background);
                                Background.setTo(0);
                                //cv::imshow("SubEdge",SubEdge);
                                //cv::waitKey(0);
                                cv::findContours(SubEdge, SubContours, SubHierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

                                int idx0 = 0;
                                int idx1 = 1;
                                if (SubContours.size()>2)
                                {
                                    //SubCA.clear();
                                    int ise = 0;
                                    int isemax=0;
                                    double area = -1;
                                    for (; ise >= 0; ise = SubHierarchy[ise][0])
                                    {
                                        if (contourArea(SubContours[ise])>area)
                                        {
                                            area = contourArea(SubContours[ise]);
                                            isemax = ise;
                                        }
                                    }
                                    //double A;
                                    //cv::minMaxIdx(SubCA, NULL, &A, NULL, CircleOutlierIdx);
                                    idx0 = isemax;
                                    //DrawPoints(Img,SubContours[idx0]);
                                    int idx11 = SubHierarchy[idx0][2];
                                    double SonArea=-1;
                                    for ( ; idx11>=0;idx11 = SubHierarchy[idx11][0])
                                    {
                                        //DrawPoints(Img,SubContours[idx11]);
                                        if (contourArea(SubContours[idx11])>SonArea)
                                        {
                                            SonArea = contourArea(SubContours[idx11]);
                                            idx1 = idx11;
                                        }
                                    }
                                }
                                if (SubContours.size()<2)
                                {
                                    //std::cout<<"continue A"<<std::endl;
                                    continue;
                                }

                                cv::Point2f CircleCentroid0;
                                cv::Point2f CircleCentroid1;
                                ComputeContourCentroid(SubContours[idx0],CircleCentroid0);
                                ComputeContourCentroid(SubContours[idx1],CircleCentroid1);
                                cv::Point2f CircleCentroid = (CircleCentroid0+CircleCentroid1)*0.5;
                                if ((int(CircleCentroid.x)<=10)||(int(CircleCentroid.x)>=(Img.cols-10))||(int(CircleCentroid.y)<=10)||(int(CircleCentroid.y)>=(Img.rows-10)))
                                {
                                   // std::cout<<"continue B"<<std::endl;
                                   // std::cout<<CircleCentroid<<std::endl;
                                    continue;
                                }
                                CircleCoordinate.emplace_back(CircleCentroid);
                                CountCircleNum++;
                                //std::cout<<CountCircleNum<<std::endl;
                                if (CountCircleNum == this->PointsNum)
                                {
                                    //std::cout << "Find Calibration Board!!!" << std::endl;
                                    FindBoard = true;

                                    break;
                                }
                            }
                        }
                        if (FindBoard == true)
                        {
                            break;
                        }
                    }
                }
                if (FindBoard == true)
                {
                    break;
                }
            }
        }

        if (FindBoard == true)
        {
            break;
        }
    }


    UsedTh = Th;
    return FindBoard;
}


bool CalibrationBoard::FindUnSymmetricCorner(cv::vector<cv::Point2f> FourCornerPointsUnSort,cv::vector<cv::Point> FittingCurve,cv::Point2f& CornerCentroid)
{
    cv::Point2f FitPointSelect;
    cv::Point2f CornerPointSelect;
    float Dist = 10e8;
    cv::Point2f Diff;


    for (int i=0;i<FittingCurve.size();i++)
    {
        FitPointSelect = FittingCurve[i];
        for (int j=0;j<FourCornerPointsUnSort.size();j++)
        {
            CornerPointSelect = FourCornerPointsUnSort[j];
            Diff = cv::Point2f(FitPointSelect.x - CornerPointSelect.x,
                               FitPointSelect.y - CornerPointSelect.y);
            if(sqrtf(Diff.x * Diff.x + Diff.y * Diff.y)<Dist)
            {
                Dist = sqrtf(Diff.x * Diff.x + Diff.y * Diff.y);
                CornerCentroid = FitPointSelect;
            }

        }
    }
}


bool CalibrationBoard::FindFourCornerPoints(cv::Mat Img,cv::vector<cv::vector<cv::Point>> contours,int idxson,cv::vector<cv::Point2f> CircleCoordinate,cv::vector<cv::Point2f>& FourCornerPointsUnSort,bool FivePointJudge,cv::Point2f& BoardMarker)
{
    /////////////  Use approxPolyDP to find out four corners feature points of an outliner contour /////////
    ///////////// At the meantime find out the nearest circle centroid of the corners ///////////

    cv::vector<cv::Point> InnerContour = contours[idxson];
    cv::vector<cv::Point> FittingCurve;
    cv::vector<cv::Point2f> FivePointsSort;
    cv::vector<cv::Point2f> FittingFourPoints;
    //cv::vector<cv::Point> FittingCurve;
    cv::vector<float> CornerCircleDist;
    int DistMinIndex[2];
    cv::Point2f Diff;
    FourCornerPointsUnSort.clear();
    bool FindBoard = false;
    for (int eps=0;eps<=Img.rows;eps++)
    {
        cv::approxPolyDP(InnerContour, FittingCurve, eps, true);
        if (FittingCurve.size() == 5)
        {
            //////  combine 5 points into 4 points ///////
            CombineFivePointsIntoFourPoints(FittingCurve,FittingFourPoints,FivePointsSort,BoardMarker);
            //DrawPoints(Img,FittingCurve);
            ///////////  judge if the FittingCurve is wrong, this is function is only design for rectified calibration board ////
            if (FivePointJudge)
            {
                FindBoard = JudgeFivePosition(FivePointsSort,BoardMarker,Img.cols,Img.rows);
                if (!FindBoard)
                {return FindBoard;}
            }
            //DrawPoints(Img, FittingCurve);
            //DrawPoints(Img, FittingFourPoints);
            //DrawPoints(Img, FittingFourPoints[0]);
            FindBoard = true;
            for (int fp = 0; fp < 4; fp++)
            {
                //std::cout<<fp<<std::endl;
                //std::cout<<FittingCurve[fp]<<std::endl;
                for (int cp = 0; cp < PointsNum; cp++)
                {
                    Diff = cv::Point2f(FittingFourPoints[fp].x - CircleCoordinate[cp].x,
                                       FittingFourPoints[fp].y - CircleCoordinate[cp].y);
                    //std::cout<<Diff<<std::endl;
                    CornerCircleDist.push_back(sqrtf(Diff.x * Diff.x + Diff.y * Diff.y));
                }
                cv::minMaxIdx(CornerCircleDist, NULL, NULL, DistMinIndex, NULL);
                //std::cout << DistMinIndex[1] << std::endl;
                FourCornerPointsUnSort.push_back(CircleCoordinate[DistMinIndex[1]]);
                CornerCircleDist.clear();
            }
            break;
        }
    }
    return FindBoard;
}


void CalibrationBoard::SortFourCornerPoints(cv::Point2f CornerCentroid, cv::vector<cv::Point2f> FourCornerPointsUnSort,cv::vector<cv::Point2f>& FourCornerPoints)
{
    //////////  Sort the above four circle centroid points //////////
    //////////  First find out Point 0  ////////
    FourCornerPoints.clear();
    int OriginalCornerIndex = 0;
    cv::Point2f Point0;
    cv::Point2f Point1;
    cv::Point2f Point2;
    cv::Point2f Point3;

    ComputeMinDistPoint(FourCornerPointsUnSort,CornerCentroid,Point0,OriginalCornerIndex);

    cv::vector<cv::Point2f> PointVector;
    cv::vector<cv::Point2f> PointUsedGroup;

    /////////  Find out the diagonal point of Point 0, namely Point3  ///////////

    for (int sp=0;sp<4;sp++)
    {
        if (sp==OriginalCornerIndex)    ////// Point 0, here total compute 3 vector
        {
            continue;
        }
        cv::Point2f VectorP0Pn = FourCornerPointsUnSort[sp]-Point0;
        double VectorNorm = sqrtf(VectorP0Pn.x*VectorP0Pn.x+VectorP0Pn.y*VectorP0Pn.y);
        VectorP0Pn = cv::Point2f(VectorP0Pn.x/VectorNorm,VectorP0Pn.y/VectorNorm);
        PointVector.push_back(VectorP0Pn);
        PointUsedGroup.push_back(FourCornerPointsUnSort[sp]);
    }
    //std::cout<<cv::Mat(PointVector)<<std::endl;
    //std::cout<<cv::Mat(PointUsedGroup)<<std::endl;

    /////////  compute vector cross product,judge if the angle between vector is clockwise or anticlockwise  ///////
    cv::vector<double> CrossPoductAll;
    cv::vector<cv::Point2f> PointUsedGroup1;

    for (int sp =0;sp<3;sp++)
    {
        cv::Point2f Vector1 = PointVector[sp];
        for (int sp1 = 0; sp1 < 3; sp1++)
        {
            if(sp1==sp)
            {
                continue;
            }
            cv::Point2f Vector2 = PointVector[sp1];
            CrossPoductAll.push_back(Vector1.cross(Vector2));
            PointUsedGroup1.push_back(PointUsedGroup[sp1]);
        }
        //std::cout<<cv::Mat(CrossPoductAll)<<std::endl;
        if(CrossPoductAll[0]*CrossPoductAll[1]<0)
        {
            Point2 = PointUsedGroup[sp];
            if (CrossPoductAll[0]<0)
            {
                Point1 = PointUsedGroup1[0];
                Point3 = PointUsedGroup1[1];
            }
            else
            {
                Point1 = PointUsedGroup1[1];
                Point3 = PointUsedGroup1[0];
            }
            break;
        }
        CrossPoductAll.clear();
        PointUsedGroup1.clear();
    }

    //std::cout<<Point0<<std::endl;
    //std::cout<<Point1<<std::endl;
    //std::cout<<Point2<<std::endl;
    //std::cout<<Point3<<std::endl;
    FourCornerPoints.push_back(Point0);
    FourCornerPoints.push_back(Point1);
    FourCornerPoints.push_back(Point2);
    FourCornerPoints.push_back(Point3);
}


void CalibrationBoard::BoardRectification(cv::Mat Img,cv::vector<cv::Point2f> FourCornerPoints,cv::Mat& HomoMatrix,cv::Mat& ImgHomo)
{
    //////// four corresponding points have been found, then do image rectification ////////

    cv::vector<cv::Point2f> FourCornerIdealPoints;
    float CornerPointsDist = CircleDistHeight*(PointsRows-1);
    float SquareDimension = Img.rows*(CornerPointsDist/BoardHeight)*0.98;
    cv::Point2f IdealPoint0 = cv::Point2f((Img.cols-SquareDimension)/2,(Img.rows-SquareDimension)/2);
    cv::Point2f IdealPoint1 = cv::Point2f((Img.cols-SquareDimension)/2+SquareDimension,(Img.rows-SquareDimension)/2);
    cv::Point2f IdealPoint2 = cv::Point2f((Img.cols-SquareDimension)/2+SquareDimension,(Img.rows-SquareDimension)/2+SquareDimension);
    cv::Point2f IdealPoint3 = cv::Point2f((Img.cols-SquareDimension)/2,(Img.rows-SquareDimension)/2+SquareDimension);
    FourCornerIdealPoints.push_back(IdealPoint0);
    FourCornerIdealPoints.push_back(IdealPoint1);
    FourCornerIdealPoints.push_back(IdealPoint2);
    FourCornerIdealPoints.push_back(IdealPoint3);

    //std::cout<<cv::Mat(FourCornerPoints)<<std::endl;
    //std::cout<<cv::Mat(FourCornerIdealPoints)<<std::endl;

    //cv::Mat ImgRectify;
    HomoMatrix = cv::findHomography(FourCornerPoints,FourCornerIdealPoints);
    cv::warpPerspective(Img,ImgHomo,HomoMatrix,cv::Size(Img.cols,Img.rows));
    ///// Apply Bilateral Filter /////

}


void CalibrationBoard::SortCirclePoints(cv::Mat Img,cv::Mat HomoMatrix,cv::vector<cv::Point2f> CircleCoordinate,cv::vector<cv::Point2f>& CirclePointsSort)
{
    CirclePointsSort.clear();
    cv::Point2f PointCompared;
    cv::Point2f PointMinDist;
    int MinIndex;
    float CornerPointsDist = CircleDistHeight*(PointsRows-1);
    float SquareDimension = Img.rows*(CornerPointsDist/BoardHeight)*0.98;
    float ExpandPara = SquareDimension/(CircleDistHeight*(PointsRows-1));
    cv::Point2f IdealPoint0 = cv::Point2f((Img.cols-SquareDimension)/2,(Img.rows-SquareDimension)/2);
    //DrawPoints(Img,CircleCoordinate);
    for (int i=0;i<ObjVector.size();i++)
    {
        PointCompared = cv::Point2f(ObjVector[i].x*ExpandPara+IdealPoint0.x,ObjVector[i].y*ExpandPara+IdealPoint0.y);
        //DrawPoints(Img,PointCompared);
        ComputeMinDistPoint(CircleCoordinate,PointCompared,PointMinDist,MinIndex);
        CirclePointsSort.push_back(PointMinDist);
        CircleCoordinate.erase(CircleCoordinate.begin()+MinIndex);
    }
    cv::perspectiveTransform(CirclePointsSort,CirclePointsSort,HomoMatrix.inv());
}



////////////////////   sub function  //////////////
void CalibrationBoard::CombineFivePointsIntoFourPoints(cv::vector<cv::Point> FivePoints,cv::vector<cv::Point2f>& FourPoints,cv::vector<cv::Point2f>& FivePointsSort,cv::Point2f& BoardMarker)
{
    cv::Point2f ComparePoint1;
    cv::Point2f ComparePoint2;
    FourPoints.clear();


    cv::Point2f Diff;
    float Dist = 10e8;
    float Dist0;
    int index0=0;
    int index1=0;
    for (int i=0;i<FivePoints.size();i++)
    {
        for (int j=i+1;j<FivePoints.size();j++)
        {
            ComparePoint1 = FivePoints[i];
            ComparePoint2 = FivePoints[j];
            Diff = ComparePoint1-ComparePoint2;
            Dist0 = sqrtf(Diff.x * Diff.x + Diff.y * Diff.y);
            if(Dist0<Dist)
            {
                Dist = Dist0;
                index0 = i;
                index1 = j;
            }
        }
    }

    //////////   sort for five points  //////////
    //std::cout<<FivePoints[index0]<<std::endl;
    //std::cout<<FivePoints[index1]<<std::endl;

////////////////////////////////////////////////////////////////////////

    cv::Point2f Point00 = FivePoints[index0]+FivePoints[index1];
    Point00 = cv::Point2f(Point00.x/2,Point00.y/2);
    FourPoints.push_back(Point00);

    int index3;

    for (int i=0;i<FivePoints.size();i++)
    {
        if ((i== index0)|(i== index1))
        {
            continue;
        }
        FourPoints.push_back(FivePoints[i]);
    }


    cv::vector<cv::Point2f> FourPointSort;
    BoardMarker = FourPoints[0];
    SortFourCornerPoints(BoardMarker,FourPoints,FourPointSort);
    cv::Point2f Point0;
    cv::Point2f Point1;
    cv::Point2f Point2;
    cv::Point2f Point3;
    cv::Point2f Point4;

    Point2 = FourPointSort[1];
    Point3 = FourPointSort[2];
    Point4 = FourPointSort[3];

    double PointDist1 = sqrt((FivePoints[index0].x-Point2.x)*(FivePoints[index0].x-Point2.x)+(FivePoints[index0].y-Point2.y)*(FivePoints[index0].y-Point2.y));
    double PointDist2 = sqrt((FivePoints[index1].x-Point2.x)*(FivePoints[index1].x-Point2.x)+(FivePoints[index1].y-Point2.y)*(FivePoints[index1].y-Point2.y));
    if (PointDist1>PointDist2)
    {
        Point0 = FivePoints[index0];
        Point1 = FivePoints[index1];
    }
    else
    {
        Point0 = FivePoints[index1];
        Point1 = FivePoints[index0];
    }

    FivePointsSort.clear();
    FivePointsSort.push_back(Point0);
    FivePointsSort.push_back(Point1);
    FivePointsSort.push_back(Point2);
    FivePointsSort.push_back(Point3);
    FivePointsSort.push_back(Point4);

}