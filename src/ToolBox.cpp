//
// Created by samuel on 17-8-10.
//

#include "ToolBox.h"

void DrawPoints(cv::Mat Img,cv::vector<cv::Point> Points)
{
    cv::Mat frameClr;
    if (Img.channels()==1)
    {
        cv::cvtColor(Img, frameClr, CV_GRAY2BGR);
    }
    else
    {
        Img.copyTo(frameClr);
    }
    for (int i=0;i<Points.size();i++)
    {
        cv::circle(frameClr,Points[i],1,cv::Scalar(0,0,255),3,8,0);
    }
    cv::namedWindow("Draw Points",cv::WINDOW_NORMAL);
    cv::imshow("Draw Points",frameClr);
    cv::waitKey(0);
   // cv::destroyWindow("Draw Points");
}

void DrawPoints(cv::Mat Img,cv::vector<cv::Point2f> Points)
{
    cv::Mat frameClr;
    if (Img.channels()==1)
    {
        cv::cvtColor(Img, frameClr, CV_GRAY2BGR);
    }
    else
    {
        Img.copyTo(frameClr);
    }

    for (int i=0;i<Points.size();i++)
    {
        cv::circle(frameClr,Points[i],1,cv::Scalar(0,0,255),3,8,0);
    }
    cv::namedWindow("Draw Points",cv::WINDOW_NORMAL);
    cv::imshow("Draw Points",frameClr);
    cv::waitKey(0);
}

void DrawPoints(cv::Mat Img,cv::Point2f Points)
{
    cv::Mat frameClr;
    if (Img.channels()==1)
    {
        cv::cvtColor(Img, frameClr, CV_GRAY2BGR);
    }
    else
    {
        Img.copyTo(frameClr);
    }
    cv::circle(frameClr,Points,1,cv::Scalar(0,0,255),3,8,0);
    cv::namedWindow("Draw Points",cv::WINDOW_NORMAL);
    cv::imshow("Draw Points",frameClr);
    cv::waitKey(0);
}





void ComputeContourCentroid(cv::vector<cv::Point> ContourPoints,cv::Point2f& ContourCentroid)
{
    cv::Moments Mom0;
    Mom0 = cv::moments(ContourPoints,false);
    ContourCentroid = cv::Point2f(Mom0.m10/Mom0.m00,Mom0.m01/Mom0.m00);
}

void ComputePointsCenter(cv::vector<cv::Point> Points,cv::Point2f& PointCenter)
{
    cv::Point2f PointCenterSum(0,0);
    for (int i=0;i<Points.size();i++)
    {
        PointCenterSum = cv::Point2f(PointCenterSum.x+Points[i].x,PointCenterSum.y+Points[i].y);
    }
    PointCenter = cv::Point2f(PointCenterSum.x/Points.size(),PointCenterSum.y/Points.size());

}

void ComputeImgBoldEdge(cv::Mat Img,cv::Mat& EdgeBoldU8)
{
    cv::Mat ImgGray;
    if (Img.channels()==3)
    {
        cv::cvtColor(Img,ImgGray, cv::COLOR_RGB2GRAY);
    }
    else
    {
        ImgGray = Img;
    }

    /////// Compute Bold Edge /////
    cv::Mat SobelX;
    cv::Mat SobelY;
    cv::Mat SobelZ;
    cv::Mat SobelW;
    cv::Mat EdgeBold;
    cv::Mat KernelZ(3,3,CV_32FC1,cv::Scalar(0.0f));
    cv::Mat KernelW(3,3,CV_32FC1,cv::Scalar(0.0f));

    KernelZ.ptr<float>(0)[0] = 2;
    KernelZ.ptr<float>(0)[1] = 1;
    KernelZ.ptr<float>(1)[0] = 1;
    KernelZ.ptr<float>(1)[2] = -1;
    KernelZ.ptr<float>(2)[1] = -1;
    KernelZ.ptr<float>(2)[2] = -2;

    KernelW.ptr<float>(0)[1] = 1;
    KernelW.ptr<float>(0)[2] = 2;
    KernelW.ptr<float>(1)[0] = -1;
    KernelW.ptr<float>(1)[2] = 1;
    KernelW.ptr<float>(2)[0] = -2;
    KernelW.ptr<float>(2)[1] = -1;

    Sobel(ImgGray, SobelX,CV_32FC1,1,0);
    Sobel(ImgGray, SobelY,CV_32FC1,0,1);

    cv::filter2D(ImgGray,SobelZ,CV_32FC1,KernelZ);
    cv::filter2D(ImgGray,SobelW,CV_32FC1,KernelW);

    EdgeBold = cv::abs(SobelX)+cv::abs(SobelY)+cv::abs(SobelZ)+cv::abs(SobelW);
    //cv::imshow("EdgeBold",EdgeBold);
    //cv::waitKey(0);
    //cv::Mat EdgeBoldU8;
    EdgeBold.convertTo(EdgeBoldU8,CV_8UC1);
    cv::threshold(EdgeBoldU8, EdgeBoldU8, 0, 255, CV_THRESH_OTSU);    /// CV_THRESH_OTSU is work only only for 8 bit images
}


void ComputeMinDistPoint(cv::vector<cv::Point2f> PointsGroup,cv::Point2f ComparedPoint,cv::Point2f& PointMinDist,int& MinIndex)
{
    double Distance;
    double DistanceCompare = 10e8;
    cv::Point2f Diff;
    for (int i=0;i<PointsGroup.size();i++)
    {
        Diff = ComparedPoint-PointsGroup[i];
        Distance = sqrtf(Diff.x * Diff.x + Diff.y * Diff.y);
        if (Distance<DistanceCompare)
        {
            DistanceCompare = Distance;
            MinIndex = i;
        }
    }
    PointMinDist = PointsGroup[MinIndex];
}


void saveMat(cv::Mat img, std::string folderPath, int imgNo, std::string suffix)
{
    //char u[3];
    //u[0] = imgNo / 10 + 48;
    //u[1] = imgNo % 10 + 48;
    //u[2] = '\0';
    std::stringstream ss;
    std::string str;
    ss<<imgNo;
    ss>>str;
    std::string fullPath = folderPath+str + suffix;
    cv::imwrite(fullPath, img);
    return;
}


void loadMat(cv::Mat& img, std::string folderPath, int imgNo, std::string suffix)
{
    //char c[3];
    //c[0] = imgNo/10 + 48;
    //c[1] = imgNo%10 + 48;
    //c[2] = '\0';
    //std::string s(c);      // 48 ASCII code offset
    std::stringstream ss;
    std::string str;
    ss<<imgNo;
    ss>>str;
    std::string path = folderPath+str + suffix;
    //std::cout<<path<<std::endl;
    img = cv::imread(path,-1);     ////////////
    return;
}

void GetScreenResolution(int& w,int& h)
{
    int fd;
    struct fb_var_screeninfo fb_var;

    fd = open("/dev/fb0",O_RDWR);
//get screen information
    ioctl (fd,FBIOGET_VSCREENINFO,&fb_var);
    w = fb_var.xres;
    h = fb_var.yres;
}

void CreateImgWindow(std::string WindowName,cv::Size ImgSize,int mode,double para)
{
    int ScreenResolutionW, ScreenResolutionH;
    GetScreenResolution(ScreenResolutionW, ScreenResolutionH);
    int WindowH = float(ScreenResolutionH) * para;
    int WindowW = (float(ImgSize.width) / float(ImgSize.height)) * float(WindowH);
    if (mode ==1)           ///////  set the image window close to the right side of computer screen  /////
    {
        int MoveH = (ScreenResolutionH - WindowH) / 2;
        int MoveW = ScreenResolutionW - WindowW;
        //std::cout << WindowH << " " << WindowW << std::endl;
        cv::namedWindow(WindowName, CV_WINDOW_NORMAL);
        cv::resizeWindow(WindowName, WindowW, WindowH);
        cv::moveWindow(WindowName, MoveW, MoveH);
        //cv::imshow(WindowName, Img);
    }
    if (mode ==2)           ///////  set the image window close to the left side of computer screen  /////
    {
        int MoveH = (ScreenResolutionH - WindowH) / 2;
        int MoveW = 0;
        //std::cout << WindowH << " " << WindowW << std::endl;
        cv::namedWindow(WindowName, CV_WINDOW_NORMAL);
        cv::resizeWindow(WindowName, WindowW, WindowH);
        cv::moveWindow(WindowName, MoveW, MoveH);
        //cv::imshow(WindowName, Img);
    }
}

void CreateImgWindow(std::string WindowNameA,cv::Size ImgSizeA,std::string WindowNameB,cv::Size ImgSizeB,double para)
{
    int ScreenResolutionW, ScreenResolutionH;
    GetScreenResolution(ScreenResolutionW, ScreenResolutionH);
    int WindowW = (double(ScreenResolutionW)/2)*para;
    int WindowH = (float(ImgSizeA.height) / float(ImgSizeA.width)) * float(WindowW);
    int MoveHA = (ScreenResolutionH - WindowH) / 2;
    int MoveWA = 0;
    int MoveHB = (ScreenResolutionH - WindowH) / 2;
    int MoveWB = ScreenResolutionW - WindowW;
    ////// CamA ////
    cv::namedWindow(WindowNameA, CV_WINDOW_NORMAL);
    cv::resizeWindow(WindowNameA, WindowW, WindowH);
    cv::moveWindow(WindowNameA, MoveWA, MoveHA);
    //cv::imshow(WindowNameA, ImgA);
    ////// CamB ////
    cv::namedWindow(WindowNameB, CV_WINDOW_NORMAL);
    cv::resizeWindow(WindowNameB, WindowW, WindowH);
    cv::moveWindow(WindowNameB, MoveWB, MoveHB);
    //cv::imshow(WindowNameB, ImgB);

}

void CreateImgWindow(std::string WindowNameA,cv::Size ImgSizeA,std::string WindowNameB,cv::Size ImgSizeB,std::string WindowNameC,cv::Size ImgSizeC,double para)
{
    int ScreenResolutionW, ScreenResolutionH;
    GetScreenResolution(ScreenResolutionW, ScreenResolutionH);
    ///// Img C /////
    int WindowHC = double(ScreenResolutionH)*0.9;
    int WindowWC = (float(ImgSizeC.width) / float(ImgSizeC.height))*WindowHC;

    ///// Img A /////
    int WindowHA = (double(ScreenResolutionH)/2)*0.85;;
    int WindowWA = (float(ImgSizeA.width) / float(ImgSizeA.height))*WindowHA;
    int MoveHA = 0;
    int MoveWA = 0;
    cv::namedWindow(WindowNameA, CV_WINDOW_NORMAL);
    cv::resizeWindow(WindowNameA, WindowWA, WindowHA);
    cv::moveWindow(WindowNameA, MoveWA, MoveHA);

    ////// ImgB //////
    int WindowHB = (double(ScreenResolutionH)/2)*0.85;;
    int WindowWB = (float(ImgSizeB.width) / float(ImgSizeB.height))*WindowHB;
    int MoveHB = WindowHA+60;
    int MoveWB = 0;
    cv::namedWindow(WindowNameB, CV_WINDOW_NORMAL);
    cv::resizeWindow(WindowNameB, WindowWB, WindowHB);
    cv::moveWindow(WindowNameB, MoveWB, MoveHB);

    //// ImgC //////
    int MoveHC = 0;
    int MoveWC = WindowWA+70;
    cv::namedWindow(WindowNameC, CV_WINDOW_NORMAL);
    cv::resizeWindow(WindowNameC, WindowWC, WindowHC);
    cv::moveWindow(WindowNameC, MoveWC, MoveHC);

}

bool JudgeFivePosition(cv::vector<cv::Point2f>FivePointsSort,cv::Point2f BoardMarker,int ImgWidth,int ImgHeight)
{
    bool Flag =true;
    int DistTolerance = ImgHeight/30;
    //std::cout<<cv::Mat(FivePointsSort)<<std::endl;
    if (((BoardMarker.x<ImgWidth/2)&(BoardMarker.y<ImgHeight/2))|((BoardMarker.x>ImgWidth/2)&(BoardMarker.y>ImgHeight/2)))
    {
        if ((abs(FivePointsSort[0].x - FivePointsSort[4].x) > DistTolerance) |
            (abs(FivePointsSort[1].y - FivePointsSort[2].y) > DistTolerance) |
            (abs(FivePointsSort[2].x - FivePointsSort[3].x) > DistTolerance) |
            (abs(FivePointsSort[3].y - FivePointsSort[4].y) > DistTolerance))
        {
            Flag = false;
        }
    }
    if (((BoardMarker.x>ImgWidth/2)&(BoardMarker.y<ImgHeight/2))|((BoardMarker.x<ImgWidth/2)&(BoardMarker.y>ImgHeight/2)))
    {
        if ((abs(FivePointsSort[0].y - FivePointsSort[4].y) > DistTolerance) |
            (abs(FivePointsSort[1].x - FivePointsSort[2].x) > DistTolerance) |
            (abs(FivePointsSort[2].y - FivePointsSort[3].y) > DistTolerance) |
            (abs(FivePointsSort[3].x - FivePointsSort[4].x) > DistTolerance))
        {
            Flag = false;
        }
    }
    return Flag;
}


void saveXmlFile(cv::Mat Matrix,std::string Name,std::string Format,std::string savePath)
{
    std::string name;
    name = savePath + Name + Format;
    cv::FileStorage fs_in(name, cv::FileStorage::WRITE);
    if(!fs_in.isOpened())
    {std::cout<<"Write Matrix Error!!!"<<std::endl;}
    fs_in<<Name<<Matrix;
    fs_in.release();

    return;
}

void loadXmlFile(cv::Mat& Matrix,std::string Name,std::string Format,std::string path)
{
    std::string str = path + Name + Format;
//	std::cout<<str<<std::endl;
    cv::FileStorage fs(str, cv::FileStorage::READ);
    if(!fs.isOpened())
    {std::cout<<"Open File Error!!!!"<<std::endl;}
    fs[Name]>>Matrix;
    //std::cout<<Matrix<<std::endl;
    fs.release();
    return;
}

void saveXYZ(std::string filenameString,const cv::Mat& mat)
{
    const char* filename = filenameString.c_str();
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    fprintf(fp,"#VRML V2.0 utf8\n"
            "Shape {\n"
            "geometry PointSet {\n"
            "coord Coordinate {\n"
            "point [");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            cv::Vec3f point = mat.at<cv::Vec3f>(y, x);
            //std::cout<<cv::Mat(point)<<std::endl;
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fprintf(fp,"]\n"
            "}\n"
            "}\n"
            "}");
    fclose(fp);
}