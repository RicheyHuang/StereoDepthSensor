//
// Created by root on 17-9-13.
//
#include "ShowImgGui.h"

static void OnMouse(int Event,int x,int y,int flags,void* param)
{

    //printf("( %d, %d) ",x,y);
    //printf("The Event is : %d ",Event);
    //printf("The flags is : %d ",flags);
    //printf("The param is : %d\n",param);
    //std::cout<<"Refpoint.........."<<std::endl;
    //std::cout<<RefPoint<<std::endl;
    cv::Point Select;
    cv::Point Movement;
    cv::Point RefPoint2;
    float ScaleFactor = 1.2;

    if ((Event==1)&&(flags==32))      ////  mouse left click down:stat moving image
    {
        //std::cout<<"Move Image Mode"<<std::endl;
        MoveMode0 = true;
        RefPoint = cv::Point(x, y)+StartPoint;
    }
    if ((Event==4)&&(flags==33))      ////  mouse left click up : stop moving image
    {
        MoveMode0 = false;
        RefPoint = cv::Point(0,0);
    }

    if (!MoveMode0)
    {
        if (((Event == 1) && (flags == 40)) || ((Event == 1) && (flags == 8)))       ////  Ctrl+Mouse Left Click : image magnifier zoom in
        {
            Select = (cv::Point(x, y) + StartPoint) * ScaleFactor;
            SizeResize = cv::Size(SizeResize.width * ScaleFactor, SizeResize.height * ScaleFactor);    ////  right
            StartPoint = cv::Point(Select.x - x, Select.y - y);          ///////
            if (StartPoint.x < 0)
            {
               // std::cout << "error1" << std::endl;
                StartPoint.x = 0;
            }
            if (StartPoint.y < 0)
            {
                //std::cout << "error2" << std::endl;
                StartPoint.y = 0;
            }
            if ((StartPoint.x + SizeOri.width) > SizeResize.width)
            {
                //std::cout << "error3" << std::endl;
                StartPoint.x = SizeResize.width - SizeOri.width;
            }
            if ((StartPoint.y + SizeOri.height) > SizeResize.height)
            {
                //std::cout << "error4" << std::endl;
                StartPoint.y = SizeResize.height - SizeOri.height;
            }
        }

        if (((Event == 2) && (flags == 40)) || ((Event == 2) && (flags == 8)))      ////  Ctrl+Mouse Right Click: zoom out
        {
            ScaleFactor = 1 / ScaleFactor;
            Select = (cv::Point(x, y) + StartPoint) * ScaleFactor;
            SizeResize = cv::Size(SizeResize.width * ScaleFactor, SizeResize.height * ScaleFactor);    ////  right
            if ((SizeResize.width < SizeOri.width) || (SizeResize.height < SizeOri.height))
            {
                SizeResize = SizeOri;
                StartPoint = cv::Point(0, 0);
            }
            else
            {
                StartPoint = cv::Point(Select.x - x, Select.y - y);          ///////
                if (StartPoint.x < 0)
                {
                   // std::cout << "error1" << std::endl;
                    StartPoint.x = 0;
                }
                if (StartPoint.y < 0)
                {
                    //std::cout << "error2" << std::endl;
                    StartPoint.y = 0;
                }
                if ((StartPoint.x + SizeOri.width) > SizeResize.width)
                {
                    //std::cout << "error3" << std::endl;
                    StartPoint.x = SizeResize.width - SizeOri.width;
                }
                if ((StartPoint.y + SizeOri.height) > SizeResize.height)
                {
                    //std::cout << "error4" << std::endl;
                    StartPoint.y = SizeResize.height - SizeOri.height;
                }
            }
        }
    }
    else     /////////////////   Move image mode //////////
    {
        float s = (float(SizeResize.width)/float(SizeOri.width));
        //std::cout<<s<<std::endl;
        //std::cout<<"......moving......."<<std::endl;
        RefPoint2 = cv::Point(x, y)+StartPoint;
        Movement = RefPoint2-RefPoint;
        //std::cout<<Move<<std::endl;
        //std::cout<<StartPointLast<<std::endl;
        StartPoint = StartPoint-Movement;
        //std::cout<<cv::Point(x, y)<<std::endl;
        if (StartPoint.x < 0)
        {
            //std::cout << "error1" << std::endl;
            StartPoint.x = 0;
        }
        if (StartPoint.y < 0)
        {
            //std::cout << "error2" << std::endl;
            StartPoint.y = 0;
        }
        if ((StartPoint.x + SizeOri.width) > SizeResize.width)
        {
            //std::cout << "error3" << std::endl;
            StartPoint.x = SizeResize.width - SizeOri.width;
        }
        if ((StartPoint.y + SizeOri.height) > SizeResize.height)
        {
            //std::cout << "error4" << std::endl;
            StartPoint.y = SizeResize.height - SizeOri.height;
        }
    }
    //std::cout<<"RefPoint"<<std::endl;
    //std::cout<<RefPoint<<std::endl;

    return void();
}


void ShowImgGui(std::string WinName,cv::Mat Img)
{
    //// load data ///
    //StartPoint = StartPointInput;
    //RefPoint = RefPointInput;
    //SizeResize = ImgShowSize;
    //MoveMode0 = MoveMode;
    const char*WinNameChar = WinName.c_str();
    ////////////////  resize image coordinate to last time scale ///////////////////


    SizeOri = Img.size();
    //MoveMode0 = MoveMode;

    if (SizeResize.width==0)
    {
        SizeResize = Img.size();
    }

    cv::resize(Img,Img,SizeResize,0,0,cv::INTER_AREA);

    cv::Mat ImgShow = Img(cv::Rect(StartPoint.x, StartPoint.y, SizeOri.width, SizeOri.height));
    cv::imshow(WinName, ImgShow);
    //cv::waitKey(1);
    cvSetMouseCallback(WinNameChar,OnMouse,NULL);

    //StartPointInput = StartPoint;
    //RefPointInput = RefPoint;
    //ImgShowSize = SizeResize;
    //MoveMode = MoveMode0;

}