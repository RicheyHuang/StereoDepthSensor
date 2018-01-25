//
// Created by samuel on 17-8-10.
//

#include "CamSys.h"


CamSys::CamSys(Camera* cam,char* CamName,int width,int height,int ExposureTime, int MaxExposure, std::string ImgPath,std::string ImgFormat):
        cam(cam),CamName(CamName),width(width),height(height),ExposureTime(ExposureTime), MaxExposureTime(MaxExposure), ImgPath(ImgPath),ImgFormat(ImgFormat)
{

}

void CamSys::Initialize()
{
    std::cout<<"Initializing Camera "<<CamName<<std::endl;
    CamHandle = cam->Init(CamName);
    cam->Play(CamHandle);
    cam->SetImageResolution(CamHandle, width, height);
    cam->SetAeState(CamHandle, false);
    cam->SetExposureTime(CamHandle, ExposureTime);
    cam->SetTriggerMode(CamHandle,1);
};

void CamSys::Trigger()
{
    cam->SoftTrigger(CamHandle);
}

void CamSys::CapImage(UINT TimeOut)
{
    frame = cam->getImage(CamHandle, TimeOut);
}

void CamSys::PreviewImage()
{
    cv::namedWindow(std::string(CamName)+" Preview",cv::WINDOW_NORMAL);
    while(true)
    {
        Trigger();
        CapImage();
        if (frame.data != NULL)
        {
            cv::imshow(std::string(CamName)+" Preview", frame);
        }
        else
        {
            std::cout<<"img is None"<<std::endl;
        }
        if ((cv::waitKey(30) & 255) == 27)                 ///// esc
        {
            break;
        }
    }
}

void CamSys::SaveFrame(std::string ImgName)
{
    cv::imwrite(ImgPath + ImgName + ImgFormat, frame);
}


void CamSys::CalibrateCamera(CalibrationBoard CircleBoard,int CaliImgNum,bool CaputureImgs)
{
    /////// image show windows information ///////
    std::string CamNameSave = std::string(CamName);
    std::string CaptureWindow = "Camera "+CamNameSave+" Captured Image";
    std::string DetectWindow = "Camera "+CamNameSave+" Find Calibration Board";

    //cv::Mat BlackImg = cv::Mat::zeros(height, width, CV_8UC1);

    CreateImgWindow(CaptureWindow,cv::Size(width,height),2,0.43);
    CreateImgWindow(DetectWindow,cv::Size(width,height));

    bool BoardFound = false;
    int success=0;
    std::vector<cv::Point2f> Corners;
    cv::Point2f BoardMarker;

    std::vector<std::vector<cv::Point2f>> ImgVectorAll;
    std::vector<std::vector<cv::Point3f>> ObjVectorAll;
    //std::vector<cv::Point2f> ImgVector;
    CircleBoard.GetObjVec();
    //std::cout<<cv::Mat(CircleBoard.ObjVector)<<std::endl;
    cv::Mat frameClr;
    int key=0;
    //CreateImgWindow("Camera "+CamNameSave+" Calibrate Images",BlackImg);

    if (CaputureImgs==true)
    {
        std::cout<<"Press 'Esc' to stop capturing image"<<std::endl;
        std::cout<<"Press 'c' to capture one image for calibration"<<std::endl;

        while (success<CaliImgNum)
        {
            Trigger();
            CapImage();
            cv::imwrite(ImgPath+"CanNotDetectImg"+ImgFormat,frame);     // for debug
            if (frame.data != NULL)
            {
                cv::imshow(CaptureWindow,frame);
                key = cv::waitKey(10);
            }
            else
            {
                std::cout<<"img is None"<<std::endl;
            }
            if ((key & 255) == 27 )        ///// Esc
            {
                std::cout<<"Stop Capturing Images"<<std::endl;
                break;
            }

            if ((key & 255) == 99)      /////  c
            {
                std::cout<<"Start detect Camera "+CamNameSave+ " Calibration Board No."<<success<<std::endl;
                BoardFound=CircleBoard.ExtractBoardCoordinate(frame,BoardMarker,Corners);
                if (BoardFound==false)
                {
                    std::cout<<CamNameSave+" Image No."<<success<<" can not find a calibration board!!!!!"<<std::endl;
                }
                else
                {
                    if (frame.channels()==1)
                    {
                        cv::cvtColor(frame, frameClr, CV_GRAY2BGR);
                    }
                    else
                    {
                        frame.copyTo(frameClr);
                    }
                    cv::drawChessboardCorners(frameClr, CircleBoard.BoardSize, Corners,BoardFound);
                    cv::circle(frameClr,BoardMarker,1,cv::Scalar(0,0,255),3,8,0);
                    cv::imshow(DetectWindow,frameClr);
                    std::cout<<"If the detection is right,input 'Enter' to save data,else input other key"<<std::endl;
                    key = cv::waitKey(0);
                    if (((key&255) == 10)||((key&255) == 141))            /////////////////// Enter
                    {
                        saveMat(frame,ImgPath+CamName+"_",success,ImgFormat);  //+"CalibrateImages_"
                        saveMat(frameClr,ImgPath+CamNameSave+"_Draw_",success,ImgFormat);
                        std::cout<<"Saving this image,press 'c' to capture another one..."<<std::endl;
                        ImgVectorAll.push_back(Corners);
                        ObjVectorAll.push_back(CircleBoard.ObjVector);
                        success++;
                    }
                }

            }
        }
    }
    else
    {
        //cv::waitKey(0);
        for (;success<CaliImgNum;success++)
        {
            std::cout<<"Start detect Camera "+CamNameSave+" Calibration Board No."<<success<<std::endl;
            loadMat(frame,ImgPath+CamNameSave+"_",success,ImgFormat);//+"CalibrateImages_"
            //cv::imshow("Calibration Board points",frame);
            cv::imshow(CaptureWindow,frame);
            cv::waitKey(500);

            ////
            BoardFound=CircleBoard.ExtractBoardCoordinate(frame,BoardMarker,Corners);
            if (BoardFound==false)
            {	std::cout<<CamNameSave+" Image No."<<success<<" can not find a calibration board!!!!!"<<std::endl;}
            else
            {
                if (frame.channels()==1)
                {
                    cv::cvtColor(frame, frameClr, CV_GRAY2BGR);
                }
                else
                {
                    frame.copyTo(frameClr);
                }
                cv::drawChessboardCorners(frameClr, CircleBoard.BoardSize, Corners,BoardFound);
                cv::circle(frameClr,BoardMarker,1,cv::Scalar(0,0,255),3,8,0);
                //cv::imshow("Calibration Board points detected",frameClr);
                cv::imshow(DetectWindow,frameClr);
                cv::waitKey(500);
                std::cout<<"Saving the coordinates..."<<std::endl;
                saveMat(frameClr,ImgPath+CamNameSave+"_Draw_",success,ImgFormat);
                ImgVectorAll.push_back(Corners);
                ObjVectorAll.push_back(CircleBoard.ObjVector);
            }
        }
    }

    cv::destroyWindow(CaptureWindow);
    cv::destroyWindow(DetectWindow);

    int CamCalibFlags = CV_CALIB_FIX_K3;
    //std::cout<<ObjVectorAll[0]<<std::endl;
    //std::cout<<ImgVectorAll[0]<<std::endl;
    //std::cout<<"before calib type: "<<this->intrinsicMatrix.type()<<std::endl;
    double CamCalibErr=cv::calibrateCamera(ObjVectorAll,ImgVectorAll,cv::Size(frame.cols,frame.rows),intrinsicMatrix,distortionMatrix,rvecs,tvecs,CamCalibFlags);

    ////
    //std::cout<<"before calib type: "<<this->intrinsicMatrix.type()<<std::endl;
    std::cout<<"camera "<<CamName<< " calibrated!"<<std::endl;
    std::cout << "done with RMS error=" << CamCalibErr << std::endl;
    //std::cout<<"Camera.intrinsicMatrix:"<<intrinsicMatrix<<std::endl;
    saveXmlFile(intrinsicMatrix,"IntrinsicMatrix"+CamNameSave,".xml",ImgPath);
    saveXmlFile(distortionMatrix,"DistortionMatrix"+CamNameSave,".xml",ImgPath);

}

void CamSys::LoadCamInfo()
{
    std::string CamNameSave = std::string(CamName);

    loadXmlFile(intrinsicMatrix,"IntrinsicMatrix"+CamNameSave,".xml",ImgPath);
    loadXmlFile(distortionMatrix,"DistortionMatrix"+CamNameSave,".xml",ImgPath);
}
