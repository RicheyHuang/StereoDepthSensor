//
// Created by root on 17-8-23.
//

#include "StereoSystem.h"


StereoSystem::StereoSystem(CamSys* CamLeft,
                           CamSys* CamRight,
                           std::string ImgPath,
                           std::string ImgPathMatch,
                           std::string ImgFormat,
                           int MaxSADWindowSize,
                           int MaxValForMinDist,
                           int MaxWorkingRange,
                           int MaxTextureThreshold,
                           int MaxUniquenessRatio):
                           CamLeft(CamLeft),
                           CamRight(CamRight),
                           ImgPath(ImgPath),
                           ImgPathMatch(ImgPathMatch),
                           ImgFormat(ImgFormat),
                           ImgSize(cv::Size(CamLeft->width,CamLeft->height)),
                           MaxSADWindowSize(MaxSADWindowSize),
                           MaxValForMinDist(MaxValForMinDist),
                           MaxWorkingRange(MaxWorkingRange),
                           MaxTextureThreshold(MaxTextureThreshold),
                           MaxUniquenessRatio(MaxUniquenessRatio)
{

}

StereoSystem::StereoSystem() {}

void StereoSystem::onChangeLeftCam(int ExposureTime, void *ptr)
{
    StereoSystem *that = (StereoSystem*) ptr;
    that->AdjustLeftCamExposure(ExposureTime, 0);
}

void StereoSystem::onChangeRightCam(int ExposureTime, void *ptr)
{
    StereoSystem *that = (StereoSystem*) ptr;
    that->AdjustRightCamExposure(ExposureTime, 0);
}

void StereoSystem::AdjustLeftCamExposure(int UpdatedExposureTime, void *)
{
    CamLeft->cam->SetExposureTime(CamLeft->CamHandle, UpdatedExposureTime);
};

void StereoSystem::AdjustRightCamExposure(int UpdatedExposureTime, void *)
{
    CamRight->cam->SetExposureTime(CamRight->CamHandle, UpdatedExposureTime);
};

void StereoSystem::onChangeSADWinSize(int SADWindowsSize, void *ptr)
{
    StereoSystem *winsize = (StereoSystem*) ptr;
    winsize->UpdateSADWindowSize(SADWindowsSize, 0);
}

void StereoSystem::onChangeMinDist(int MinDist,void *ptr)
{
    StereoSystem *mindist = (StereoSystem*) ptr;
    mindist->UpdateMinDist(MinDist, 0);
}

void StereoSystem::onChangeWorkRange(int WorkingRange, void *ptr)
{
    StereoSystem *workrange = (StereoSystem*) ptr;
    workrange->UpdateWorkingRange(WorkingRange, 0);
}

void StereoSystem::onChangeTextureThresh(int TextureThresh, void *ptr)
{
    StereoSystem *texthresh = (StereoSystem*) ptr;
    texthresh->UpdateTextureThresh(TextureThresh, 0);
}

void StereoSystem::onChangeUniquenessRatio(int UniquenessRatio, void *ptr)
{
    StereoSystem *uniquenessratio = (StereoSystem*) ptr;
    uniquenessratio->UpdateUniquenessRatio(UniquenessRatio, 0);
}


void StereoSystem::UpdateSADWindowSize(int SADWindowSize, void*)
{
    /// The SADWindowSize should be odd number ///
    if (SADWindowSize % 2 == 0)
        SADWindowSize += 1;
    bm->setBlockSize(SADWindowSize);
}

void StereoSystem::UpdateTextureThresh(int TextureThresh, void *)
{
    bm->setTextureThreshold(TextureThresh);
}

void StereoSystem::UpdateUniquenessRatio(int UniquenessRatio, void *)
{
    bm->setUniquenessRatio(UniquenessRatio);
}

void StereoSystem::UpdateMinDist(int MinDist, void*)
{
    WorkingDistance[0] = MinDist;

    int disp1 = -PL.at<double>(0,3)/WorkingDistance[0];
    int disp2 = -PL.at<double>(0,3)/WorkingDistance[1];
    int xd = PL.at<double>(0,2)-PR.at<double>(0,2);
    int mindisp = disp1;
    bm->setMinDisparity(mindisp-xd);
    bm->setNumDisparities((int((abs(disp1-disp2)+15)/16))*16);
    DepthMapSize = cv::Size(ImgSize.width+bm->getMinDisparity(),ImgSize.height);

}

void StereoSystem::UpdateWorkingRange(int WorkingRange, void*)
{
    CamWorkingRange = WorkingRange;
    WorkingDistance[1] = CamWorkingRange + WorkingDistance[0];

    int disp1 = -PL.at<double>(0,3)/WorkingDistance[0];
    int disp2 = -PL.at<double>(0,3)/WorkingDistance[1];
    int xd = PL.at<double>(0,2)-PR.at<double>(0,2);
    int mindisp = disp1;
    bm->setMinDisparity(mindisp-xd);
    bm->setNumDisparities((int((abs(disp1-disp2)+15)/16))*16);
    DepthMapSize = cv::Size(ImgSize.width+bm->getMinDisparity(),ImgSize.height);

}


bool StereoSystem::SelectCamDirection()
{
    bool ChangeDirection = false;
    std::string HandleLeft;
    std::string HandleRight;
    HandleLeft = std::string(CamLeft->CamName);
    HandleRight = std::string(CamRight->CamName);

    CreateImgWindow(HandleLeft,ImgSize,HandleRight,cv::Size(CamRight->width,CamRight->height),0.95);
    std::cout<<"If the Left/Right position of the the images is wrong,Press s"<<std::endl;
    std::cout<<"If the Left/Right position of the the images is Right,Press ESC"<<std::endl;
    int key;
    CamSys* C;

    while(true)
    {
        CamLeft->Trigger();
        CamRight->Trigger();

        CamLeft->CapImage();
        CamRight->CapImage();

        cv::imshow(HandleLeft, CamLeft->frame);
        cv::imshow(HandleRight,CamRight->frame);
        key = cv::waitKey(1);
        if((key & 255) == 27 )                          ///// esc
        {
            break;
        }
        else if((key & 255) == 115)                     ////// s
        {
            ChangeDirection = !ChangeDirection;
            C = CamLeft;
            CamLeft = CamRight;
            CamRight = C;
            std::cout<<"If the Left/Right position of the the images is wrong,Press s"<<std::endl;
            std::cout<<"If the Left/Right position of the the images is Right,Press ESC"<<std::endl;
            HandleLeft = std::string(CamLeft->CamName);
            HandleRight = std::string(CamRight->CamName);
            CreateImgWindow(HandleLeft,ImgSize,HandleRight,cv::Size(CamRight->width,CamRight->height),0.95);
        }
    }
    cv::destroyWindow(HandleLeft);
    cv::destroyWindow(HandleRight);
    return ChangeDirection;
}


void StereoSystem::AdjustCameraFocus()
{
    std::string HandleLeft;
    std::string HandleRight;
    HandleLeft = std::string(CamLeft->CamName);
    HandleRight = std::string(CamRight->CamName);
    CreateImgWindow(HandleLeft, ImgSize, HandleRight, cv::Size(CamRight->width,CamRight->height), 0.95);

    cv::createTrackbar("ExposureTime：", HandleLeft,  &CamLeft->ExposureTime,  CamLeft->MaxExposureTime,  onChangeLeftCam,  this);
    cv::createTrackbar("ExposureTime：", HandleRight, &CamRight->ExposureTime, CamRight->MaxExposureTime, onChangeRightCam, this);

    int key;
    while(true)
    {
        CamLeft->Trigger();
        CamLeft->CapImage();
        ShowImgGui(HandleLeft, CamLeft->frame);
        key = cv::waitKey(1);
        if((key & 255) == 27 )                 ///// esc
        {
            break;
        }
    }

    while(true)
    {
        CamRight->Trigger();
        CamRight->CapImage();
        ShowImgGui(HandleRight, CamRight->frame);
        key = cv::waitKey(1);
        if((key & 255) == 27 )                 ///// esc
        {
            break;
        }
    }

    AdjustLeftCamExposure (CamLeft->ExposureTime,   0);
    AdjustRightCamExposure(CamRight->ExposureTime,  0);

    cv::destroyWindow(HandleLeft);
    cv::destroyWindow(HandleRight);
    return void();
}


void StereoSystem::StereoCalibration(CalibrationBoard Board,int CaliImgNum,bool CaputureImgs)
{
    std::string CamNameLeftSave = std::string(CamLeft->CamName);
    std::string CamNameRightSave = std::string(CamRight->CamName);

    std::string HandleLeft;
    std::string HandleRight;
    HandleLeft = std::string(CamLeft->CamName);
    HandleRight = std::string(CamRight->CamName);
    CreateImgWindow(HandleLeft,ImgSize,HandleRight,cv::Size(CamRight->width,CamRight->height),0.95);
    int key;
    bool BoardFound = false;
    int success=0;

    cv::Point2f BoardMarker;
    cv::Mat frameClrLeft;
    cv::Mat frameClrRight;
    std::vector<cv::Point2f> CamLeftCorners;
    std::vector<cv::Point2f> CamRightCorners;
    std::vector<std::vector<cv::Point2f>> CamLeftCornersAll;
    std::vector<std::vector<cv::Point2f>> CamRightCornersAll;
    std::vector<std::vector<cv::Point3f>> ObjVectorAll;

    CamLeftCornersAll.clear();
    CamRightCornersAll.clear();
    ObjVectorAll.clear();

    if(CaputureImgs)
    {
        std::cout<<"Press 'Esc' to stop capturing image"<<std::endl;
        std::cout<<"Press 'c' to capture one image for calibration"<<std::endl;
        while(success<CaliImgNum)
        {
            CamLeft->Trigger();
            CamRight->Trigger();

            CamLeft->CapImage();
            CamRight->CapImage();
            cv::imshow(HandleLeft, CamLeft->frame);
            cv::imshow(HandleRight, CamRight->frame);
            key = cv::waitKey(1);
            if ((CamLeft->frame.data == NULL)|(CamRight->frame.data == NULL))
            {
                std::cout<<"img is None"<<std::endl;
            }
            if ((key & 255) == 27 )        ///// Esc
            {
                std::cout<<"Stop Capturing Images"<<std::endl;
                break;
            }
            if ((key & 255) == 99)      /////  c    start capture images
            {
                std::cout<<"Start detect Camera "+CamNameLeftSave+" Calibration Board No."<<success<<std::endl;
                BoardFound=Board.ExtractBoardCoordinate(CamLeft->frame,BoardMarker,CamLeftCorners);
                if (BoardFound==false)
                {
                    std::cout<<CamNameLeftSave+" Image No."<<success<<" can not find a calibration board!!!!!"<<std::endl;
                    saveMat(CamLeft->frame,ImgPath+CamLeft->CamName+"_StereoDraw_",success,ImgFormat);
                    cv::imwrite(ImgPath+"CanNotDetectImg"+ImgFormat,CamLeft->frame);

                }
                else
                {
                    if (CamLeft->frame.channels()==1)
                    {
                        cv::cvtColor(CamLeft->frame, frameClrLeft, CV_GRAY2BGR);
                    }
                    else
                    {
                        CamLeft->frame.copyTo(frameClrLeft);
                    }
                    cv::drawChessboardCorners(frameClrLeft, Board.BoardSize, CamLeftCorners,BoardFound);
                    cv::circle(frameClrLeft,BoardMarker,1,cv::Scalar(0,0,255),3,8,0);
                    cv::imshow(HandleLeft,frameClrLeft);
                    std::cout<<"If the detection is right,input 'Enter' to save data,else input other key"<<std::endl;
                    key = cv::waitKey(0);
                    if (((key&255) == 13))            /////////////////// Enter
                    {
                        std::cout<<"Start detect Camera "+CamNameRightSave+" Calibration Board No."<<success<<std::endl;
                        BoardFound=Board.ExtractBoardCoordinate(CamRight->frame,BoardMarker,CamRightCorners);
                        if (BoardFound==false)
                        {
                            std::cout<<CamNameRightSave+" Image No."<<success<<" can not find a calibration board!!!!!"<<std::endl;
                            cv::imwrite(ImgPath+"CanNotDetectImg"+ImgFormat,CamRight->frame);
                        }
                        else
                        {
                            if (CamRight->frame.channels() == 1)
                            {
                                cv::cvtColor(CamRight->frame, frameClrRight, CV_GRAY2BGR);
                            }
                            else
                            {
                                CamRight->frame.copyTo(frameClrRight);
                            }
                            cv::drawChessboardCorners(frameClrRight, Board.BoardSize, CamRightCorners, BoardFound);
                            cv::circle(frameClrRight, BoardMarker, 1, cv::Scalar(0, 0, 255), 3, 8, 0);
                            cv::imshow(HandleRight, frameClrRight);
                            std::cout << "If the detection is right,input 'Enter' to save data,else input other key" << std::endl;
                            key = cv::waitKey(0);
                            if (((key&255) == 13))            /////////////////// Enter
                            {
                                std::cout<<"Saving this image,press 'c' to capture another one..."<<std::endl;
                                saveMat(CamLeft->frame,ImgPath+CamLeft->CamName+"_Stereo_",success,ImgFormat);  //+"CalibrateImages_"
                                saveMat(frameClrLeft,ImgPath+CamLeft->CamName+"_StereoDraw_",success,ImgFormat);

                                saveMat(CamRight->frame,ImgPath+CamRight->CamName+"_Stereo_",success,ImgFormat);
                                saveMat(frameClrRight,ImgPath+CamRight->CamName+"_StereoDraw_",success,ImgFormat);
                                CamLeftCornersAll.push_back(CamLeftCorners);
                                CamRightCornersAll.push_back(CamRightCorners);
                                ObjVectorAll.push_back(Board.ObjVector);
                                success++;
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        for (success=0;success<CaliImgNum;success++)
        {
            loadMat(CamLeft->frame, ImgPath+CamLeft->CamName+"_Stereo_",success, ImgFormat);
            loadMat(CamRight->frame, ImgPath+CamRight->CamName+"_Stereo_",success, ImgFormat);
            cv::imshow(HandleLeft, CamLeft->frame);
            cv::imshow(HandleRight, CamRight->frame);
            cv::waitKey(500);
            std::cout<<"Start detect Camera "+CamNameLeftSave+" Calibration Board No."<<success<<std::endl;
            BoardFound=Board.ExtractBoardCoordinate(CamLeft->frame,BoardMarker,CamLeftCorners);
            if (!BoardFound)
            {	std::cout<<CamNameLeftSave+" Image No."<<success<<" can not find a calibration board!!!!!"<<std::endl;}
            else
            {
                if (CamLeft->frame.channels()==1)
                {
                    cv::cvtColor(CamLeft->frame, frameClrLeft, CV_GRAY2BGR);
                }
                else
                {
                    CamLeft->frame.copyTo(frameClrLeft);
                }
                cv::drawChessboardCorners(frameClrLeft, Board.BoardSize, CamLeftCorners,BoardFound);
                cv::circle(frameClrLeft,BoardMarker,1,cv::Scalar(0,0,255),3,8,0);
                cv::imshow(HandleLeft,frameClrLeft);
                key = cv::waitKey(500);

                cv::waitKey(500);
                std::cout<<"Start detect Camera "+CamNameRightSave+" Calibration Board No."<<success<<std::endl;
                BoardFound=Board.ExtractBoardCoordinate(CamRight->frame,BoardMarker,CamRightCorners);
                if (!BoardFound)
                { std::cout<<CamNameRightSave+" Image No."<<success<<" can not find a calibration board!!!!!"<<std::endl;}
                else
                {
                    if (CamRight->frame.channels() == 1)
                    {
                        cv::cvtColor(CamRight->frame, frameClrRight, CV_GRAY2BGR);
                    }
                    else
                    {
                        CamRight->frame.copyTo(frameClrRight);
                    }
                    cv::drawChessboardCorners(frameClrRight, Board.BoardSize, CamRightCorners, BoardFound);
                    cv::circle(frameClrRight, BoardMarker, 1, cv::Scalar(0, 0, 255), 3, 8, 0);
                    cv::imshow(HandleRight, frameClrRight);
                    key = cv::waitKey(500);
                    saveMat(frameClrLeft,ImgPath+CamLeft->CamName+"_StereoDraw_",success,ImgFormat);
                    saveMat(frameClrRight,ImgPath+CamRight->CamName+"_StereoDraw_",success,ImgFormat);
                    std::cout<<"Saving the coordinates..."<<std::endl;
                    CamLeftCornersAll.push_back(CamLeftCorners);
                    CamRightCornersAll.push_back(CamRightCorners);
                    ObjVectorAll.push_back(Board.ObjVector);
                }
            }
        }
    }

    cv::destroyWindow(HandleLeft);
    cv::destroyWindow(HandleRight);

    cv::Rect validRoiLeft;
    cv::Rect validRoiRight;

    std::cout<<"...Using Image Points To Compute Stereo Camera Calibration..."<<std::endl;
    double CaliError = cv::stereoCalibrate(ObjVectorAll,CamRightCornersAll,CamLeftCornersAll,CamRight->intrinsicMatrix,CamRight->distortionMatrix,CamLeft->intrinsicMatrix,CamLeft->distortionMatrix,
                                           ImgSize,R,T,E,F,CV_CALIB_USE_INTRINSIC_GUESS, cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-6));
    std::cout << "done with RMS error=" <<CaliError<< std::endl;

    ///   ComputeStereoRectifyMatrix  ///
    ///   The unit of StereoCamLR.T is mm  ///
    cv::stereoRectify(CamRight->intrinsicMatrix,CamRight->distortionMatrix,CamLeft->intrinsicMatrix,CamLeft->distortionMatrix,
                      ImgSize,R,T, RR, RL, PR,PL, Q, CV_CALIB_ZERO_DISPARITY, -1, ImgSize,&validRoiRight,&validRoiLeft);

    ///  save calibration data  ///
    std::string ImgPathRightLeft = ImgPath+"RightToLeft//";
    saveXmlFile(CamLeft->intrinsicMatrix,"A_"+CamNameLeftSave,".xml",ImgPathRightLeft);
    saveXmlFile(CamLeft->distortionMatrix,"D_"+CamNameLeftSave,".xml",ImgPathRightLeft);

    saveXmlFile(CamRight->intrinsicMatrix,"A_"+CamNameRightSave,".xml",ImgPathRightLeft);
    saveXmlFile(CamRight->distortionMatrix,"D_"+CamNameRightSave,".xml",ImgPathRightLeft);

    saveXmlFile(R,"R",".xml",ImgPathRightLeft);
    saveXmlFile(T,"T",".xml",ImgPathRightLeft);
    saveXmlFile(E,"E",".xml",ImgPathRightLeft);
    saveXmlFile(F,"F",".xml",ImgPathRightLeft);

    saveXmlFile(RL,"RL",".xml",ImgPathRightLeft);
    saveXmlFile(RR,"RR",".xml",ImgPathRightLeft);

    saveXmlFile(PL,"PL",".xml",ImgPathRightLeft);
    saveXmlFile(PR,"PR",".xml",ImgPathRightLeft);

    saveXmlFile(Q,"Q",".xml",ImgPathRightLeft);
}


void StereoSystem::LoadStereoCamInfo()
{
    std::string CamNameLeftSave = std::string(CamLeft->CamName);
    std::string CamNameRightSave = std::string(CamRight->CamName);
    std::string PathLoad ;

    PathLoad = ImgPath+"RightToLeft//";

    loadXmlFile(CamLeft->intrinsicMatrix,"A_"+CamNameLeftSave,".xml",PathLoad);
    loadXmlFile(CamLeft->distortionMatrix,"D_"+CamNameLeftSave,".xml",PathLoad);

    loadXmlFile(CamRight->intrinsicMatrix,"A_"+CamNameRightSave,".xml",PathLoad);
    loadXmlFile(CamRight->distortionMatrix,"D_"+CamNameRightSave,".xml",PathLoad);

    loadXmlFile(R,"R",".xml",PathLoad);
    loadXmlFile(T,"T",".xml",PathLoad);
    loadXmlFile(E,"E",".xml",PathLoad);
    loadXmlFile(F,"F",".xml",PathLoad);

    loadXmlFile(RL,"RL",".xml",PathLoad);
    loadXmlFile(RR,"RR",".xml",PathLoad);

    loadXmlFile(PL,"PL",".xml",PathLoad);
    loadXmlFile(PR,"PR",".xml",PathLoad);

    loadXmlFile(Q,"Q",".xml",PathLoad);
}


void StereoSystem::RectifyPreview()
{
    std::string HandleLeft = std::string(CamLeft->CamName);
    std::string HandleRight = std::string(CamRight->CamName);

    CreateImgWindow(HandleLeft,ImgSize,HandleRight,cv::Size(CamRight->width,CamRight->height),0.95);

    cv::initUndistortRectifyMap(CamLeft->intrinsicMatrix,CamLeft->distortionMatrix, RL,PL,ImgSize, CV_16SC2, ReMapLeftX,ReMapLeftY);
    cv::initUndistortRectifyMap(CamRight->intrinsicMatrix,CamRight->distortionMatrix, RR, PR,ImgSize, CV_16SC2,ReMapRightX,ReMapRightY);


    int key;
    while(true)
    {
        CamLeft->Trigger();
        CamRight->Trigger();

        CamLeft->CapImage();
        CamRight->CapImage();
        cv::remap(CamLeft->frame, RectifyLeft, ReMapLeftX, ReMapLeftY, CV_INTER_LINEAR);
        cv::remap(CamRight->frame, RectifyRight, ReMapRightX,ReMapRightY, CV_INTER_LINEAR);
        cv::imshow(HandleLeft,RectifyLeft);
        cv::imshow(HandleRight,RectifyRight);
        key = cv::waitKey(1);
        /// Press ESC to exit ///
        if((key & 255) == 27 )
        {
            break;
        }
    }
}


void StereoSystem::Compute3DMap(int* WorkingDist, int SADWindowSize, int TextureThreshold)
{
    WorkingDistance[0] = WorkingDist[0];
    WorkingDistance[1] = WorkingDist[1];
    CamWorkingRange = WorkingDistance[1] - WorkingDistance[0];

    cv::initUndistortRectifyMap(CamLeft->intrinsicMatrix,CamLeft->distortionMatrix, RL,PL,ImgSize, CV_16SC2, ReMapLeftX,ReMapLeftY);
    cv::initUndistortRectifyMap(CamRight->intrinsicMatrix,CamRight->distortionMatrix, RR, PR,ImgSize, CV_16SC2,ReMapRightX,ReMapRightY);

    ///  block matching parameters  ///

    blockSize = SADWindowSize;
    textureThreshold = TextureThreshold;
    bm->setBlockSize(blockSize);
    bm->setTextureThreshold(textureThreshold);

    uniquenessRatio = 5;
    bm->setUniquenessRatio(uniquenessRatio);
    bm->setDisp12MaxDiff(-1);

    int disp1 = -PL.at<double>(0,3)/WorkingDistance[0];
    int disp2 = -PL.at<double>(0,3)/WorkingDistance[1];
    int xd = PL.at<double>(0,2)-PR.at<double>(0,2);
    int mindisp = disp1;
    bm->setMinDisparity(mindisp-xd);
    bm->setNumDisparities((int((abs(disp1-disp2)+15)/16))*16);

    DepthMapSize = cv::Size(ImgSize.width+bm->getMinDisparity(),ImgSize.height);
    HandleLeft = std::string(CamLeft->CamName);
    HandleRight = std::string(CamRight->CamName);
    HandleDepthMap = "DepthMap";

    bool GetImgError = false;
    int key;
    CreateImgWindow(HandleLeft,ImgSize,HandleRight,ImgSize,HandleDepthMap,DepthMapSize,0.95);
    cv::createTrackbar("ExposureTime：",     HandleLeft,     &CamLeft->ExposureTime,  CamLeft->MaxExposureTime,  onChangeLeftCam,         this);
    cv::createTrackbar("ExposureTime：",     HandleRight,    &CamRight->ExposureTime, CamRight->MaxExposureTime, onChangeRightCam,        this);
    cv::createTrackbar("SADWindowSize：",    HandleDepthMap, &blockSize,              MaxSADWindowSize,          onChangeSADWinSize,      this);
    cv::createTrackbar("MinDistance：",      HandleDepthMap, &WorkingDistance[0],     MaxValForMinDist,          onChangeMinDist,         this);
    cv::createTrackbar("WorkingRange：",     HandleDepthMap, &CamWorkingRange,        MaxWorkingRange,           onChangeWorkRange,       this);
    cv::createTrackbar("TextureThreshold：", HandleDepthMap, &textureThreshold,       MaxTextureThreshold,       onChangeTextureThresh,   this);
    cv::createTrackbar("UniquenessRatio：",  HandleDepthMap, &uniquenessRatio,        MaxUniquenessRatio,        onChangeUniquenessRatio, this);

    while(true)
    {

        CamLeft->Trigger();
        CamRight->Trigger();

        CamLeft->CapImage();
        CamRight->CapImage();

        if ((CamLeft->frame.data==NULL)||(CamRight->frame.data==NULL))
        {
            GetImgError = true;
            std::cout<<"Can not get image from camera!!"<<std::endl;
        }


        if (!GetImgError)
        {
            cv::remap(CamLeft->frame, RectifyLeft, ReMapLeftX, ReMapLeftY, CV_INTER_LINEAR);
            cv::remap(CamRight->frame, RectifyRight, ReMapRightX, ReMapRightY, CV_INTER_LINEAR);

            ShowImgGui(HandleLeft, RectifyLeft);
            ShowImgGui(HandleRight, RectifyRight);

            bm->compute(RectifyRight, RectifyLeft, dispsbm);

            dispsbm = dispsbm(cv::Rect(0, 0, DepthMapSize.width, DepthMapSize.height));
            cv::minMaxLoc(dispsbm, &minVal, &maxVal, NULL, NULL);
            dispsbm8 = (dispsbm - minVal) * (255 / (maxVal - minVal));
            dispsbm8.convertTo(dispsbm8, CV_8UC1);
            cv::reprojectImageTo3D(dispsbm, Pts3D, Q, true);

            cv::imshow(HandleDepthMap, dispsbm8);

            key = cv::waitKey(10);
            if ((key & 255) == 27)                 ///// esc
            {
                break;
            }
            if ((key & 255) == 99)      /////  c  capture and save image
            {
                std::cout << "Saving Current Images..." << std::endl;
                cv::imwrite(ImgPathMatch + "Left" + ImgFormat, CamLeft->frame);
                cv::imwrite(ImgPathMatch + "Right" + ImgFormat, CamRight->frame);
                cv::imwrite(ImgPathMatch + "LeftRectify" + ImgFormat, RectifyLeft);
                cv::imwrite(ImgPathMatch + "RightRectify" + ImgFormat, RectifyRight);
                cv::imwrite(ImgPathMatch + "DepthMap" + ImgFormat, dispsbm8);
                saveXYZ(ImgPathMatch + "DepthMap.WRL", Pts3D);
                savePLY(ImgPathMatch + "PointCloud.ply", Pts3D);
                savePCD(ImgPathMatch + "PointCloud.pcd", Pts3D);
            }
        }

    }

    AdjustLeftCamExposure (CamLeft->ExposureTime,     0);
    AdjustRightCamExposure(CamRight->ExposureTime,    0);
    UpdateSADWindowSize   (bm->getBlockSize(),        0);
    UpdateMinDist         (WorkingDistance[0],        0);
    UpdateWorkingRange    (CamWorkingRange,           0);
    UpdateTextureThresh   (bm->getTextureThreshold(), 0);
    UpdateUniquenessRatio (bm->getUniquenessRatio(),  0);


}