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
                           int MaxTextureThreshold):
                           CamLeft(CamLeft),
                           CamRight(CamRight),
                           ImgPath(ImgPath),
                           ImgPathMatch(ImgPathMatch),
                           ImgFormat(ImgFormat),
                           ImgSize(cv::Size(CamLeft->width,CamLeft->height)),
                           MaxSADWindowSize(MaxSADWindowSize),
                           MaxValForMinDist(MaxValForMinDist),
                           MaxWorkingRange(MaxWorkingRange),
                           MaxTextureThreshold(MaxTextureThreshold)
{

}

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


void StereoSystem::UpdateSADWindowSize(int SADWindowSize, void*)
{
    /// The SADWindowSize should be odd number ///
    if (SADWindowSize % 2 == 0)
        SADWindowSize += 1;
    bm.state->SADWindowSize = SADWindowSize;
}

void StereoSystem::UpdateTextureThresh(int TextureThresh, void *)
{
    bm.state->textureThreshold =TextureThresh;
}

void StereoSystem::UpdateMinDist(int MinDist, void*)
{
    WorkingDistance[0] = MinDist;
    int disp1 = -PL.at<double>(0,3)/WorkingDistance[0];
    int disp2 = -PL.at<double>(0,3)/WorkingDistance[1];
    int xd = PL.at<double>(0,2)-PR.at<double>(0,2);
    int mindisp = disp1;
    bm.state->minDisparity = mindisp-xd;
    bm.state->numberOfDisparities = (int((abs(disp1-disp2)+15)/16))*16;
    DepthMapSize = cv::Size(ImgSize.width+bm.state->minDisparity,ImgSize.height);

//    DepthMapSize = cv::Size(ImgSize.width+bm.state->minDisparity,ImgSize.height);
//    cv::minMaxLoc(dispsbm, &minVal, &maxVal, NULL, NULL);
//    dispsbm8 = (dispsbm - minVal) * (255 / (maxVal - minVal));
//    dispsbm8.convertTo(dispsbm8, CV_8UC1);
//    cv::reprojectImageTo3D(dispsbm, Pts3D, Q, true);

}

void StereoSystem::UpdateWorkingRange(int WorkingRange, void*)
{
    CamWorkingRange = WorkingRange;
    WorkingDistance[1] = CamWorkingRange + WorkingDistance[0];

    int disp1 = -PL.at<double>(0,3)/WorkingDistance[0];
    int disp2 = -PL.at<double>(0,3)/WorkingDistance[1];
    int xd = PL.at<double>(0,2)-PR.at<double>(0,2);
    int mindisp = disp1;
    bm.state->minDisparity = mindisp-xd;
    bm.state->numberOfDisparities = (int((abs(disp1-disp2)+15)/16))*16;
    DepthMapSize = cv::Size(ImgSize.width+bm.state->minDisparity,ImgSize.height);

//    DepthMapSize = cv::Size(ImgSize.width+bm.state->minDisparity,ImgSize.height);
//    cv::minMaxLoc(dispsbm, &minVal, &maxVal, NULL, NULL);
//    dispsbm8 = (dispsbm - minVal) * (255 / (maxVal - minVal));
//    dispsbm8.convertTo(dispsbm8, CV_8UC1);
//    cv::reprojectImageTo3D(dispsbm, Pts3D, Q, true);
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
        //std::cout<<(key & 255)<<std::endl;
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

    AdjustLeftCamExposure (CamLeft->ExposureTime,  0);
    AdjustRightCamExposure(CamRight->ExposureTime, 0);

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
                    if (((key&255) == 10)||((key&255) == 141))            /////////////////// Enter
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
                            if (((key&255) == 10)||((key&255) == 141))            /////////////////// Enter
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
            loadMat(CamLeft->frame, ImgPath+CamLeft->CamName+"_Stereo_",success, ImgFormat);    //+"_Stereo_"
            //CamLeft->frame = cv::imread(ImgPath+"CanNotDetectImg"+ImgFromat,-1);
            loadMat(CamRight->frame, ImgPath+CamRight->CamName+"_Stereo_",success, ImgFormat);    //+"_Stereo_"
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
                    saveMat(CamLeft->frame,ImgPath+CamLeft->CamName+"_StereoDraw_",success,ImgFormat);
                    saveMat(CamRight->frame,ImgPath+CamRight->CamName+"_StereoDraw_",success,ImgFormat);
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

    /////////////////////////////////   Left Right Stereo ////////////////////////////////////
    std::cout << "...Using Image Points To Compute Stereo Camera Calibration..." << std::endl;
    double CaliError = cv::stereoCalibrate(ObjVectorAll, CamLeftCornersAll, CamRightCornersAll, CamLeft->intrinsicMatrix, CamLeft->distortionMatrix, CamRight->intrinsicMatrix, CamRight->distortionMatrix,
                                           ImgSize, R, T, E, F, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6), CV_CALIB_USE_INTRINSIC_GUESS);
    std::cout << "done with RMS error=" << CaliError << std::endl;

    ////////////////   ComputeStereoRectifyMatrix

    ////////////////   The unit of StereoCamLR.T is mm /////////////
    cv::stereoRectify(CamLeft->intrinsicMatrix, CamLeft->distortionMatrix, CamRight->intrinsicMatrix, CamRight->distortionMatrix,
                      ImgSize, R, T, RL, RR, PL, PR, Q, 0, CV_CALIB_ZERO_DISPARITY, ImgSize, &validRoiLeft,&validRoiRight);    //   cv::CALIB_ZERO_DISPARITY  ???
    ////// save calibration data //////
    std::string ImgPathLeftRight = ImgPath+"LeftRight//";
    saveXmlFile(CamLeft->intrinsicMatrix,"IntrinsicMatrix"+CamNameLeftSave,".xml",ImgPathLeftRight);
    saveXmlFile(CamLeft->distortionMatrix,"DistortionMatrix"+CamNameLeftSave,".xml",ImgPathLeftRight);

    saveXmlFile(CamRight->intrinsicMatrix,"IntrinsicMatrix"+CamNameRightSave,".xml",ImgPathLeftRight);
    saveXmlFile(CamRight->distortionMatrix,"DistortionMatrix"+CamNameRightSave,".xml",ImgPathLeftRight);

    saveXmlFile(R,"CamABRotationMatrix",".xml",ImgPathLeftRight);
    saveXmlFile(T,"CamABTranslationMatrix",".xml",ImgPathLeftRight);
    saveXmlFile(E,"CamABEssentialMatrix",".xml",ImgPathLeftRight);
    saveXmlFile(F,"CamABFundamentalMatrix",".xml",ImgPathLeftRight);

    saveXmlFile(RL,"RL",".xml",ImgPathLeftRight);
    saveXmlFile(RR,"RR",".xml",ImgPathLeftRight);

    saveXmlFile(PL,"PL",".xml",ImgPathLeftRight);
    saveXmlFile(PR,"PR",".xml",ImgPathLeftRight);

    saveXmlFile(Q,"Q",".xml",ImgPathLeftRight);





    /////////////////////////////////   Right Left Stereo ////////////////////////////////////
    std::cout<<"...Using Image Points To Compute Stereo Camera Calibration..."<<std::endl;
    CaliError = cv::stereoCalibrate(ObjVectorAll,CamRightCornersAll,CamLeftCornersAll,CamRight->intrinsicMatrix,CamRight->distortionMatrix,CamLeft->intrinsicMatrix,CamLeft->distortionMatrix,
                                    ImgSize,R,T,E,F,cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-6),CV_CALIB_USE_INTRINSIC_GUESS);
    std::cout << "done with RMS error=" <<CaliError<< std::endl;

    ////////////////   ComputeStereoRectifyMatrix
    ////////////////   The unit of StereoCamLR.T is mm /////////////
    cv::stereoRectify(CamRight->intrinsicMatrix,CamRight->distortionMatrix,CamLeft->intrinsicMatrix,CamLeft->distortionMatrix,
                      ImgSize,R,T, RR, RL, PR,PL, Q,0, CV_CALIB_ZERO_DISPARITY ,ImgSize,&validRoiRight,&validRoiLeft);    //   cv::CALIB_ZERO_DISPARITY  ???
    ////// save calibration data //////
    std::string ImgPathRightLeft = ImgPath+"RightLeft//";
    saveXmlFile(CamLeft->intrinsicMatrix,"IntrinsicMatrix"+CamNameLeftSave,".xml",ImgPathRightLeft);
    saveXmlFile(CamLeft->distortionMatrix,"DistortionMatrix"+CamNameLeftSave,".xml",ImgPathRightLeft);

    saveXmlFile(CamRight->intrinsicMatrix,"IntrinsicMatrix"+CamNameRightSave,".xml",ImgPathRightLeft);
    saveXmlFile(CamRight->distortionMatrix,"DistortionMatrix"+CamNameRightSave,".xml",ImgPathRightLeft);

    saveXmlFile(R,"CamABRotationMatrix",".xml",ImgPathRightLeft);
    saveXmlFile(T,"CamABTranslationMatrix",".xml",ImgPathRightLeft);
    saveXmlFile(E,"CamABEssentialMatrix",".xml",ImgPathRightLeft);
    saveXmlFile(F,"CamABFundamentalMatrix",".xml",ImgPathRightLeft);

    saveXmlFile(RL,"RL",".xml",ImgPathRightLeft);
    saveXmlFile(RR,"RR",".xml",ImgPathRightLeft);

    saveXmlFile(PL,"PL",".xml",ImgPathRightLeft);
    saveXmlFile(PR,"PR",".xml",ImgPathRightLeft);

    saveXmlFile(Q,"Q",".xml",ImgPathRightLeft);



}

void StereoSystem::LoadStereoCamInfo(bool LeftRight)
{
    std::string CamNameLeftSave = std::string(CamLeft->CamName);
    std::string CamNameRightSave = std::string(CamRight->CamName);
    std::string PathLoad ;

    if (LeftRight)
    {PathLoad = ImgPath+"LeftRight/";}
    else
    {PathLoad = ImgPath+"RightLeft/";}

    loadXmlFile(CamLeft->intrinsicMatrix,"IntrinsicMatrix"+CamNameLeftSave,".xml",PathLoad);
    loadXmlFile(CamLeft->distortionMatrix,"DistortionMatrix"+CamNameLeftSave,".xml",PathLoad);

    loadXmlFile(CamRight->intrinsicMatrix,"IntrinsicMatrix"+CamNameRightSave,".xml",PathLoad);
    loadXmlFile(CamRight->distortionMatrix,"DistortionMatrix"+CamNameRightSave,".xml",PathLoad);

    loadXmlFile(R,"CamABRotationMatrix",".xml",PathLoad);
    loadXmlFile(T,"CamABTranslationMatrix",".xml",PathLoad);
    loadXmlFile(E,"CamABEssentialMatrix",".xml",PathLoad);
    loadXmlFile(F,"CamABFundamentalMatrix",".xml",PathLoad);
/////////////////////////  Wrong ??????  /////////////////
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


void StereoSystem::Compute3DMap(int* WorkingDist, bool LeftRight, bool DebugMode, int SADWindowSize, int TextureThreshold)
{
    WorkingDistance[0] = WorkingDist[0];
    WorkingDistance[1] = WorkingDist[1];
    CamWorkingRange = WorkingDistance[1] - WorkingDistance[0];

    cv::initUndistortRectifyMap(CamLeft->intrinsicMatrix,CamLeft->distortionMatrix, RL,PL,ImgSize, CV_16SC2, ReMapLeftX,ReMapLeftY);
    cv::initUndistortRectifyMap(CamRight->intrinsicMatrix,CamRight->distortionMatrix, RR, PR,ImgSize, CV_16SC2,ReMapRightX,ReMapRightY);

    /////////// block matching parameters ///////

    bm.state->SADWindowSize=SADWindowSize; //39 17
    bm.state->textureThreshold =TextureThreshold;   //5000; threshold for denoise filter

    bm.state->uniquenessRatio = 5;
    bm.state->disp12MaxDiff = -1;
    // bm.state->speckleWindowSize = 9;
    // bm.state->speckleRange = 3;

    ////  for testing ///
    // bm.state->speckleWindowSize = 0;
    // bm.state->speckleRange = 0;
    // bm.state->uniquenessRatio = 1;

    if (LeftRight)
    {
        int disp1 = -PR.at<double>(0,3)/WorkingDistance[0];
        int disp2 = -PR.at<double>(0,3)/WorkingDistance[1];
        int xd = PR.at<double>(0,2)-PL.at<double>(0,2);
        int mindisp = disp1;
        bm.state->minDisparity = mindisp+xd;
        bm.state->numberOfDisparities = (int((abs(disp1-disp2)+15)/16))*16;
    }
    else
    {
        int disp1 = -PL.at<double>(0,3)/WorkingDistance[0];
        int disp2 = -PL.at<double>(0,3)/WorkingDistance[1];
        int xd = PL.at<double>(0,2)-PR.at<double>(0,2);
        int mindisp = disp1;
        bm.state->minDisparity = mindisp-xd;
        bm.state->numberOfDisparities = (int((abs(disp1-disp2)+15)/16))*16;
    }

    DepthMapSize = cv::Size(ImgSize.width+bm.state->minDisparity,ImgSize.height);
//    cv::Size DepthMapSizeResize = DepthMapSize;
    HandleLeft = std::string(CamLeft->CamName);
    HandleRight = std::string(CamRight->CamName);
    HandleDepthMap = "DepthMap";

    bool GetImgError = false;
    int key;
    CreateImgWindow(HandleLeft,ImgSize,HandleRight,ImgSize,HandleDepthMap,DepthMapSize,0.95);
    cv::createTrackbar("ExposureTime：",     HandleLeft,     &CamLeft->ExposureTime,      CamLeft->MaxExposureTime,  onChangeLeftCam,       this);
    cv::createTrackbar("ExposureTime：",     HandleRight,    &CamRight->ExposureTime,     CamRight->MaxExposureTime, onChangeRightCam,      this);
    cv::createTrackbar("SADWindowSize：",    HandleDepthMap, &bm.state->SADWindowSize,    MaxSADWindowSize,          onChangeSADWinSize,    this);
    cv::createTrackbar("MinDistance：",      HandleDepthMap, &WorkingDistance[0],         MaxValForMinDist,          onChangeMinDist,       this);
    cv::createTrackbar("WorkingRange：",     HandleDepthMap, &CamWorkingRange,            MaxWorkingRange,           onChangeWorkRange,     this);
    cv::createTrackbar("TextureThreshold：", HandleDepthMap, &bm.state->textureThreshold, MaxTextureThreshold,       onChangeTextureThresh, this);

    while(true)
    {
        if (DebugMode)
        {
            CamLeft->frame = cv::imread(ImgPathMatch + "Left.png", -1);
            CamRight->frame = cv::imread(ImgPathMatch + "Right.png", -1);
        }
        else
        {
            CamLeft->Trigger();
            CamRight->Trigger();

            CamLeft->CapImage();
            CamRight->CapImage();
            //cv::imshow("CamLeft->frame",CamLeft->frame);
            //cv::waitKey(0);
            if ((CamLeft->frame.data==NULL)||(CamRight->frame.data==NULL))
            {
                GetImgError = true;
                std::cout<<"Can not get image from camera!!"<<std::endl;
            }
        }

        if (!GetImgError)
        {
            cv::remap(CamLeft->frame, RectifyLeft, ReMapLeftX, ReMapLeftY, CV_INTER_LINEAR);
            cv::remap(CamRight->frame, RectifyRight, ReMapRightX, ReMapRightY, CV_INTER_LINEAR);

            //cv::imshow(HandleLeft, RectifyLeft);
            //cv::imshow(HandleRight, RectifyRight);
            ShowImgGui(HandleLeft, RectifyLeft);
            ShowImgGui(HandleRight, RectifyRight);

            if (LeftRight)
            {
                bm.operator()(RectifyLeft, RectifyRight, dispsbm, CV_32F);
            } else
            {
                bm.operator()(RectifyRight, RectifyLeft, dispsbm, CV_32F);
            }

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
                saveXYZ(ImgPathMatch + "3D.WRL", Pts3D);
                savePCD(ImgPathMatch + "PointCloud.pcd", Pts3D);
                savePLY(ImgPathMatch + "PointCloud.ply", Pts3D);
            }
        }

    }

    AdjustLeftCamExposure (CamLeft->ExposureTime,   0);
    AdjustRightCamExposure(CamRight->ExposureTime,  0);
    UpdateSADWindowSize(bm.state->SADWindowSize,    0);
    UpdateMinDist      (WorkingDistance[0],         0);
    UpdateWorkingRange (CamWorkingRange,            0);
    UpdateTextureThresh(bm.state->textureThreshold, 0);


}