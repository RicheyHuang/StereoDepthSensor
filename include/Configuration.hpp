#include <string>
#include <opencv2/opencv.hpp>
#include "CamSys.h"
#include "Camera.h"
#include "StereoSystem.h"

struct CameraParameters
{
    int ImageWidth;
    int ImageHeight;
    int LeftCamExposureTime;
    int RightCamExposureTime;
    int BlockSize;
    int MinDistance;
    int WorkingRange;
    int MaxExposureTime;
    int MaxBlockSize;
    int MaxValForMinDistance;
    int MaxWorkingRange;
    int TextureThreshold;
    int MaxTextureThreshold;
    int MaxUniquenessRatio;
};

struct CameraProperties
{
    std::string LeftCamName;
    std::string RightCamName;
    std::string LeftCamCalibrationFilesPath;
    std::string RightCamCalibrationFilesPath;
    std::string StereoCalibrationFilesPath;
    std::string DepthImageFilePath;
    std::string ImageSavedFormat;
};

struct CalibrationParameters
{
    std::string DoOpenCamera;
    std::string DoCalibration;
    std::string DoCaptureImage;
    int  ImageNum;
};

struct CalibrationBoardParameters
{
    int   PointsCols;
    int   PointsRows;
    float CircleDistanceWidth;
    float CircleDistanceHeight;
    float CircleDiameter;
    float BoardWidth;
    float BoardHeight;
};

class ParameterConfiguration
{
public:
    std::string                m_Path_ConfigFile;

    CameraProperties           m_CamProp;
    CameraParameters           m_CamParams;
    CalibrationBoardParameters m_BoardParams;
    CalibrationParameters      m_CalibParams;


    ParameterConfiguration(const std::string &Path_ConfigFile)
    {
        m_Path_ConfigFile     = Path_ConfigFile;
    }

    ParameterConfiguration(){}

    void ConfigureCamProp()
    {
        cv::FileStorage File(m_Path_ConfigFile, cv::FileStorage::READ);
        File["LeftCamName"]                  >> m_CamProp.LeftCamName;
        File["RightCamName"]                 >> m_CamProp.RightCamName;
        File["LeftCamCalibrationFilesPath"]  >> m_CamProp.LeftCamCalibrationFilesPath;
        File["RightCamCalibrationFilesPath"] >> m_CamProp.RightCamCalibrationFilesPath;
        File["StereoCalibrationFilesPath"]   >> m_CamProp.StereoCalibrationFilesPath;
        File["DepthImageFilePath"]           >> m_CamProp.DepthImageFilePath;
        File["ImageSavedFormat"]             >> m_CamProp.ImageSavedFormat;
        File.release();
    };

    void ConfigureCamParams()
    {
        cv::FileStorage File(m_Path_ConfigFile, cv::FileStorage::READ);
        File["ImageWidth"]           >> m_CamParams.ImageWidth;
        File["ImageHeight"]          >> m_CamParams.ImageHeight;
        File["LeftCamExposureTime"]  >> m_CamParams.LeftCamExposureTime;
        File["RightCamExposureTime"] >> m_CamParams.RightCamExposureTime;
        File["BlockSize"]            >> m_CamParams.BlockSize;
        File["MinDistance"]          >> m_CamParams.MinDistance;
        File["WorkingRange"]         >> m_CamParams.WorkingRange;
        File["MaxExposureTime"]      >> m_CamParams.MaxExposureTime;
        File["MaxBlockSize"]         >> m_CamParams.MaxBlockSize;
        File["MaxValForMinDistance"] >> m_CamParams.MaxValForMinDistance;
        File["MaxWorkingRange"]      >> m_CamParams.MaxWorkingRange;
        File["TextureThreshold"]     >> m_CamParams.TextureThreshold;
        File["MaxTextureThreshold"]  >> m_CamParams.MaxTextureThreshold;
        File["MaxUniquenessRatio"]   >> m_CamParams.MaxUniquenessRatio;
        File.release();
    };

    void ConfigureBoardParams()
    {
        cv::FileStorage File(m_Path_ConfigFile, cv::FileStorage::READ);
        File["PointsCols"]           >> m_BoardParams.PointsCols;
        File["PointsRows"]           >> m_BoardParams.PointsRows;
        File["CircleDistanceWidth"]  >> m_BoardParams.CircleDistanceWidth;
        File["CircleDistanceHeight"] >> m_BoardParams.CircleDistanceHeight;
        File["CircleDiameter"]       >> m_BoardParams.CircleDiameter;
        File["BoardWidth"]           >> m_BoardParams.BoardWidth;
        File["BoardHeight"]          >> m_BoardParams.BoardHeight;
        File.release();
    };

    void ConfigureCalibParams()
    {
        cv::FileStorage File(m_Path_ConfigFile, cv::FileStorage::READ);
        File["DoOpenCamera"]   >> m_CalibParams.DoOpenCamera;
        File["DoCalibration"]  >> m_CalibParams.DoCalibration;
        File["DoCaptureImage"] >> m_CalibParams.DoCaptureImage;
        File["ImageNum"]       >> m_CalibParams.ImageNum;
        File.release();
    };

    void Configure()
    {
        ConfigureCamProp();
        ConfigureCamParams();
        ConfigureBoardParams();
        ConfigureCalibParams();
    }
};


class DepthSensor
{
public:
    Camera*                m_Cam             = nullptr;
    ParameterConfiguration m_ParamsConfig    = ParameterConfiguration();
    CamSys                 m_LeftCam         = CamSys();
    CamSys                 m_RightCam        = CamSys();
    StereoSystem           m_StereoCam       = StereoSystem();
    CalibrationBoard       m_CalibBoard      = CalibrationBoard();
    int                    m_WorkingDistance[2];

    std::vector<char>      m_LeftCamName;
    std::vector<char>      m_RightCamName;
    bool                   m_DoOpenCamera;
    bool                   m_IsDebugMode;
    bool                   m_DoCalibration;
    bool                   m_DoCaptureImage;
    

    DepthSensor(const ParameterConfiguration& ParamsConfig)
    {
        m_ParamsConfig = ParamsConfig;
    };

    void Initialize()
    {
        std::istringstream(m_ParamsConfig.m_CalibParams.DoOpenCamera) >> std::boolalpha >> m_DoOpenCamera;
        std::istringstream(m_ParamsConfig.m_CalibParams.DoCalibration) >> std::boolalpha >> m_DoCalibration;
        std::istringstream(m_ParamsConfig.m_CalibParams.DoCaptureImage) >> std::boolalpha >> m_DoCaptureImage;

        m_Cam = new Camera();
        if (m_DoOpenCamera)
        {
            bool IsFound = m_Cam->EnumerateDevice();
            if (!IsFound)
            {
                std::cout << "Cannot Find Any Camera!" << std::endl;
                exit(0);
            }
        }

        m_LeftCamName = std::vector<char>(m_ParamsConfig.m_CamProp.LeftCamName.begin(),m_ParamsConfig.m_CamProp.LeftCamName.end());
        m_LeftCamName.push_back('\0');

        m_RightCamName = std::vector<char>(m_ParamsConfig.m_CamProp.RightCamName.begin(),m_ParamsConfig.m_CamProp.RightCamName.end());
        m_RightCamName.push_back('\0');

        m_LeftCam = CamSys(m_Cam,
                           &m_LeftCamName[0],
                           m_ParamsConfig.m_CamParams.ImageWidth,
                           m_ParamsConfig.m_CamParams.ImageHeight,
                           m_ParamsConfig.m_CamParams.LeftCamExposureTime,
                           m_ParamsConfig.m_CamParams.MaxExposureTime,
                           m_ParamsConfig.m_CamProp.LeftCamCalibrationFilesPath,
                           m_ParamsConfig.m_CamProp.ImageSavedFormat);
        m_RightCam = CamSys(m_Cam,
                            &m_RightCamName[0],
                            m_ParamsConfig.m_CamParams.ImageWidth,
                            m_ParamsConfig.m_CamParams.ImageHeight,
                            m_ParamsConfig.m_CamParams.RightCamExposureTime,
                            m_ParamsConfig.m_CamParams.MaxExposureTime,
                            m_ParamsConfig.m_CamProp.RightCamCalibrationFilesPath,
                            m_ParamsConfig.m_CamProp.ImageSavedFormat);

        m_LeftCam.Initialize();
        m_RightCam.Initialize();
        m_StereoCam = StereoSystem(&m_LeftCam,
                                   &m_RightCam,
                                   m_ParamsConfig.m_CamProp.StereoCalibrationFilesPath,
                                   m_ParamsConfig.m_CamProp.DepthImageFilePath,
                                   m_ParamsConfig.m_CamProp.ImageSavedFormat,
                                   m_ParamsConfig.m_CamParams.MaxBlockSize,
                                   m_ParamsConfig.m_CamParams.MaxValForMinDistance,
                                   m_ParamsConfig.m_CamParams.MaxWorkingRange,
                                   m_ParamsConfig.m_CamParams.MaxTextureThreshold,
                                   m_ParamsConfig.m_CamParams.MaxUniquenessRatio);
    };

    void Debug()
    {
        m_CalibBoard = CalibrationBoard(m_ParamsConfig.m_BoardParams.PointsCols,
                                        m_ParamsConfig.m_BoardParams.PointsRows,
                                        m_ParamsConfig.m_BoardParams.CircleDistanceWidth,
                                        m_ParamsConfig.m_BoardParams.CircleDistanceHeight,
                                        m_ParamsConfig.m_BoardParams.BoardWidth,
                                        m_ParamsConfig.m_BoardParams.BoardHeight,
                                        m_ParamsConfig.m_BoardParams.CircleDiameter);
        m_StereoCam.DebugMode(m_CalibBoard);
    }


    void Calibrate()
    {
        m_CalibBoard = CalibrationBoard(m_ParamsConfig.m_BoardParams.PointsCols,
                                        m_ParamsConfig.m_BoardParams.PointsRows,
                                        m_ParamsConfig.m_BoardParams.CircleDistanceWidth,
                                        m_ParamsConfig.m_BoardParams.CircleDistanceHeight,
                                        m_ParamsConfig.m_BoardParams.BoardWidth,
                                        m_ParamsConfig.m_BoardParams.BoardHeight,
                                        m_ParamsConfig.m_BoardParams.CircleDiameter);

        if (m_DoCalibration)
        {
            if  (m_DoCaptureImage)
            {
                bool ChangeDirection = m_StereoCam.SelectCamDirection();
                if (ChangeDirection)
                {
                    std::cout<<"The Positions of Cameras Need to be Changed!"<<std::endl;
                    exit(0);
                }
                m_StereoCam.AdjustCameraFocus();
            }
            m_StereoCam.CamLeft->CalibrateCamera (m_CalibBoard, m_ParamsConfig.m_CalibParams.ImageNum,
                                                  m_DoCaptureImage);
            m_StereoCam.CamRight->CalibrateCamera(m_CalibBoard, m_ParamsConfig.m_CalibParams.ImageNum,
                                                  m_DoCaptureImage);
            m_StereoCam.StereoCalibration        (m_CalibBoard, m_ParamsConfig.m_CalibParams.ImageNum,
                                                  m_DoCaptureImage);
        }
    };

    void Run()
    {
        m_StereoCam.LoadStereoCamInfo();

        m_WorkingDistance[0] = m_ParamsConfig.m_CamParams.MinDistance;
        m_WorkingDistance[1] = m_ParamsConfig.m_CamParams.MinDistance + m_ParamsConfig.m_CamParams.WorkingRange;
        m_StereoCam.Compute3DMap(m_WorkingDistance,
                                 m_ParamsConfig.m_CamParams.BlockSize,
                                 m_ParamsConfig.m_CamParams.TextureThreshold);
    };

};