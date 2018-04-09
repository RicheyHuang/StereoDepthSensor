#include <iostream>
#include <string.h>
#include "opencv2/opencv.hpp"
#include "ToolBox.h"
#include "CalibrationBoard.h"
#include "CamSys.h"
#include "Camera.h"
#include "StereoSystem.h"
#include "Configuration.hpp"

int main()
{
    /// Using Right Camera as reference ///
    std::string            Path_ConfigFile = "..//Calibration//Configuration//CamGroup1_Config.yaml";
    ParameterConfiguration ConfigFile(Path_ConfigFile);
    ConfigFile.Configure();
    DepthSensor            DepthCam(ConfigFile);
    DepthCam.Initialize();
//    DepthCam.Debug();
//    DepthCam.Calibrate();
    DepthCam.Run();
    return 0;
}
