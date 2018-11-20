/**
@brief main file 
*/

#include "INIReader.h"
#include "SysUtil.hpp"
#include "SingleCameraCalibration.h"

int main(int argc, char* argv[]) 
{
    std::string configFile = "./CCConfig.ini";
    if(argc > 1)
    {
        configFile = argv[1];
    }
    INIReader reader(configFile);
    if (reader.ParseError() < 0)
	{
		SysUtil::errorOutput(std::string("main Can't load :") + configFile);
		return -1;
	}

    SingleCalibrater SinCal;

    if(reader.GetBoolean("CameraCalibration","UseListFile","false") == true)
        SinCal.SetImageList(reader.Get("CameraCalibration", "ImageList", ""));
    else
    {
        std::string inputFolder = reader.Get("CameraCalibration", "ImageDir", ".");
        std::vector<std::string> imageNames;
        std::vector<std::string> allowedExtensions = { ".jpg", ".png" ,".jpeg"};
        for (int i = 0; i < allowedExtensions.size(); i++) {
            std::vector<std::string> imageNamesCurrentExtension;
            cv::glob(
                inputFolder + "/*" + allowedExtensions[i],
                imageNamesCurrentExtension,
                true
            );
            imageNames.insert(
                imageNames.end(),
                imageNamesCurrentExtension.begin(),
                imageNamesCurrentExtension.end()
            );
        }
        SinCal.SetImageList(imageNames);
    }
    SinCal.SetBoardSize(
        reader.GetInteger("CameraCalibration", "BoardWidth", 0),
        reader.GetInteger("CameraCalibration", "BoardHeight", 0),
        reader.GetReal("CameraCalibration", "BoardSize", 0)
    );

    cv::Mat cM,dC;
    SinCal.Calibrate(cM, dC);
    SinCal.saveParams(eader.Get("CameraCalibration", "OutputFile", "./result.xml"));
    SysUtil::infoOutput("done!");

    return 0;
}