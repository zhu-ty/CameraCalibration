/**
@brief main file 
*/

#include "INIReader.h"
#include "SysUtil.hpp"
#include "SingleCameraCalibration.h"
#include "StereoCameraCalibration.h"

int main(int argc, char* argv[]) 
{
	//std::string a1 = "00001.png";
	//std::string a2 = "00002.png";
	//std::string a10 = "00010.png";
	//int c1 = a1.compare(a2); //< 0
	//int c2 = a1.compare(a10); //< 0
	//int c3 = a2.compare(a10); //< 0

	std::string a2;

	cv::Mat a, b, c, d, e, f, g;
	StereoCalibrater SC;
	SC.Calibrate(a, b, c, d, e, f, g);


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

    if(reader.GetBoolean("CameraCalibration","UseListFile",false) == true)
        SinCal.SetImageList(reader.Get("CameraCalibration", "ImageList", ""));
    else
    {
        std::string inputFolder = reader.Get("CameraCalibration", "ImageDir", ".");
        std::vector<std::string> imageNames;
        std::vector<std::string> allowedExtensions = { ".jpg", ".png" ,".jpeg"};
        for (int i = 0; i < allowedExtensions.size(); i++) {
            std::vector<cv::String> imageNamesCurrentExtension;
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
    SinCal.SaveParams(reader.Get("CameraCalibration", "OutputFile", "./result.xml"));
    SysUtil::infoOutput("done!");

    return 0;
}
