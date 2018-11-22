/**
@brief main file 
*/

#include "INIReader.h"
#include "SKCommon.hpp"
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







    SingleCalibrater SinCal1, SinCal2;
	StereoCalibrater SteCal;
	SinCal1.SetBoardSize(
		reader.GetInteger("CameraCalibration", "BoardWidth", 0),
		reader.GetInteger("CameraCalibration", "BoardHeight", 0),
		reader.GetReal("CameraCalibration", "BoardSize", 0)
	);
	SinCal2.SetBoardSize(
		reader.GetInteger("CameraCalibration", "BoardWidth", 0),
		reader.GetInteger("CameraCalibration", "BoardHeight", 0),
		reader.GetReal("CameraCalibration", "BoardSize", 0)
	);
	SteCal.SetBoardSize(
		reader.GetInteger("CameraCalibration", "BoardWidth", 0),
		reader.GetInteger("CameraCalibration", "BoardHeight", 0),
		reader.GetReal("CameraCalibration", "BoardSize", 0)
	);


    if(reader.GetBoolean("CameraCalibration1","UseListFile",false) == true)
        SinCal1.SetImageList(reader.Get("CameraCalibration1", "ImageList", ""));
    else
    {
        std::string inputFolder = reader.Get("CameraCalibration1", "ImageDir", ".");
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
        SinCal1.SetImageList(imageNames);
    }
	if (reader.GetBoolean("CameraCalibration2", "UseListFile", false) == true)
		SinCal2.SetImageList(reader.Get("CameraCalibration2", "ImageList", ""));
	else
	{
		std::string inputFolder = reader.Get("CameraCalibration2", "ImageDir", ".");
		std::vector<std::string> imageNames;
		std::vector<std::string> allowedExtensions = { ".jpg", ".png" ,".jpeg" };
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
		SinCal2.SetImageList(imageNames);
	}
   

    cv::Mat cM1,dC1;
    SinCal1.Calibrate(cM1, dC1);
    SinCal1.SaveParams(reader.Get("CameraCalibration1", "OutputFile", "./result_left.xml"));
	cv::Mat cM2, dC2;
	SinCal2.Calibrate(cM2, dC2);
	SinCal2.SaveParams(reader.Get("CameraCalibration2", "OutputFile", "./result_right.xml"));

	SteCal.SetCameraIntrinsics(cM1, dC1, cM2, dC2);

	{
		std::vector<std::string> imageNamesLeft, imageNamesRight;
		std::vector<std::string> allowedExtensions = { ".jpg", ".png" ,".jpeg" };

		std::string inputFolder = reader.Get("StereoCalibration", "ImageDirLeft", ".");
		for (int i = 0; i < allowedExtensions.size(); i++) {
			std::vector<cv::String> imageNamesCurrentExtension;
			cv::glob(
				inputFolder + "/*" + allowedExtensions[i],
				imageNamesCurrentExtension,
				true
			);
			imageNamesLeft.insert(
				imageNamesLeft.end(),
				imageNamesCurrentExtension.begin(),
				imageNamesCurrentExtension.end()
			);
		}
		inputFolder = reader.Get("StereoCalibration", "ImageDirRight", ".");
		for (int i = 0; i < allowedExtensions.size(); i++) {
			std::vector<cv::String> imageNamesCurrentExtension;
			cv::glob(
				inputFolder + "/*" + allowedExtensions[i],
				imageNamesCurrentExtension,
				true
			);
			imageNamesRight.insert(
				imageNamesRight.end(),
				imageNamesCurrentExtension.begin(),
				imageNamesCurrentExtension.end()
			);
		}
		SteCal.SetImageListAndPair(imageNamesLeft, imageNamesRight);
	}

	cv::Mat R, T, R1, R2, P1, P2, Q;
	SteCal.Calibrate(R, T, R1, R2, P1, P2, Q);
	SteCal.SaveParams(
		reader.Get("StereoCalibration", "OutputFile", "./result_stereo.xml"),
		reader.Get("StereoCalibration", "OutPutParamDir", "./param/")
	);

    SysUtil::infoOutput("done!");

    return 0;
}
