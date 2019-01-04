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

	//auto t1 = clock();
	//auto tt1 = SysUtil::getCurrentTimeMicroSecond();
	//std::this_thread::sleep_for(std::chrono::milliseconds(10000));
	//auto t2 = clock();
	//auto tt2 = SysUtil::getCurrentTimeMicroSecond();
	//SysUtil::infoOutput(SysUtil::format("TIME_PASSED = %f", t2 - t1));
	//SysUtil::infoOutput(SysUtil::format("TIME_PASSED2 = %ld", tt2 - tt1));
	//system("pause");







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
	SinCal1.SetVignettingMat(
		reader.Get("CameraCalibration1", "Vignetting", "")
	);
	SinCal1.SetRedSpot(
		reader.GetBoolean("CameraCalibration", "RedSpot", false)
	);

	SinCal2.SetBoardSize(
		reader.GetInteger("CameraCalibration", "BoardWidth", 0),
		reader.GetInteger("CameraCalibration", "BoardHeight", 0),
		reader.GetReal("CameraCalibration", "BoardSize", 0)
	);
	SinCal2.SetVignettingMat(
		reader.Get("CameraCalibration2", "Vignetting", "")
	);
	SinCal2.SetRedSpot(
		reader.GetBoolean("CameraCalibration", "RedSpot", false)
	);


	SteCal.SetBoardSize(
		reader.GetInteger("CameraCalibration", "BoardWidth", 0),
		reader.GetInteger("CameraCalibration", "BoardHeight", 0),
		reader.GetReal("CameraCalibration", "BoardSize", 0)
	);
	SteCal.SetVignettingMat(
		reader.Get("CameraCalibration1", "Vignetting", ""),
		reader.Get("CameraCalibration2", "Vignetting", "")
	);
	SteCal.SetRedSpot(
		reader.GetBoolean("CameraCalibration", "RedSpot", false)
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
	cv::Mat cM1, dC1;
	SinCal1.Calibrate(cM1, dC1);
	SinCal1.SaveParams(reader.Get("CameraCalibration1", "OutputFile", "./result_left.xml"));


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
	SteCal.ShowResults();
    SysUtil::infoOutput("done!");
	
    return 0;
}
