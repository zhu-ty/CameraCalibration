#include "StereoCameraCalibration.h"
#include "SingleCameraCalibration.h"

int StereoCalibrater::SetImageListAndPair(std::vector<std::string> listFile1, std::vector<std::string> listFile2)
{
	int i = 0, j = 0;
	this->_pairedFiles.resize(0);
	while (true)
	{
		if (i == listFile1.size() || j == listFile2.size())
			break;
		int comp = listFile1[i].compare(listFile2[j]);
		if (comp < 0)
			i++;
		else if (comp > 0)
			j++;
		else
		{
			_pairedFiles.push_back(std::make_pair(listFile1[i], listFile2[j]));
			i++; j++;
		}
	}
	return 0;
}

int StereoCalibrater::SetBoardSize(int cornerWidth, int cornerHeight, double squareSize)
{
	this->_cornerWidth = cornerWidth;
	this->_cornerHeight = cornerHeight;
	this->_squareSize = squareSize;
	return 0;
}

int StereoCalibrater::SetCameraIntrinsics(cv::Mat cameraMatrix1, cv::Mat distCoeffs1, cv::Mat cameraMatrix2, cv::Mat distCoeffs2)
{
	_cameraIntrinsics[0]._cameraMatrix = cameraMatrix1;
	_cameraIntrinsics[0]._distCoeffs = distCoeffs1;
	_cameraIntrinsics[1]._cameraMatrix = cameraMatrix2;
	_cameraIntrinsics[1]._distCoeffs = distCoeffs2;
	return 0;
}

int StereoCalibrater::Calibrate(cv::Mat & R, cv::Mat & T, cv::Mat & R1, cv::Mat & R2, cv::Mat & P1, cv::Mat & P2, cv::Mat & Q)
{
	if (_pairedFiles.size() == 0)
	{
		SysUtil::errorOutput(DEBUG_STRING + "Only 0 paired images.");
	}
	return 0;
}

int StereoCalibrater::SaveParams(std::string resultFile, std::string rectifyDataDir)
{
	return 0;
}
