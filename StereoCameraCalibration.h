/**
@brief class to calibrate stereo cameras
@author zhu-ty
@date Nov 20, 2018
*/

#ifndef __CAMERA_CALIBRATION_STEREO__
#define __CAMERA_CALIBRATION_STEREO__

#include "SysUtil.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>


class StereoCalibrater
{
private:
	std::vector<std::pair<std::string, std::string>> _pairedFiles;
	int _cornerWidth, _cornerHeight;
	double _squareSize;
	cv::Size _imageSize;

	struct
	{
		cv::Mat _cameraMatrix, _distCoeffs;
	} _cameraIntrinsics[2];
public:
	//Pair 2 img lists, file with same name will be paired
	int SetImageListAndPair(std::vector<std::string> listFile1, std::vector<std::string> listFile2);

	int SetBoardSize(int cornerWidth, int cornerHeight, double squareSize);

	int SetCameraIntrinsics(cv::Mat cameraMatrix1, cv::Mat distCoeffs1, cv::Mat cameraMatrix2, cv::Mat distCoeffs2);

	int Calibrate(cv::Mat &R, cv::Mat &T, cv::Mat &R1, cv::Mat &R2, cv::Mat &P1, cv::Mat &P2, cv::Mat &Q);

	int SaveParams(std::string resultFile, std::string rectifyDataDir);
};

#endif //__CAMERA_CALIBRATION_STEREO__