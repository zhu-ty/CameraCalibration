/**
@brief class to calibrate stereo cameras
@author zhu-ty
@date Nov 20, 2018
*/

#ifndef __CAMERA_CALIBRATION_STEREO__
#define __CAMERA_CALIBRATION_STEREO__

#include "SKCommon.hpp"

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
	/*
	https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga91018d80e2a93ade37539f01e6f07de5
	*/
	const int _flag =
		//cv::CALIB_FIX_INTRINSIC |
		cv::CALIB_USE_INTRINSIC_GUESS |
		//cv::CALIB_FIX_FOCAL_LENGTH |
		cv::CALIB_FIX_ASPECT_RATIO |
		cv::CALIB_RATIONAL_MODEL |
		cv::CALIB_ZERO_TANGENT_DIST |
		cv::CALIB_FIX_K1 |
		cv::CALIB_FIX_K2 |
		cv::CALIB_FIX_K3 |
		cv::CALIB_FIX_K4 |
		cv::CALIB_FIX_K5 |
		cv::CALIB_FIX_K6 |
		cv::CALIB_FIX_S1_S2_S3_S4 |
		cv::CALIB_FIX_TAUX_TAUY |
		cv::CALIB_SAME_FOCAL_LENGTH;
		//cv::CALIB_THIN_PRISM_MODEL | 
		//cv::CALIB_TILTED_MODEL;
	std::vector<std::pair<std::string, std::string>> _pairedFiles;
	int _cornerWidth, _cornerHeight;
	double _squareSize;
	cv::Size _imageSize;
	cv::Rect _validRoi[2];

	struct
	{
		cv::Mat _cameraMatrix, _distCoeffs;
	} _cameraIntrinsics[2];

	cv::Mat _R, _T, _R1, _R2, _P1, _P2, _Q;

	cv::Mat _vignettingL, _vignettingR;
	bool _red;
private:
	std::string goodImageList(int x);
public:
	int SetVignettingMat(std::string vigMatLeft, std::string vigMatRight);

	int SetRedSpot(bool redSpot);

	//Pair 2 img lists, file with same name will be paired
	int SetImageListAndPair(std::vector<std::string> listFile1, std::vector<std::string> listFile2);

	int SetBoardSize(int cornerWidth, int cornerHeight, double squareSize);

	int SetCameraIntrinsics(cv::Mat cameraMatrix1, cv::Mat distCoeffs1, cv::Mat cameraMatrix2, cv::Mat distCoeffs2);

	int Calibrate(cv::Mat &R, cv::Mat &T, cv::Mat &R1, cv::Mat &R2, cv::Mat &P1, cv::Mat &P2, cv::Mat &Q);

	int SaveParams(std::string resultFile, std::string rectifyDataDir);

	int ShowResults();
};

#endif //__CAMERA_CALIBRATION_STEREO__