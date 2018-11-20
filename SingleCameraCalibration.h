/**
@brief class to calibrate single camera
@author zhu-ty
@date Nov 20, 2018
*/

#ifndef __CAMERA_CALIBRATION_SINGLE__
#define __CAMERA_CALIBRATION_SINGLE__

#include "SysUtil.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#define ASPECT_RATIO 1.0

class SingleCalibrater
{
private:
	//const int flags = cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_TILTED_MODEL | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5;
	const int flags = cv::CALIB_RATIONAL_MODEL | cv::CALIB_THIN_PRISM_MODEL | cv::CALIB_TILTED_MODEL;

	std::string _xmlListFile;
	std::vector<std::string> _listFile;
	int _cornerWidth, _cornerHeight;
	double _squareSize;

	std::vector<std::vector<cv::Point2f>> _imagePoints;
	cv::Mat _cameraMatrix, _distCoeffs;
	cv::Size _imageSize;
private:
	// 0::suc 1::fail
	int readStringList(const std::string& filename, std::vector<std::string>& l);
public:
	SingleCalibrater() {
		_xmlListFile = "";
		_cornerWidth = 0;
		_cornerHeight = 0;
		_squareSize = 0;
		_listFile.resize(0);
	};
	~SingleCalibrater() {};


	int SetImageList(std::string xmlListFile);

	int SetImageList(std::vector<std::string> listFile);

	int SetBoardSize(int cornerWidth, int cornerHeight, double squareSize);

	int Calibrate(cv::Mat &cameraMatrix, cv::Mat &distCoeffs);

	int SaveParams(std::string file);
};

#endif //__CAMERA_CALIBRATION_SINGLE__