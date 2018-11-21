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
	std::vector<std::pair<std::string, std::string>> _pairFiles;
	int _cornerWidth, _cornerHeight;
	double _squareSize;
	cv::Size _imageSize;

	struct
	{
		cv::Mat _cameraMatrix, _distCoeffs;
	} _cameraIntrinsics[2];
public:
	int SetImageList(std::vector<std::string> listFile);

	int SetBoardSize(int cornerWidth, int cornerHeight, double squareSize);


};

#endif //__CAMERA_CALIBRATION_STEREO__