#include "SingleCameraCalibration.h"

int SingleCalibrater::readStringList(const std::string & filename, std::vector<std::string>& l)
{
	l.resize(0);
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
		return -1;
	cv::FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != cv::FileNode::SEQ)
		return -1;
	cv::FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((std::string)*it);
	return 0;
}

int SingleCalibrater::SetImageList(std::string xmlListFile)
{
	this->_xmlListFile = xmlListFile;
	return 0;
}

int SingleCalibrater::SetBoardSize(int cornerWidth, int cornerHeight, double squareSize)
{
	this->_cornerWidth = cornerWidth;
	this->_cornerHeight = cornerHeight;
	this->_squareSize = squareSize;
	return 0;
}

int SingleCalibrater::Calibrate(cv::Mat & cameraMatrix, cv::Mat & distCoeffs)
{
	if (_xmlListFile == "")
	{
		SysUtil::errorOutput("SingleCalibrater::Calibrate xmlListFile not set.");
		return -1;
	}
	if (_cornerWidth == 0 || _cornerHeight == 0 || _squareSize == 0)
	{
		SysUtil::errorOutput("SingleCalibrater::Calibrate corner info not set.");
		return -1;
	}

	std::vector<std::string> fileList;
	if (readStringList(_xmlListFile, fileList) != 0)
	{
		SysUtil::errorOutput("SingleCalibrater::Calibrate file list read error.");
		return -1;
	}

	_imageSize = cv::Size(cv::imread(fileList[0]).size());
	cv::Size boardSize(_cornerWidth, _cornerHeight);

	for (int i = 0; i < fileList.size(); i++)
	{
		std::vector<cv::Point2f> pointBuf;
		cv::Mat img = cv::imread(fileList[i]);
		bool found = cv::findChessboardCorners(img, boardSize, pointBuf,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
		if (found)
		{
			cv::Mat imgGray;
			cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
			cv::cornerSubPix(imgGray, pointBuf, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
			_imagePoints.push_back(pointBuf);
			SysUtil::infoOutput("Found corners in image " + fileList[i]);
		}
		else
		{
			SysUtil::warningOutput("Corners not found in image " + fileList[i]);
		}
	}
	if (_imagePoints.size() <= 0)
	{
		SysUtil::errorOutput("SingleCalibrater::Calibrate 0 corners found in the list.");
		return -1;
	}

	//run calibration
	std::vector<cv::Mat> rvecs, tvecs;
	std::vector<float> reprojErrs;
	double totalAvgErr = 0;

	_cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	_cameraMatrix.at<double>(0, 0) = ASPECT_RATIO; //TODO: now only 4:3 camera
	_distCoeffs = cv::Mat::zeros(14, 1, CV_64F);

	std::vector<std::vector<cv::Point3f> > objectPoints(1);
	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			objectPoints[0].push_back(cv::Point3f(j*_squareSize, i*_squareSize, 0));
	objectPoints.resize(_imagePoints.size(), objectPoints[0]);
	double rms;

	//TODO: check these flags.
	rms = cv::calibrateCamera(objectPoints, _imagePoints, _imageSize, _cameraMatrix, _distCoeffs, rvecs, tvecs, flags);
	SysUtil::infoOutput(SysUtil::format("SingleCalibrater::Calibrate Re-projection error reported by calibrateCamera: %f", rms));
	if (!(cv::checkRange(_cameraMatrix) && cv::checkRange(_distCoeffs)))
	{
		SysUtil::errorOutput("SingleCalibrater::Calibrate calibration check range failed.");
		return -1;
	}
	cameraMatrix = _cameraMatrix;
	distCoeffs = _distCoeffs;

	return 0;
}

int SingleCalibrater::SaveParams(std::string file)
{
	cv::FileStorage fs(file, cv::FileStorage::WRITE);
	fs << "image_width" << _imageSize.width;
	fs << "image_height" << _imageSize.height;
	fs << "board_width" << _cornerWidth;
	fs << "board_height" << _cornerHeight;
	fs << "square_size" << _squareSize;
	fs << "fix_aspect_ratio" << ASPECT_RATIO;
	if (flags)
	{
		std::stringstream flagsStringStream;
		flagsStringStream << "flags:"
			<< (flags & cv::CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
			<< (flags & cv::CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
			<< (flags & cv::CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
			<< (flags & cv::CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
			<< (flags & cv::CALIB_TILTED_MODEL ? " +calib_tilted_model" : "")
			<< (flags & cv::CALIB_FIX_K1 ? " +fix_k1" : "")
			<< (flags & cv::CALIB_FIX_K2 ? " +fix_k2" : "")
			<< (flags & cv::CALIB_FIX_K3 ? " +fix_k3" : "")
			<< (flags & cv::CALIB_FIX_K4 ? " +fix_k4" : "")
			<< (flags & cv::CALIB_FIX_K5 ? " +fix_k5" : "");
		fs.writeComment(flagsStringStream.str());
	}
	fs << "flags" << flags;
	fs << "camera_matrix" << _cameraMatrix;
	fs << "distortion_coefficients" << _distCoeffs;

	fs.release();
	return 0;
}
