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
	if (_cornerWidth == 0 || _cornerHeight == 0 || _squareSize == 0)
	{
		SysUtil::errorOutput(DEBUG_STRING + "corner info not set.");
		return -1;
	}
	_imageSize = cv::Size(cv::imread(_pairedFiles[0].first).size());
	cv::Size boardSize(_cornerWidth, _cornerHeight);
	std::vector<std::vector<cv::Point2f>> pointList_1;
	std::vector<std::vector<cv::Point2f>> pointList_2;
	for (int i = 0; i < _pairedFiles.size(); i++)
	{

		std::vector<cv::Point2f> pointBuf1, pointBuf2;
		cv::Mat img1 = cv::imread(this->_pairedFiles[i].first);
		int found = SingleCalibrater::findChessboardCornersTimeout(img1, boardSize, pointBuf1,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK, FIND_POINT_TIMEOUT_MS);
		if (found == 0)
		{
			SysUtil::warningOutput("Corners not found in image " + this->_pairedFiles[i].first);
			continue;
		}
		else if (found == -1)
		{
			SysUtil::warningOutput("Corners founding in image " + this->_pairedFiles[i].first + " over time!");
			continue;
		}
		else
		{
			SysUtil::errorOutput("Unknown error in founding corners in image " + this->_pairedFiles[i].first);
			continue;
		}

		cv::Mat img2 = cv::imread(this->_pairedFiles[i].second);
		found = SingleCalibrater::findChessboardCornersTimeout(img2, boardSize, pointBuf2,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK, FIND_POINT_TIMEOUT_MS);
		if (found == 0)
		{
			SysUtil::warningOutput("Corners not found in image " + this->_pairedFiles[i].second);
			continue;
		}
		else if (found == -1)
		{
			SysUtil::warningOutput("Corners founding in image " + this->_pairedFiles[i].second + " over time!");
			continue;
		}
		else
		{
			SysUtil::errorOutput("Unknown error in founding corners in image " + this->_pairedFiles[i].second);
			continue;
		}
		pointList_1.push_back(pointBuf1);
		pointList_2.push_back(pointBuf2);
	}
	if (pointList_1.size() <= 0)
	{
		SysUtil::errorOutput(DEBUG_STRING + "0 corners found in the list.");
		return -1;
	}
	SysUtil::infoOutput("Running stereo calibration ...");
	std::vector<std::vector<cv::Point3f> > objectPoints(1);
	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			objectPoints[0].push_back(cv::Point3f(j*_squareSize, i*_squareSize, 0));
	objectPoints.resize(pointList_1.size(), objectPoints[0]);

	cv::Mat _E, _F;


	double rms = stereoCalibrate(objectPoints, pointList_1, pointList_2,
		_cameraIntrinsics[0]._cameraMatrix, _cameraIntrinsics[0]._distCoeffs,
		_cameraIntrinsics[1]._cameraMatrix, _cameraIntrinsics[1]._distCoeffs,
		_imageSize, _R, _T, _E, _F,
		_flag);
	SysUtil::infoOutput(SysUtil::format("StereoCalibrater::Calibrate Re-projection error reported by stereoCalibrate: %f", rms));

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	std::vector<cv::Vec3f> lines[2];
	for (int i = 0; i < pointList_1.size(); i++)
	{
		int npt = (int)pointList_1[i].size();
		cv::Mat imgpt[2];
		for (int k = 0; k < 2; k++)
		{
			imgpt[k] = cv::Mat((k == 0) ? (pointList_1[i]) : (pointList_2[i]));
			undistortPoints(imgpt[k], imgpt[k], _cameraIntrinsics[k]._cameraMatrix, _cameraIntrinsics[k]._distCoeffs);
			computeCorrespondEpilines(imgpt[k], k + 1, _F, lines[k]);
		}
		for (int j = 0; j < npt; j++)
		{
			double errij = fabs(pointList_1[i][j].x*lines[1][j][0] +
				pointList_1[i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(pointList_2[i][j].x*lines[0][j][0] +
					pointList_2[i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	SysUtil::infoOutput(SysUtil::format("StereoCalibrater::Calibrate Average epipolar err = %f", err / npoints));

	cv::Rect validRoi[2];
	

	cv::stereoRectify(_cameraIntrinsics[0]._cameraMatrix, _cameraIntrinsics[0]._distCoeffs,
		_cameraIntrinsics[1]._cameraMatrix, _cameraIntrinsics[1]._distCoeffs,
		_imageSize, _R, _T, _R1, _R2, _P1, _P2, _Q,
		cv::CALIB_ZERO_DISPARITY, 1, _imageSize, &validRoi[0], &validRoi[1]);

	R = _R;
	T = _T;
	R1 = _R1;
	R2 = _R2;
	P1 = _P1;
	P2 = _P2;
	Q = _Q;

	SysUtil::infoOutput("StereoCalibrater::Calibrate Everything is now computed.");

	return 0;
}

int StereoCalibrater::SaveParams(std::string resultFile, std::string rectifyDataDir)
{
	cv::FileStorage fs(resultFile, cv::FileStorage::WRITE);
	fs << "image_width" << _imageSize.width;
	fs << "image_height" << _imageSize.height;
	fs << "board_width" << _cornerWidth;
	fs << "board_height" << _cornerHeight;
	fs << "square_size" << _squareSize;
	//fs << "fix_aspect_ratio" << ASPECT_RATIO;
	if (_flag)
	{
		std::stringstream flagsStringStream;
		flagsStringStream << "flags:"
			<< (_flag & cv::CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "")
			<< (_flag & cv::CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "")
			<< (_flag & cv::CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "")
			<< (_flag & cv::CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "")
			<< (_flag & cv::CALIB_TILTED_MODEL ? " +calib_tilted_model" : "")
			<< (_flag & cv::CALIB_FIX_INTRINSIC ? " +CALIB_FIX_INTRINSIC" : "")
			<< (_flag & cv::CALIB_SAME_FOCAL_LENGTH ? " +CALIB_SAME_FOCAL_LENGTH" : "")
			<< (_flag & cv::CALIB_FIX_K1 ? " +fix_k1" : "")
			<< (_flag & cv::CALIB_FIX_K2 ? " +fix_k2" : "")
			<< (_flag & cv::CALIB_FIX_K3 ? " +fix_k3" : "")
			<< (_flag & cv::CALIB_FIX_K4 ? " +fix_k4" : "")
			<< (_flag & cv::CALIB_FIX_K5 ? " +fix_k5" : "");
		fs.writeComment(flagsStringStream.str());
	}
	fs << "flags" << _flag;
	fs << "camera_matrix_1" << _cameraIntrinsics[0]._cameraMatrix;
	fs << "distortion_coefficients_1" << _cameraIntrinsics[0]._distCoeffs;
	fs << "camera_matrix_2" << _cameraIntrinsics[1]._cameraMatrix;
	fs << "distortion_coefficients_2" << _cameraIntrinsics[1]._distCoeffs;

	fs.release();

	SysUtil::mkdir(rectifyDataDir);

	fs.open(rectifyDataDir + "/intrinsics.yml", cv::FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << _cameraIntrinsics[0]._cameraMatrix << "D1" << _cameraIntrinsics[0]._distCoeffs <<
			"M2" << _cameraIntrinsics[1]._cameraMatrix << "D2" << _cameraIntrinsics[1]._distCoeffs;
		fs.release();
	}
	else
		SysUtil::errorOutput(DEBUG_STRING + "Error: can not save the intrinsic parameters");

	fs.open(rectifyDataDir + "extrinsics.yml", cv::FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << _R << "T" << _T << "R1" << _R1 << "R2" << _R2 << "P1" << _P1 << "P2" << _P2 << "Q" << _Q;
		fs.release();
	}
	else
		SysUtil::errorOutput(DEBUG_STRING + "Error: can not save the extrinsic parameters");

	return 0;
}
