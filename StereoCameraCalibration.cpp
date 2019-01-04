#include "StereoCameraCalibration.h"
#include "SingleCameraCalibration.h"

std::string StereoCalibrater::goodImageList(int x)
{
	return (x % 2 == 0) ? _pairedFiles[x / 2].first : _pairedFiles[x / 2].second;
}

int StereoCalibrater::SetVignettingMat(std::string& vigMatLeft, std::string& vigMatRight)
{
	if (vigMatLeft != "")
		_vignettingL = cv::imread(vigMatLeft, cv::IMREAD_UNCHANGED);
	if (vigMatRight != "")
		_vignettingR = cv::imread(vigMatRight, cv::IMREAD_UNCHANGED);
	return 0;
}

int StereoCalibrater::SetRedSpot(bool redSpot)
{
	_red = redSpot;
	return 0;
}

int StereoCalibrater::SetImageListAndPair(std::vector<std::string> listFile1, std::vector<std::string> listFile2)
{
	int i = 0, j = 0;
	this->_pairedFiles.resize(0);
	while (true)
	{
		if (i == listFile1.size() || j == listFile2.size())
			break;
		
		int comp = SysUtil::getFileName(listFile1[i]).compare(SysUtil::getFileName(listFile2[j]));
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
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK, FIND_POINT_TIMEOUT_MS,
			_vignettingL, _red);
		if (found == 0)
		{
			SysUtil::warningOutput("Corners not found in image " + this->_pairedFiles[i].first);
			continue;
		}
		else if (found == -1)
		{
			SysUtil::warningOutput("Corners finding in image " + this->_pairedFiles[i].first + " over time!");
			continue;
		}
		else if(found != 1)
		{
			SysUtil::errorOutput("Unknown error in finding corners in image " + this->_pairedFiles[i].first);
			continue;
		}
		//SysUtil::infoOutput(SysUtil::format("Found corners (%d) in image ", pointBuf1.size()) + this->_pairedFiles[i].first);

		cv::Mat img2 = cv::imread(this->_pairedFiles[i].second);
		found = SingleCalibrater::findChessboardCornersTimeout(img2, boardSize, pointBuf2,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK, FIND_POINT_TIMEOUT_MS,
			_vignettingR, _red);
		if (found == 0)
		{
			SysUtil::warningOutput("Corners not found in image " + this->_pairedFiles[i].second);
			continue;
		}
		else if (found == -1)
		{
			SysUtil::warningOutput("Corners finding in image " + this->_pairedFiles[i].second + " over time!");
			continue;
		}
		else if (found != 1)
		{
			SysUtil::errorOutput("Unknown error in finding corners in image " + this->_pairedFiles[i].second);
			continue;
		}
		SysUtil::infoOutput(SysUtil::format("[Stereo] Found corners (%d) in image ", pointBuf2.size()) + SysUtil::getFileName(this->_pairedFiles[i].second));
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
		_flag,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 1000, 1e-5));
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
			cv::undistortPoints(imgpt[k], imgpt[k],
				_cameraIntrinsics[k]._cameraMatrix, _cameraIntrinsics[k]._distCoeffs,
				cv::Mat(), _cameraIntrinsics[k]._cameraMatrix);
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

	cv::stereoRectify(_cameraIntrinsics[0]._cameraMatrix, _cameraIntrinsics[0]._distCoeffs,
		_cameraIntrinsics[1]._cameraMatrix, _cameraIntrinsics[1]._distCoeffs,
		_imageSize, _R, _T, _R1, _R2, _P1, _P2, _Q,
		cv::CALIB_ZERO_DISPARITY, 1, _imageSize, &_validRoi[0], &_validRoi[1]);

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
			//<< (_flag & cv::CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess\n" : "")
			//<< (_flag & cv::CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio\n" : "")
			//<< (_flag & cv::CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point\n" : "")
			//<< (_flag & cv::CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist\n" : "")
			//<< (_flag & cv::CALIB_TILTED_MODEL ? " +calib_tilted_model\n" : "")
			//<< (_flag & cv::CALIB_FIX_INTRINSIC ? " +CALIB_FIX_INTRINSIC\n" : "")
			//<< (_flag & cv::CALIB_SAME_FOCAL_LENGTH ? " +CALIB_SAME_FOCAL_LENGTH\n" : "")
			//<< (_flag & cv::CALIB_FIX_K1 ? " +fix_k1\n" : "")
			//<< (_flag & cv::CALIB_FIX_K2 ? " +fix_k2\n" : "")
			//<< (_flag & cv::CALIB_FIX_K3 ? " +fix_k3\n" : "")
			//<< (_flag & cv::CALIB_FIX_K4 ? " +fix_k4\n" : "")
			//<< (_flag & cv::CALIB_FIX_K5 ? " +fix_k5\n" : "")

			<< (_flag & cv::CALIB_USE_INTRINSIC_GUESS ? " +CALIB_USE_INTRINSIC_GUESS\n" : "")
			<< (_flag & cv::CALIB_FIX_ASPECT_RATIO ? " +CALIB_FIX_ASPECT_RATIO\n" : "")
			<< (_flag & cv::CALIB_FIX_PRINCIPAL_POINT ? " +CALIB_FIX_PRINCIPAL_POINT\n" : "")
			<< (_flag & cv::CALIB_ZERO_TANGENT_DIST ? " +CALIB_ZERO_TANGENT_DIST\n" : "")
			<< (_flag & cv::CALIB_FIX_FOCAL_LENGTH ? " +CALIB_FIX_FOCAL_LENGTH\n" : "")
			<< (_flag & cv::CALIB_FIX_K1 ? " +CALIB_FIX_K1\n" : "")
			<< (_flag & cv::CALIB_FIX_K2 ? " +CALIB_FIX_K2\n" : "")
			<< (_flag & cv::CALIB_FIX_K3 ? " +CALIB_FIX_K3\n" : "")
			<< (_flag & cv::CALIB_FIX_K4 ? " +CALIB_FIX_K4\n" : "")
			<< (_flag & cv::CALIB_FIX_K5 ? " +CALIB_FIX_K5\n" : "")
			<< (_flag & cv::CALIB_FIX_K6 ? " +CALIB_FIX_K6\n" : "")
			<< (_flag & cv::CALIB_RATIONAL_MODEL ? " +CALIB_RATIONAL_MODEL\n" : "")
			<< (_flag & cv::CALIB_THIN_PRISM_MODEL ? " +CALIB_THIN_PRISM_MODEL\n" : "")
			<< (_flag & cv::CALIB_FIX_S1_S2_S3_S4 ? " +CALIB_FIX_S1_S2_S3_S4\n" : "")
			<< (_flag & cv::CALIB_TILTED_MODEL ? " +CALIB_TILTED_MODEL\n" : "")
			<< (_flag & cv::CALIB_FIX_TAUX_TAUY ? " +CALIB_FIX_TAUX_TAUY\n" : "")
			<< (_flag & cv::CALIB_USE_QR ? " +CALIB_USE_QR\n" : "")
			<< (_flag & cv::CALIB_FIX_TANGENT_DIST ? " +CALIB_FIX_TANGENT_DIST\n" : "")
			<< (_flag & cv::CALIB_FIX_INTRINSIC ? " +CALIB_FIX_INTRINSIC\n" : "")
			<< (_flag & cv::CALIB_SAME_FOCAL_LENGTH ? " +CALIB_SAME_FOCAL_LENGTH\n" : "");

		fs.writeComment(flagsStringStream.str());
	}
	fs << "flags" << _flag;
	fs << "camera_matrix_1" << _cameraIntrinsics[0]._cameraMatrix;
	fs << "distortion_coefficients_1" << _cameraIntrinsics[0]._distCoeffs;
	fs << "camera_matrix_2" << _cameraIntrinsics[1]._cameraMatrix;
	fs << "distortion_coefficients_2" << _cameraIntrinsics[1]._distCoeffs;

	fs << "R" << _R;
	fs << "T" << _T;
	fs << "R1" << _R1;
	fs << "R2" << _R2;
	fs << "P1" << _P1;
	fs << "P2" << _P2;
	fs << "Q" << _Q;

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

	fs.open(rectifyDataDir + "/extrinsics.yml", cv::FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << _R << "T" << _T << "R1" << _R1 << "R2" << _R2 << "P1" << _P1 << "P2" << _P2 << "Q" << _Q;
		fs.release();
	}
	else
		SysUtil::errorOutput(DEBUG_STRING + "Error: can not save the extrinsic parameters");

	return 0;
}


int StereoCalibrater::ShowResults()
{
	bool isVerticalStereo = fabs(_P2.at<double>(1, 3)) > fabs(_P2.at<double>(0, 3));
	cv::Mat rmap[2][2];
	//Precompute maps for cv::remap()
	cv::initUndistortRectifyMap(_cameraIntrinsics[0]._cameraMatrix, _cameraIntrinsics[0]._distCoeffs, _R1, _P1, _imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	cv::initUndistortRectifyMap(_cameraIntrinsics[1]._cameraMatrix, _cameraIntrinsics[1]._distCoeffs, _R2, _P2, _imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	cv::Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo)
	{
		sf = 600. / MAX(_imageSize.width, _imageSize.height);
		w = cvRound(_imageSize.width*sf);
		h = cvRound(_imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(_imageSize.width, _imageSize.height);
		w = cvRound(_imageSize.width*sf);
		h = cvRound(_imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	for (int i = 0; i < _pairedFiles.size(); i++)
	{
		for (int k = 0; k < 2; k++)
		{
			cv::Mat img = cv::imread(goodImageList(i * 2 + k), 0), rimg, cimg;
			cv::remap(img, rimg, rmap[k][0], rmap[k][1], cv::INTER_LINEAR);
			cv::cvtColor(rimg, cimg, cv::COLOR_GRAY2BGR);
			cv::Mat canvasPart = !isVerticalStereo ? canvas(cv::Rect(w*k, 0, w, h)) : canvas(cv::Rect(0, h*k, w, h));
			cv::resize(cimg, canvasPart, canvasPart.size(), 0, 0, cv::INTER_AREA);
			cv::Rect vroi(cvRound(_validRoi[k].x*sf), cvRound(_validRoi[k].y*sf),
				cvRound(_validRoi[k].width*sf), cvRound(_validRoi[k].height*sf));
			cv::rectangle(canvasPart, vroi, cv::Scalar(0, 0, 255), 3, 8);
		}


		if (!isVerticalStereo)
			for (int j = 0; j < canvas.rows; j += 16)
				line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
		else
			for (int j = 0; j < canvas.cols; j += 16)
				line(canvas, cv::Point(j, 0), cv::Point(j, canvas.rows), cv::Scalar(0, 255, 0), 1, 8);
		imshow("rectified", canvas);
		char c = (char)cv::waitKey();
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}
	return 0;
}


//TODO View result