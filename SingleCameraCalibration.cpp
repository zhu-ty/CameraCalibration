#include "SingleCameraCalibration.h"

#include <future>
#include <condition_variable>

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
	this->_listFile.resize(0);
	return 0;
}

int SingleCalibrater::SetImageList(std::vector<std::string> listFile)
{
	this->_xmlListFile = "";
	this->_listFile = listFile;
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
	if (_xmlListFile == "" && _listFile.size() == 0)
	{
		SysUtil::errorOutput("SingleCalibrater::Calibrate xmlListFile not set.");
		return -1;
	}
	if (_cornerWidth == 0 || _cornerHeight == 0 || _squareSize == 0)
	{
		SysUtil::errorOutput("SingleCalibrater::Calibrate corner info not set.");
		return -1;
	}

	std::vector<std::string> fileList = _listFile;
	if (_xmlListFile != "")
		if(readStringList(_xmlListFile, fileList) != 0)
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
		//bool found = cv::findChessboardCorners(img, boardSize, pointBuf,
		//	cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
		//if (found)
		//{
		//	cv::Mat imgGray;
		//	cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
		//	cv::cornerSubPix(imgGray, pointBuf, cv::Size(11, 11), cv::Size(-1, -1),
		//		cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
		//	_imagePoints.push_back(pointBuf);
		//	SysUtil::infoOutput("Found corners in image " + fileList[i]);

		//	
		//	//cv::drawChessboardCorners(img, boardSize, pointBuf, true);
		//	//cv::imshow("hello", img);
		//	//cv::waitKey(0);
		//}
		//else
		//{
		//	SysUtil::warningOutput("Corners not found in image " + fileList[i]);
		//}
		int found = SingleCalibrater::findChessboardCornersTimeout(img, boardSize, pointBuf,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK, FIND_POINT_TIMEOUT_MS);
		if (found == 1)
		{
			SysUtil::infoOutput(SysUtil::format("Found corners (%d) in image ", pointBuf.size()) + fileList[i]);
			_imagePoints.push_back(pointBuf);
		}
		else if (found == 0)
		{
			SysUtil::warningOutput("Corners not found in image " + fileList[i]);
		}
		else if (found == -1)
		{
			SysUtil::warningOutput("Corners founding in image " + fileList[i] + " over time!");
		}
		else
		{
			SysUtil::errorOutput("Unknown error in founding corners in image " + fileList[i]);
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
	//_cameraMatrix.at<double>(0, 0) = ASPECT_RATIO; //TODO: now only 4:3 camera
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

	//for (int i = 0; i < fileList.size(); i++)
	//{
	//	cv::Mat img = cv::imread(fileList[i]);
	//	cv::Mat tmp = img.clone();
	//	cv::undistort(img, tmp, _cameraMatrix, _distCoeffs);
	//}

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
	//fs << "fix_aspect_ratio" << ASPECT_RATIO;
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



int SingleCalibrater::findChessboardCornersTimeout(cv::Mat &img, cv::Size &boardSize, std::vector<cv::Point2f> &out_pointList, int flag, int timeoutMs)
{
	std::vector<cv::Point2f> tmp_out_pointList;


	//std::mutex m;
	//std::condition_variable cv;
	bool retValue;
	bool running = true;
#if defined(_WIN32) || defined(WIN32)
	int thread_id;
#endif
	std::thread t([&]()
	{
		retValue = false;
#if defined(_WIN32) || defined(WIN32)
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		thread_id = GetCurrentThreadId();
#else
		int *x;
		pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,x);
#endif

		bool found = cv::findChessboardCorners(img, boardSize, tmp_out_pointList, flag);
		if (found)
		{
			cv::Mat imgGray;
			cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
			cv::cornerSubPix(imgGray, tmp_out_pointList, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
			retValue = true;
		}
		else
		{
			retValue = false;
		}
		running = false;

	});
#if !(defined(_WIN32) || defined(WIN32))
	pthread_t thread_handle = t.native_handle();
#endif
	t.detach();


	int i = 0;
	while (true)
	{
		if (i * 10 >= timeoutMs)
		{
#if defined(_WIN32) || defined(WIN32)
			//int result = TerminateThread(th, -10);
			//int result = TerminateThread(t.native_handle(), -10);
			DWORD dwDesiredAccess = PROCESS_TERMINATE;
			BOOL  bInheritHandle = FALSE;
			HANDLE hProcess = OpenThread(dwDesiredAccess, bInheritHandle, thread_id);
			
			if (hProcess == NULL)
				return -3;
			int result = TerminateThread(hProcess, -10);
			CloseHandle(hProcess);
			if (result == 0)
			{
				int result2 = GetLastError();
				SysUtil::errorOutput(SysUtil::format("TerminateThread error with code %d", result2));
			}
#else
			pthread_cancel(thread_handle);
#endif
			return -1;
		}
		if (running)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			i++;
		}
		else
		{
			if (retValue)
				out_pointList = tmp_out_pointList;
			return (retValue) ? 1 : 0;
		}
	}
	return -2;
}
