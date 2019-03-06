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

int SingleCalibrater::SetVignettingMat(std::string vigMat)
{
	if (vigMat != "")
		_vignetting = cv::imread(vigMat, cv::IMREAD_UNCHANGED);
	return 0;
}

int SingleCalibrater::SetRedSpot(bool redSpot)
{
	_red = redSpot;
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
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK, FIND_POINT_TIMEOUT_MS,
			_vignetting, _red);
		if (found == 1)
		{
			SysUtil::infoOutput(SysUtil::format("Found corners (%d) in image ", pointBuf.size()) + fileList[i]);
			_imagePoints.push_back(pointBuf);
		}
		else if (found == 0)
			SysUtil::warningOutput("Corners not found in image " + fileList[i]);
		else if (found == -1)
			SysUtil::warningOutput("Corners finding in image " + fileList[i] + " over time!");
		else
			SysUtil::errorOutput("Unknown error in finding corners in image " + fileList[i]);

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
	//_cameraMatrix.at<double>(0, 0) = ASPECT_RATIO; 
	_distCoeffs = cv::Mat::zeros(14, 1, CV_64F);

	std::vector<std::vector<cv::Point3f> > objectPoints(1);
	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			objectPoints[0].push_back(cv::Point3f(j*_squareSize, i*_squareSize, 0));
	objectPoints.resize(_imagePoints.size(), objectPoints[0]);
	double rms;

	//TODO: check these flags.
	rms = cv::calibrateCamera(objectPoints, _imagePoints, _imageSize, _cameraMatrix, _distCoeffs, rvecs, tvecs, flags, cv::TermCriteria(cv::TermCriteria::EPS, 1000, 1e-7));
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
		int _flag = flags;
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
	fs << "flags" << flags;
	fs << "camera_matrix" << _cameraMatrix;
	fs << "distortion_coefficients" << _distCoeffs;

	fs.release();
	return 0;
}



int SingleCalibrater::findChessboardCornersTimeout(cv::Mat &img, cv::Size &boardSize, std::vector<cv::Point2f> &out_pointList, int flag, int timeoutMs,
	const cv::Mat &_vignetting, bool findRedROI)
{
	//pre
	cv::Rect finalRect(0, 0, img.cols, img.rows);
	if (findRedROI)
	{
		struct Contour
		{
			double area;
			cv::Point center;
			cv::Rect bRect;
			std::vector<cv::Point> data;
			Contour(std::vector<cv::Point> _data)
			{
				data = _data;
				area = cv::contourArea(data);
				bRect = cv::boundingRect(data);
				center.x = bRect.x + bRect.width / 2;
				center.y = bRect.y + bRect.height / 2;
			}
			static bool compBigger(const Contour &a, const Contour &b)
			{
				return a.area > b.area;
			}
		};
		cv::Mat lab, arrayLab[3], A;

		//{
		//	cv::Mat hsv, hsv_array[3];
		//	cv::split(img, hsv_array);
		//	cv::Mat s3;
		//	hsv_array[1].convertTo(s3, CV_8UC1, 1.16);
		//	hsv_array[1] = s3;
		//	cv::merge(hsv_array, 3, img);

		//	
		//	cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
		//	cv::split(hsv, hsv_array);
		//	cv::Mat s2;
		//	hsv_array[1].convertTo(s2, CV_8UC1, 5);
		//	hsv_array[1] = s2;
		//	cv::merge(hsv_array, 3, hsv);
		//	cv::cvtColor(hsv, img, cv::COLOR_HSV2BGR);

		//	
		//}


		cv::cvtColor(img, lab, cv::COLOR_BGR2Lab);
		cv::split(lab, arrayLab);

		cv::equalizeHist(arrayLab[1], arrayLab[1]);
		cv::threshold(arrayLab[1], A, 254, 255, cv::THRESH_BINARY);
		cv::Mat Element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
		cv::erode(A, A, Element);

		//cv::threshold(arrayLab[1], A, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
		std::vector<std::vector<cv::Point>> contours; // Vector for storing contour
		std::vector<cv::Point> max4c[4];
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(A, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		std::vector<Contour> cs;
		if (contours.size() >= 4)
		{
			for (int j = 0; j < contours.size(); j++)
			{
				Contour c(contours[j]);
				if (c.area > FIND_POINT_MIN_AREA_REDDOT)
					cs.push_back(c);
			}
			if (cs.size() >= 4)
			{
				std::sort(cs.begin(), cs.end(), Contour::compBigger);
				std::vector<cv::Point> finalRectPt(4);
				for (int j = 0; j < 4; j++)
				{
					finalRectPt[j] = cs[j].center;
				}
				finalRect = cv::boundingRect(finalRectPt);
			}
		}
	}
	cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
	if (!_vignetting.empty() && _vignetting.size == img.size)
	{
		cv::Mat fimg;
		img.convertTo(fimg, CV_32FC1);
		img = fimg.mul(_vignetting);
		img.convertTo(img, CV_8UC1);
	}
	if (findRedROI)
	{
		cv::Mat tmp;
		img(finalRect).copyTo(tmp);
		img = tmp;
	}



	std::vector<cv::Point2f> tmp_out_pointList;
	bool retValue;
	bool running = true;
#if defined(_WIN32) || defined(WIN32)
	int thread_id;
#else
	bool can_detach = false;
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
		can_detach = true;
#endif
		bool found = cv::findChessboardCorners(img, boardSize, tmp_out_pointList, flag);
		if (found)
		{
			cv::Mat imgGray = img;
			//cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

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
	while(!can_detach)
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
			{
				out_pointList = tmp_out_pointList;
				if (findRedROI)
				{
					for (int j = 0; j < out_pointList.size(); j++)
					{
						out_pointList[j].x += (double)finalRect.x;
						out_pointList[j].y += (double)finalRect.y;
					}
				}
			}
			return (retValue) ? 1 : 0;
		}
	}
	return -2;
}
