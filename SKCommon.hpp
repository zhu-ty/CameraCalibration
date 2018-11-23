/**
@brief class for basic system operations
@author: zhu-ty
@date: Aug 8, 2018
*/

#ifndef __SHADOWK_COMMON__
#define __SHADOWK_COMMON__

#pragma once
// include stl
#include <memory>
#include <vector>
#include <thread>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <queue>
#include <cmath>
#include <cstring>
#include <sstream>
#include <string>
#include <ctime>
#include <algorithm>
#if defined(_WIN32) || defined(WIN32)
#include <windows.h>
#include <direct.h>
#include <time.h>
#include <Winsock2.h>
#else
#include <signal.h>
#include <sys/time.h>
#include <stdarg.h>
#include <pthread.h>
#endif
#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#ifndef WIN32
#define BLACK_TEXT(x) "\033[30;1m" << x << "\033[0m"
#define RED_TEXT(x) "\033[31;1m" << x << "\033[0m"
#define GREEN_TEXT(x) "\033[32;1m" <<  x << "\033[0m"
#define YELLOW_TEXT(x) "\033[33;1m" << x << "\033[0m"
#define BLUE_TEXT(x) "\033[34;1m" << x << "\033[0m"
#define MAGENTA_TEXT(x) "\033[35;1m" << x << "\033[0m"
#define CYAN_TEXT(x) "\033[36;1m" << x << "\033[0m"
#define WHITE_TEXT(x) "\033[37;1m" << x << "\033[0m"
#endif

#ifdef WIN32
#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
#define DELTA_EPOCH_IN_MICROSECS 11644473600000000Ui64
#else
#define DELTA_EPOCH_IN_MICROSECS 11644473600000000ULL
#endif


#endif

#define DEBUG_STRING std::string("file: " +   \
((std::string(__FILE__).find_last_of("/") != std::string::npos || std::string(__FILE__).find_last_of("\\") != std::string::npos) ?   \
 (  \
(std::string(__FILE__).find_last_of("\\") != std::string::npos) ? \
 std::string(__FILE__).erase(0, std::string(__FILE__).find_last_of("\\") + 1) : \
 std::string(__FILE__).erase(0, std::string(__FILE__).find_last_of("/") + 1) \
 )  \
 : std::string(__FILE__))  \
+ " line: " + std::to_string(__LINE__) +" func: " + std::string(__func__) +"\n") 
 

class SysUtil {
private:
	enum class ConsoleColor {
		red = 12,
		blue = 9,
		green = 10,
		yellow = 14,
		white = 15,
		pink = 13,
		cyan = 11
	};
public:

	/***********************************************************/
	/*                 Replace string function                 */
	/***********************************************************/
	static inline std::string stringReplace(std::string strBase, std::string strSrc, std::string strDes)
	{
		std::string::size_type pos = 0;
		std::string::size_type srcLen = strSrc.size();
		std::string::size_type desLen = strDes.size();
		pos = strBase.find(strSrc, pos);
		while ((pos != std::string::npos))
		{
			strBase.replace(pos, srcLen, strDes);
			pos = strBase.find(strSrc, (pos + desLen));
		}
		return strBase;
	}

	/***********************************************************/
	/*             Get file name from path function            */
	/***********************************************************/
	static inline std::string getFileName(std::string path)
	{
		path = stringReplace(path, "\\", "/");
		return (path.find_last_of("/") != std::string::npos) ? path.erase(0, path.find_last_of("/") + 1) : path;
	}



	/***********************************************************/
	/*                    mkdir function                       */
	/***********************************************************/
	static int mkdir(char* dir) {
#ifdef WIN32
		_mkdir(dir);
#else
		char command[256];
		sprintf(command, "mkdir %s", dir);
		system(command);
#endif
		return 0;
	}
	static int mkdir(std::string dir) {
		return mkdir((char *)dir.c_str());
	}

	/***********************************************************/
	/*                    sleep function                       */
	/***********************************************************/
	static int sleep(size_t miliseconds) {
		std::this_thread::sleep_for(std::chrono::milliseconds(miliseconds));
		return 0;
	}

	/***********************************************************/
	/*             make colorful console output                */
	/***********************************************************/
	static int setConsoleColor(ConsoleColor color) {
#ifdef WIN32
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), static_cast<int>(color));
#endif
		return 0;
	}

	/***********************************************************/
	/*                 warning error output                    */
	/***********************************************************/
	static int errorOutput(std::string info) {
#ifdef WIN32
		SysUtil::setConsoleColor(ConsoleColor::red);
		std::cerr << "ERROR: " << info.c_str() << std::endl;
		SysUtil::setConsoleColor(ConsoleColor::white);
#else
		std::cerr << RED_TEXT("ERROR: ") << RED_TEXT(info.c_str())
			<< std::endl;
#endif
		return 0;
	}

	static int warningOutput(std::string info) {
#ifdef WIN32
		SysUtil::setConsoleColor(ConsoleColor::yellow);
		std::cerr << "WARNING: " << info.c_str() << std::endl;
		SysUtil::setConsoleColor(ConsoleColor::white);
#else
		std::cerr << YELLOW_TEXT("WARNING: ") << YELLOW_TEXT(info.c_str())
			<< std::endl;
#endif
		return 0;
	}

	static int infoOutput(std::string info) {
#ifdef WIN32
		SysUtil::setConsoleColor(ConsoleColor::green);
		std::cerr << "INFO: " << info.c_str() << std::endl;
		SysUtil::setConsoleColor(ConsoleColor::white);
#else
		std::cerr << GREEN_TEXT("INFO: ") << GREEN_TEXT(info.c_str())
			<< std::endl;
#endif
		return 0;
	}

	static int debugOutput(std::string info) {
#ifdef WIN32
		SysUtil::setConsoleColor(ConsoleColor::pink);
		std::cerr << "DEBUG INFO: " << info.c_str() << std::endl;
		SysUtil::setConsoleColor(ConsoleColor::white);
#else
		std::cerr << MAGENTA_TEXT("DEBUG INFO: ") << MAGENTA_TEXT(info.c_str())
			<< std::endl;
#endif
		return 0;
	}

	struct timezone
	{
		int  tz_minuteswest; // minutes W of Greenwich  
		int  tz_dsttime;     // type of dst correction
	};

#ifdef WIN32
	static int gettimeofday(struct timeval *tv, struct timezone *tz)
	{
		FILETIME ft;
		uint64_t tmpres = 0;
		static int tzflag = 0;


		if (tv)
		{
#ifdef _WIN32_WCE
			SYSTEMTIME st;
			GetSystemTime(&st);
			SystemTimeToFileTime(&st, &ft);
#else
			GetSystemTimeAsFileTime(&ft);
#endif


			tmpres |= ft.dwHighDateTime;
			tmpres <<= 32;
			tmpres |= ft.dwLowDateTime;


			/*converting file time to unix epoch*/
			tmpres /= 10;  /*convert into microseconds*/
			tmpres -= DELTA_EPOCH_IN_MICROSECS;
			tv->tv_sec = (long)(tmpres / 1000000UL);
			tv->tv_usec = (long)(tmpres % 1000000UL);
		}


		if (tz) {
			if (!tzflag) {
				tzflag++;
			}
			tz->tz_minuteswest = _timezone / 60;
			tz->tz_dsttime = _daylight;
		}


		return 0;
	}
#endif

	static int64_t getCurrentTimeMicroSecond()
	{
		struct timeval tv;
		gettimeofday(&tv, NULL);
		return tv.tv_sec * (int64_t)1000000 + tv.tv_usec;
	}

	static std::string getTimeString()
	{
		time_t timep;
		time(&timep);
		char tmp[64];
		strftime(tmp, sizeof(tmp), "__%Y_%m_%d_%H_%M_%S__", localtime(&timep));
		return tmp;
	}

	static inline std::string format(const char *msg, ...)
	{
		std::size_t const STRING_BUFFER(4096);
		char text[STRING_BUFFER];
		va_list list;

		if (msg == 0)
			return std::string();

		va_start(list, msg);
#		if(GLM_COMPILER & GLM_COMPILER_VC)
		vsprintf_s(text, STRING_BUFFER, msg, list);
#		else//
		vsprintf(text, msg, list);
#		endif//
		va_end(list);

		return std::string(text);
	}
};

#endif //__SHADOWK_COMMON__