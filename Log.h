/*
 * Log.h
 *
 *  Created on: 07-12-2015
 *      Author: felipe-narvaez
 */

#ifndef UTILS_LOG_H_
#define UTILS_LOG_H_
#include <iostream>
#include <fstream>
#include <ctime>
#include <boost/filesystem.hpp>

class buffer;

class Log {
public:
	Log(std::string path);
	void close();
	bool write(std::string& data);
	bool folderExist(std::string& path);
	void createFolder(std::string& path);
	bool writeDate();
	virtual ~Log();
private:
	std::ofstream buffer;
};
#endif /* UTILS_LOG_H_ */
