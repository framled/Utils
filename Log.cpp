/*
 * Log.cpp
 *
 *  Created on: 07-12-2015
 *      Author: felipe-narvaez
 */

#include "Log.h"

/**
 * Log class
 */
Log::Log(std::string path) {
	if (folderExist(path)) {
		buffer.open(path.c_str(), std::ios::in);
	} else {
		createFolder(path);
		buffer.open(path.c_str());
	}

	writeDate();
}
void Log::close() {
	buffer.close();
}
Log::~Log() {
	close();
}
bool Log::writeDate() {
	time_t now = time(0);
	tm *ltm = localtime(&now);
	std::string date(ctime(&now));
	write(date);
}

bool Log::write(std::string& data) {
	if (this->buffer.is_open()) {
		this->buffer << "================================================";
		this->buffer << data;
		this->buffer << "================================================";
		return true;
	}
	return false;
}
bool Log::folderExist(std::string& path) {

	try {
		boost::filesystem::path p(path);
		if (exists(p))    // does p actually exist?
				{
			if (is_directory(p))        // is p a regular file?
				return true;
		}
	} catch (const boost::filesystem::filesystem_error& ex) {
		std::cerr << ex.what() << "\n";
		return false;
	}
	return false;
}
void Log::createFolder(std::string& path) {
	boost::filesystem::path p(path);
	if (!folderExist(path))
		create_directory(p.parent_path());
}
