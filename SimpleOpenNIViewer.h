/*
 * SimpleOpenniViewer.h
 *
 *  Created on: 21-12-2015
 *      Author: felipe-narvaez
 */

#ifndef UTILS_SIMPLEOPENNIVIEWER_H_
#define UTILS_SIMPLEOPENNIVIEWER_H_

#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#define use_CloudViewer	// [un]comment this line to switch between CloudViewer and PCLVisualizer



class SimpleOpenNIViewer {
public:
	typedef pcl::PointXYZRGBA PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;

	SimpleOpenNIViewer();
	void run ();
	void cloud_cb_ (const PointCloudT::ConstPtr &cloud);
	virtual ~SimpleOpenNIViewer();
private:
	#ifdef use_CloudViewer
	pcl::visualization::CloudViewer viewer;
	#else
	pcl::visualization::PCLVisualizer viewer;
	#endif
};

#endif /* UTILS_SIMPLEOPENNIVIEWER_H_ */
