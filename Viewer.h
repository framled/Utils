#ifndef SOURCE_DIRECTORY__VIEWER_H_
#define SOURCE_DIRECTORY__VIEWER_H_
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <iostream>
#include "Utilities.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

template<typename PointT>
class Viewer
{
public:
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::ConstPtr ConstPtr;
	typedef typename PointCloud::Ptr CloudPtr;
	Viewer();
	void run();
	void updateCloud(CloudPtr& cloud);
	void addCloud(ConstPtr cloud);
	virtual ~Viewer();
private:

	void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*);
	void mouse_callback(const pcl::visualization::MouseEvent& event, void*);
	void leftMouse_pressed(const pcl::visualization::MouseEvent& event);
	void mouse_dragged(const pcl::visualization::MouseEvent& event);
	void rotatePointCloud(const pcl::PointXY& first, const pcl::PointXY& second);
	void save(CloudPtr& cloud);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	CloudPtr ptr_cloud;
	std::string lastKeyPressed;
	pcl::PointXY firstPoint;
	pcl::PointXY currentPosition;
	pcl::PointXY secondPoint;
	bool isFirstPoint;
	bool isRotateActive;
};

#endif /*SOURCE_DIRECTORY__UTILITIES_H_*/
