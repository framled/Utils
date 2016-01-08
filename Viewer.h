#ifndef SOURCE_DIRECTORY__VIEWER_H_
#define SOURCE_DIRECTORY__VIEWER_H_
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <iostream>

template<typename PointT>
class Viewer
{
public:
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::ConstPtr ConstPtr;
	Viewer();
	void run();
	void addCloud(const ConstPtr cloud);
	virtual ~Viewer();
private:

	void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*);
	void mouse_callback(const pcl::visualization::MouseEvent& event, void*);
	void rotatePointCloud(const pcl::PointXY& first, const pcl::PointXY& second);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	typename pcl::PointCloud<PointT>::Ptr ptr_cloud;
	std::string lastKeyPressed;
	pcl::PointXY firstPoint;
	pcl::PointXY secondPoint;
	bool isFirstPoint;
};

#endif /*SOURCE_DIRECTORY__UTILITIES_H_*/
