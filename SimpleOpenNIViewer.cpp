/*
 * SimpleOpenniViewer.cpp
 *
 *  Created on: 21-12-2015
 *      Author: felipe-narvaez
 */

#include "SimpleOpenNIViewer.h"

SimpleOpenNIViewer::~SimpleOpenNIViewer() {
	// TODO Auto-generated destructor stub
}

SimpleOpenNIViewer::SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {

}

void SimpleOpenNIViewer::cloud_cb_ (const PointCloudT::ConstPtr &cloud)
{
	if (!viewer.wasStopped()) {

		// Downsampling the point cloud
		//PointCloudT::Ptr cloud_filtered (new PointCloudT);
		//pcl::VoxelGrid<PointT> vg;
		//vg.setInputCloud (cloud);
		//vg.setLeafSize (0.04f, 0.04f, 0.04f);	// Massive downsampling
		//vg.filter (*cloud_filtered);

		#ifdef use_CloudViewer
		viewer.showCloud(cloud, "cloud");
		#else
		if(!viewer.updatePointCloud(cloud, "cloud")){	// Update all other times

			viewer.addPointCloud (cloud, "cloud");	// Add the point cloud once
			PCL_WARN("First loop; adding cloud\n");


			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
			viewer.addCoordinateSystem (1, "cloud", 0);
			viewer.initCameraParameters();
			//viewer.spinOnce();
		}
		#endif
	}
}

void SimpleOpenNIViewer::run ()
{
	pcl::Grabber* interface = new pcl::OpenNIGrabber();
	boost::function<void (const PointCloudT::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

	interface->registerCallback (f);
	interface->start ();

	while (!viewer.wasStopped()) {
		boost::this_thread::sleep (boost::posix_time::seconds (1));
	}
	interface->stop ();
}


