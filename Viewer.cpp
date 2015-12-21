#include "Viewer.h"


template<typename PointT>
Viewer<PointT>::Viewer() : viewer(new pcl::visualization::PCLVisualizer("PCL Cloud"))
{
}

template<typename PointT> void 
Viewer<PointT>::run()
{
	viewer->registerMouseCallback(&Viewer::mouse_callback, *this);
	viewer->registerKeyboardCallback(&Viewer::keyboard_callback, *this);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));

		/*if (GetKeyState(VK_ESCAPE) < 0) {
			break;
		}*/
	}
}
/*template<typename PointT> void
Viewer<PointT>::cloud_cb_ (const ConstPtr &cloud)
 {
   if (!viewer->wasStopped()){
	   if(!viewer->updatePointCloud(cloud,"PCL Cloud")){
		   addCloud(cloud, true);
	   }
   }

 }
template<typename PointT> void
Viewer<PointT>::runOpenni(){
	pcl::Grabber* interface = new pcl::OpenNIGrabber();

	boost::function<void (const ConstPtr&)> f = boost::bind (&Viewer::cloud_cb_, this, _1);

	interface->registerCallback (f);

	interface->start ();
	run();
	interface->stop ();
}*/
template<typename PointT> void 
Viewer<PointT>::addCloud(const ConstPtr& cloud, bool isColored)
{
	viewer->setBackgroundColor(0, 0, 0);
	/*if (isColored) 
	{
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb,"PCL Cloud");
	}
	else 
	{
		PointCloud<pcl::PointXYZ> out;
		pcl::copyPointCloud(*cloud,out)
		viewer->addPointCloud<PointXYZ>(out, "PCL Cloud");
	}*/
	viewer->addPointCloud<PointT>(cloud, "PCL Cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "PCL Cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
};


template<typename PointT> void 
Viewer<PointT>::keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
{
	if (event.getKeyCode() && event.keyDown()) {
		std::cout << "Key : " << event.getKeyCode() << std::endl;
	}
}

template<typename PointT> void 
Viewer<PointT>::mouse_callback(const pcl::visualization::MouseEvent& event, void*)
{
	if (event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && event.getButton() == pcl::visualization::MouseEvent::LeftButton) {
		std::cout << "Mouse : " << event.getX() << ", " << event.getY() << std::endl;
	}
}

template<typename PointT>
Viewer<PointT>::~Viewer()
{
}

template class Viewer<pcl::PointXYZ>;
template class Viewer<pcl::PointXYZRGB>;
