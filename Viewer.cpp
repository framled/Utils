#include "Viewer.h"


template<typename PointT>
Viewer<PointT>::Viewer() : viewer(new pcl::visualization::PCLVisualizer("PCL Cloud")),
	lastKeyPressed (),
	firstPoint(),
	secondPoint()
{
	this->isFirstPoint = true;
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
		if(this->lastKeyPressed.compare("Escape") == 0){
			break;
		}
	}
}
template<typename PointT> void 
Viewer<PointT>::addCloud(const ConstPtr cloud)
{
	//*ptr_cloud = *cloud;
	if(!viewer->updatePointCloud(cloud, "PCL Cloud")){
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<PointT>(cloud, "PCL Cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "PCL Cloud");
		viewer->addCoordinateSystem(1.0);
		viewer->initCameraParameters();
	}
};


template<typename PointT> void 
Viewer<PointT>::keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
{
	if (event.getKeyCode() && event.keyDown()) {
		std::cout << "Key : " << event.getKeyCode() << " " << event.getKeySym()  << std::endl;
		this->lastKeyPressed = event.getKeySym();
	}
}
/**
 * Mouse callback
 * Necesito arreglar EIGEN Vector 4f
 */
template<typename PointT> void 
Viewer<PointT>::mouse_callback(const pcl::visualization::MouseEvent& event, void*)
{
	if (event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && event.getButton() == pcl::visualization::MouseEvent::LeftButton) {
		std::cout << "Mouse : " << event.getX() << ", " << event.getY() << std::endl;
		if(isFirstPoint){
			firstPoint.x = event.getX();
			firstPoint.y = event.getY();

			isFirstPoint = false;
		}else{
			secondPoint.x = event.getX();
			secondPoint.y = event.getY();
			this->rotatePointCloud(firstPoint, secondPoint);
			firstPoint = secondPoint;

		}
	}
	if (event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease && event.getButton() == pcl::visualization::MouseEvent::LeftButton) {
		isFirstPoint = true;
	}

}
template<typename PointT> void
Viewer<PointT>::rotatePointCloud(const pcl::PointXY& first, const pcl::PointXY& second){
	float distance = pcl::euclideanDistance(first, second);
	float theta = pcl::deg2rad(distance);
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	if(this->lastKeyPressed.compare("y")){
		transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
	}else
	if(this->lastKeyPressed.compare("z")){
			transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
	}else
	if(this->lastKeyPressed.compare("x")){
		transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));
	}
	//pcl::transformPointCloud(*ptr_cloud, *ptr_cloud, transform);
	//this->addCloud(ptr_cloud);
}
template<typename PointT>
Viewer<PointT>::~Viewer()
{
}

template class Viewer<pcl::PointXYZ>;
template class Viewer<pcl::PointXYZRGB>;
