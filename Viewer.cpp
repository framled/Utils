#include "Viewer.h"


template<typename PointT>
Viewer<PointT>::Viewer() :
	viewer(new pcl::visualization::PCLVisualizer("PCL Cloud")),
	lastKeyPressed (),
	firstPoint(),
	secondPoint()
{
	this->isFirstPoint = true;
	this->isRotateActive = false;
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
Viewer<PointT>::addCloud(ConstPtr cloud)
{
	if(!viewer->updatePointCloud(cloud, "PCL Cloud")){
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<PointT>(cloud, "PCL Cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "PCL Cloud");
		viewer->addCoordinateSystem(1.0);
		viewer->initCameraParameters();
	}
}

template<typename PointT> void
Viewer<PointT>::updateCloud(CloudPtr& cloud){
	this->ptr_cloud = cloud;
	this->addCloud(this->ptr_cloud);
}

template<typename PointT>void
Viewer<PointT>::save(CloudPtr& cloud){
	std::string path("");
	std::string format("%d_%m_%Y [%H:%M:%S]");
	Utilities::FormatDateTime(format,boost::posix_time::second_clock::local_time(), path);
	path = path + ".pcd";
	if(!pcl::io::savePCDFileASCII(path, *cloud)){
		std::cout << "Saved " << path << std::endl;
	}
}

template<typename PointT> void 
Viewer<PointT>::keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
{
	if (event.getKeyCode()
			&& event.keyDown()
			&& (event.getKeySym() == "x" || event.getKeySym() == "y" || event.getKeySym() == "z")) {
		std::cout << "Rotate aroun axis : " << event.getKeyCode()  << std::endl;
		this->lastKeyPressed = event.getKeySym();
	}
	this->isRotateActive = event.getKeyCode() && event.keyDown() && event.getKeySym() == "k";
	if (event.getKeyCode() && event.keyUp()) {
		this->isRotateActive = false;
	}
	if(event.getKeyCode() && event.isCtrlPressed() && event.getKeySym() == "g"){
		this->save(this->ptr_cloud);
	}
}
/**
 * Mouse callback
 */
template<typename PointT> void 
Viewer<PointT>::mouse_callback(const pcl::visualization::MouseEvent& event, void*)
{
	if(event.getType() == pcl::visualization::MouseEvent::MouseButtonPress){
		if(event.getButton() == pcl::visualization::MouseEvent::LeftButton) {
			this->leftMouse_pressed(event);
		}
	}
	if(event.getType() == pcl::visualization::MouseEvent::MouseMove){
		this->mouse_dragged(event);
	}
}
template<typename PointT> void
Viewer<PointT>::leftMouse_pressed(const pcl::visualization::MouseEvent& event){
	currentPosition.x = event.getX();
	currentPosition.y = event.getY();
}
template<typename PointT> void
Viewer<PointT>::mouse_dragged(const pcl::visualization::MouseEvent& event){
	pcl::PointXY secondPoint;
	secondPoint.x = event.getX();
	secondPoint.y = event.getY();

	this->rotatePointCloud(this->currentPosition, secondPoint);
	this->currentPosition = secondPoint;
}
template<typename PointT> void
Viewer<PointT>::rotatePointCloud(const pcl::PointXY& first, const pcl::PointXY& second){
	if(!this->isRotateActive) return;
	std::cout << "rotando " << std::endl;
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
	pcl::transformPointCloud(*ptr_cloud, *ptr_cloud, transform);
	this->updateCloud(ptr_cloud);
}
template<typename PointT>
Viewer<PointT>::~Viewer()
{
}

template class Viewer<pcl::PointXYZ>;
template class Viewer<pcl::PointXYZRGB>;
