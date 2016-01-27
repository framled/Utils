#include "Utilities.h"

Utilities::Utilities()
{
}

Utilities::~Utilities()
{
}


int Utilities::UniqueNumber()
{
	static int current = 0;
	return ++current;
}

std::vector<int> Utilities::getIndices(const unsigned int length)
{
	std::vector<int> indices(length);
	std::generate_n(indices.begin(), length, UniqueNumber);
	return indices;
}

void Utilities::convert2XYZ(std::vector<pcl::PCLPointCloud2>& input, pcl::PointCloud<pcl::PointXYZ>::Ptr& output)
{
	for (std::vector<pcl::PCLPointCloud2>::iterator cloud = input.begin(); cloud != input.end(); ++cloud) {
		pcl::PointCloud<pcl::PointXYZ> c;
		pcl::fromPCLPointCloud2(*cloud, c);
		*output += c;
	}
}
void Utilities::convert2XYZRGB(std::vector<pcl::PCLPointCloud2>& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output)
{
	for (std::vector<pcl::PCLPointCloud2>::iterator cloud = input.begin(); cloud != input.end(); ++cloud) {
		pcl::PointCloud<pcl::PointXYZRGB> c;
		pcl::fromPCLPointCloud2(*cloud, c);
		*output += c;
	}
}

void Utilities::show(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	Viewer<pcl::PointXYZ> viewer;
	viewer.addCloud(cloud);
	viewer.run();
}

void Utilities::showColor(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
	Viewer<pcl::PointXYZRGB> viewer;
	viewer.addCloud(cloud);
	viewer.run();
}

void Utilities::getFiles(std::string path, std::vector<std::string>& output)
{
	boost::filesystem::path p(path);
	boost::filesystem::directory_iterator i;
	for (i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); ++i)
	{
		if (!is_directory(i->path()))
		{
			size_t found_pcd = i->path().filename().string().find(".pcd");
			size_t found_ply = i->path().filename().string().find(".ply");
			if (found_pcd != std::string::npos || found_ply != std::string::npos)
			{
				std::cout << "Found file " << i->path().string() << std::endl;
				output.push_back(i->path().string());
			}
		}
	}
}

void Utilities::getFiles(char** argv, std::vector<int>& indices, std::vector<std::string>& output)
{
	std::cout << indices.size() << std::endl;
	for (std::vector<int>::iterator i = indices.begin(); i != indices.end(); ++i)
	{
		std::cout << "Found file " << argv[*i] << std::endl;
		output.push_back(argv[*i]);
	}
}

void Utilities::read(std::vector<std::string> paths, std::vector<pcl::PCLPointCloud2>& clouds_blob)
{
	boost::thread_group tgroup;
	int i = 0;
	for (std::vector<std::string>::iterator path = paths.begin(); path != paths.end(); ++path) {
		size_t found_pcd = path->find(".pcd");
		size_t found_ply = path->find(".ply");

		if (found_pcd != std::string::npos)
		{
			tgroup.create_thread(boost::bind(Utilities::readPCDFile, *path, boost::ref(clouds_blob[i])));
		}
		else if (found_ply != std::string::npos)
		{
			tgroup.create_thread(boost::bind(Utilities::readPLYFile, *path, boost::ref(clouds_blob[i])));
		}
		i++;
	}
	tgroup.join_all();
}

void Utilities::readPCDFile(std::string path, pcl::PCLPointCloud2& cloud)
{
	std::cout << "==============================================================" << std::endl
		<< "prepare for read " << path << std::endl;
	if (pcl::io::loadPCDFile(path, cloud) == -1)
	{
		std::string error("Couldn't read file " + path + " \n");
		PCL_ERROR(error.c_str());
	}
	else
	{
		std::cout << "success read file " << path << std::endl
			<< "==============================================================" << std::endl
			<< "Details of the point cloud " << std::endl
			<< "Width:	" << cloud.width << std::endl
			<< "Height: " << cloud.height << std::endl
			<< "==============================================================" << std::endl;
	}
}

void Utilities::readPLYFile(std::string path, pcl::PCLPointCloud2& cloud)
{

	if (pcl::io::loadPLYFile(path, cloud) == -1)
	{
		std::string error("Couldn't read file " + path + " \n");
		PCL_ERROR(error.c_str());
	}
	else
	{
		std::cout << "success read file " << path << std::endl
			<< "==============================================================" << std::endl
			<< "Details of the point cloud " << std::endl
			<< "Width:	" << cloud.width << std::endl
			<< "Height: " << cloud.height << std::endl
			<< "==============================================================" << std::endl;
	}
}


void Utilities::writePCDFile(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, std::string path, int how_many_files = 1)
{
	path = mkdir(path);
	std::cout << "writing file " << path << std::endl;
	for (int i = 0; i < how_many_files; ++i) {
		std::string extension(getExtension(path));
		std::string str(path);
		str.replace(str.find(extension), sizeof(extension) - 1, "_" + boost::lexical_cast<std::string>(i) + extension);
		std::vector<int> indices(getIndices((cloud->width * cloud->height) / how_many_files));
		std::cout << cloud->width * cloud->height << std::endl;
		pcl::io::savePCDFile(str, *cloud, indices);
	}
}
void Utilities::writePCDFile(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, std::string path, int how_many_files = 1)
{
	path = mkdir(path);
	std::cout << "writing file " << path << std::endl;
	for (int i = 0; i < how_many_files; ++i) {
		std::string extension(getExtension(path));
		std::string str(path);
		str.replace(str.find(extension), sizeof(extension) - 1, "_" + boost::lexical_cast<std::string>(i) + extension);
		std::vector<int> indices(getIndices((cloud->width * cloud->height) / how_many_files));
		std::cout << cloud->width * cloud->height << std::endl;
		pcl::io::savePCDFile (str, *cloud, indices);
	}
}
std::string Utilities::mkdir(const std::string& dir)
{
	boost::filesystem::path p (dir);
	if( !( boost::filesystem::exists(p.parent_path())))
	{
		std::string input;
		std::cout << "path: " << p << " doesn't exist, I going to create one" << std::endl;
		if(boost::filesystem::create_directory(p.parent_path())){
			std::cout << "Yiey, i have success for create route =D" << std::endl;
		}
	}
	return p.string();
}

std::string Utilities::getExtension(std::string file)
{
	std::string extension("");
	for (int i = file.length() - 1; i >= 0; i--) {
		if (file[i] != '.') 
		{
			extension += file[i];
		}
		else 
		{
			break;
		}
	}
	std::reverse(extension.begin(), extension.end());
	return "." + extension;
}

void Utilities::print(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	for (size_t i = 0; i < cloud->points.size (); ++i)
	    std::cout << i << "	" << cloud->points[i].x
	              << " 	"    << cloud->points[i].y
	              << "	"    << cloud->points[i].z << std::endl;
}

pcl::PointXYZ& Utilities::mean_point (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    unsigned int i;
    unsigned int size = cloud->points.size();
    double mean_x=0.0, mean_y=0.0, mean_z=0.0;
    pcl::PointXYZ point;

    std::cout << "Calculating mean point." << std::endl;

    // find mean
    for (i=0; i<size; i++)
    {
        mean_x += cloud->points[i].x;
        mean_y += cloud->points[i].y;
        mean_z += cloud->points[i].z;
    }
    mean_x /= (double)size;
    mean_y /= (double)size;
    mean_z /= (double)size;

    point.x = mean_x;
    point.y = mean_y;
    point.z = mean_z;

    return point;
}

//==================================================================
void Utilities::normalize_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    unsigned int i;
    unsigned int size = cloud->points.size();
    pcl::PointXYZ mean_p;
    double std_x=0.0, std_y=0.0, std_z=0.0;
    double std_max = 0.0;

    std::cout << "Point cloud normalization:" << std::endl;

    // find the mean point
    mean_p = mean_point(cloud);

    std::cout << "Finding standard-deviation" << std::endl;
    // find sigma^2
    for (i=0; i<size; i++)
    {
        std_x += cloud->points[i].x - mean_p.x;
        std_y += cloud->points[i].y - mean_p.y;
        std_z += cloud->points[i].z - mean_p.z;
    }
    std_x /= (double)size;
    std_y /= (double)size;
    std_z /= (double)size;

    // which one is the biggest? we want to normalize with respect
    // to the biggest component
    std_max = (std_x>std_y)?std_x:std_y;
    std_max = (std_z>std_max)?std_z:std_max;

    std::cout << "Scaling point cloud." << std::endl;

    // normalize
    for (i=0; i<size; i++)
    {
        cloud->points[i].x /= std_max;
        cloud->points[i].y /= std_max;
        cloud->points[i].z /= std_max;
    }
}
