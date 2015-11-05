// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "KinectGrabber.h"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>


std::string file_root;
uint32_t number_to_grab = 1, count = 0;

void savecloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
	if (count < number_to_grab)
	{
		boost::filesystem::path path = file_root;
		path.replace_extension("");

		std::stringstream filename;
		filename << path << count << ".pcd";
		std::cout << "Saving to " << filename.str();
		pcl::io::savePCDFile(filename.str(), *cloud, true);

		++count;
	}

	std::cout << ". [" << count << "] frames saved" << std::endl;
}




int main(int argc, char* argv[])
{
	boost::program_options::options_description desc("./spectroscan3d_grabpcd root_output_name");

	desc.add_options()
		("help", "produce help message")
		("output,o", boost::program_options::value<std::string>(&file_root)->required(), "output pcd ")
		("number,n", boost::program_options::value<uint32_t>(&number_to_grab)->default_value(1), "number of pcd to grab")
		;

	boost::program_options::positional_options_description p;
	p.add("output", 1);

	boost::program_options::variables_map vm;
	try
	{
		boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
		boost::program_options::notify(vm);
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		std::cout << desc << std::endl;
		return 1;
	}
	if (vm.count("help"))
	{
		std::cout << desc << std::endl;
		return 0;
	}


	// Create Cloud Viewer
	pcl::visualization::CloudViewer viewer("Point Cloud Viewer");

	// Callback Function to be called when Updating Data
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> show_cb =
		[&viewer](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		if (!viewer.wasStopped()){
			viewer.showCloud(cloud);
		}
	};

	// Callback Function to be called when Updating Data
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> save_cloud_cb =
		[&viewer](const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		savecloud(cloud);
	};


	// Create KinectGrabber
	pcl::Grabber* grabber = new pcl::KinectGrabber();

	// Regist Callback Function
	grabber->registerCallback(show_cb);
	grabber->registerCallback(save_cloud_cb);

	// Start Retrieve Data
	grabber->start();

	while (!viewer.wasStopped())
	{
		// Input Key ( Exit ESC key )
		if (GetKeyState(VK_ESCAPE) < 0)
		{
			break;
		}
	}

	// Stop Retrieve Data
	grabber->stop();

	return EXIT_SUCCESS;
}
