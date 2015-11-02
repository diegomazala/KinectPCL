#include <SDKDDKVer.h>

// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "KinectGrabber.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc

#include <iostream>



class KinectViewer
{
public:
	KinectViewer() : viewer("PCL OpenNI Viewer"), cloud_prev(new pcl::PointCloud<pcl::PointXYZRGB>)
	{}

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		if (!viewer.wasStopped())
		{
			pcl::console::TicToc time;
			const int max_iterations = 2;

			
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);  // point cloud for voxel grid
			const float leaf_size = 0.1f;
			time.tic();
			pcl::VoxelGrid<pcl::PointXYZRGB> sor;
			sor.setInputCloud(cloud);
			sor.setLeafSize(leaf_size, leaf_size, leaf_size);
			sor.filter(*cloud_filtered);
			std::cout << "\nFiltered cloud : leaf_size( " << leaf_size << ") Cloud size: (" << cloud_filtered->size() << " points) in " << time.toc() << " ms\n" << std::endl;



			// The Iterative Closest Point algorithm
			time.tic();
			pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
			icp.setMaximumIterations(max_iterations);
			icp.setInputSource(cloud_prev);
			icp.setInputTarget(cloud_filtered);
			//icp.setInputTarget(cloud);

			// Set the euclidean distance difference epsilon (criterion 3)
			//icp.setEuclideanFitnessEpsilon(1e-5);
			// Set the transformation epsilon (criterion 2)
			//icp.setTransformationEpsilon(1e-2);
			// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
			//icp.setMaxCorrespondenceDistance(0.5);


			icp.align(*cloud_prev);
			std::cout << "Applied " << max_iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

			if (icp.hasConverged())
			{
				std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
				std::cout << "\nICP transformation " << max_iterations << " : cloud_icp -> cloud_in" << std::endl;
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
			}


			
			pcl::copyPointCloud(*cloud_filtered, *cloud_prev);

			//viewer.showCloud(cloud_filtered);
			viewer.showCloud(cloud);
		}
	}

	void run()
	{
		pcl::Grabber* grabber = new pcl::KinectGrabber();

		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
			boost::bind(&KinectViewer::cloud_cb_, this, _1);

		grabber->registerCallback(f);

		grabber->start();

		while (!viewer.wasStopped())
		{
			boost::this_thread::sleep(boost::posix_time::seconds(1));
		}

		grabber->stop();
	}

	pcl::visualization::CloudViewer viewer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_prev;  // point cloud for ICP
};



int main()
{
	KinectViewer v;
	v.run();
	return 0;
}

