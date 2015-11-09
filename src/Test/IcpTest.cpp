
#include <SDKDDKVer.h>

#include "CppUnitTest.h"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;


#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

namespace pcl
{		
	TEST_CLASS(icp)
	{
	public:
		
		TEST_METHOD(TestCupCloud)
		{
			typedef pcl::PointXYZRGB PointT;
			typedef pcl::PointCloud<PointT> PointCloudT;
			PointCloudT::Ptr cloud_input(new PointCloudT);
			PointCloudT::Ptr cloud_transformed(new PointCloudT);
			const std::string input_file("../../data/cup.pcd");
			const std::string output_folder("../../data/");

			Assert::IsTrue(pcl::io::loadPCDFile(input_file, *cloud_input) > -1, L"\n<TestCupCloud could not load input file>\n", LINE_INFO());


			for (int i = 0; i < 10; ++i)
			{
				Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
				double theta = (i * M_PI) / 180.0;
				transformation_matrix(0, 0) = cos(theta);
				transformation_matrix(0, 1) = -sin(theta);
				transformation_matrix(1, 0) = sin(theta);
				transformation_matrix(1, 1) = cos(theta);

				transformation_matrix(0, 3) = 0.05;	// A translation on X axis (0.02 meters)
				transformation_matrix(2, 3) = 0.05;	// A translation on Z axis (0.02 meters)

				pcl::transformPointCloud(*cloud_input, *cloud_transformed, transformation_matrix);

				std::stringstream filename;
				filename << output_folder << i << ".pcd";

				pcl::PCDWriter w;
				Assert::IsTrue(w.writeBinaryCompressed(filename.str(), *cloud_transformed) > -1, L"\n<TestCupCloud could not save output file>\n", LINE_INFO());
			}
		}

	};
}