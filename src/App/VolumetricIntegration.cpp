#include <iostream>
#include <string>
#include <fstream>
#include <Eigen/Dense>
#include <vector>


#if 0
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

#include <pcl/filters/voxel_grid.h>
#endif


#define DegToRad(angle_degrees) (angle_degrees * M_PI / 180.0)		// Converts degrees to radians.
#define RadToDeg(angle_radians) (angle_radians * 180.0 / M_PI)		// Converts radians to degrees.

const double fov_y = 70.0;
Eigen::Matrix3d									K;
std::pair<Eigen::MatrixXd, Eigen::MatrixXd>		Rt(Eigen::MatrixXd(3, 4), Eigen::MatrixXd(3, 4));
std::pair<Eigen::Matrix3d, Eigen::Matrix3d>		R(Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero());
std::pair<Eigen::Vector3d, Eigen::Vector3d>		t;
std::pair<Eigen::Matrix4d, Eigen::Matrix4d>		T(Eigen::Matrix4d::Zero(), Eigen::Matrix4d::Zero());

template<typename Type>
struct Voxel
{
	Eigen::Matrix<Type, 3, 1> point;
	Type tsdf;
	Type weight;
};
typedef Voxel<double> Voxeld;
typedef Voxel<float> Voxelf;

void setupMatrices()
{
	//
	// K matrix
	//
	//double width = 1936 * 2; // 1920.0;
	//double height = 1296 * 2; // 1080.0;
#if 0
	double width = 1920.0;
	double height = 1080.0;
	double aspect = 1.0; // width / height;
	//double f = 114.873 / 0.0130887; // 1.0 / std::tan(DegToRad(fov_y) / 2.0);
	double f = 1.0 / std::tan(DegToRad(fov_y) / 2.0);
	K.setZero();
	K(0, 0) = f / aspect;
	K(1, 1) = f;
	K(0, 2) = width / 2.0;
	K(1, 2) = height / 2.0;
	K(2, 2) = 1.0;
#else
	double width = 1920.0;
	double height = 1080.0;
	double aspect = 1.0; // width / height;
	double f = 1.0 / std::tan(DegToRad(fov_y) / 2.0);
	K.setZero();
	K(0, 0) = f / aspect;
	K(1, 1) = f;
	K(0, 2) = 0.0;
	K(1, 2) = 0.0;
	K(2, 2) = 0.0;
#endif

	//
	// R matrix
	//
	Eigen::Affine3d rotation = Eigen::Affine3d::Identity();
	rotation.rotate(Eigen::AngleAxisd(DegToRad(90.0), Eigen::Vector3d::UnitY()));		// 90º
	R.first.setIdentity();
	R.second = rotation.matrix().block(0, 0, 3, 3);

	//
	// t vector
	//
	t.first << 0.0, 0.0, 0.0;
	t.second << 0.0, 0.0, 0.0;

	//
	// Rt matrix
	//
	Rt.first.block(0, 0, 3, 3) = R.first;
	Rt.second.block(0, 0, 3, 3) = R.second;
	Rt.first.col(3) = R.first * t.first;
	Rt.second.col(3) = R.second * t.second;

	T.first.block(0, 0, 3, 4) = Rt.first;
	T.second.block(0, 0, 3, 4) = Rt.second;
	T.first.row(3) << 0.0, 0.0, 0.0, 1.0;
	T.second.row(3) << 0.0, 0.0, 0.0, 1.0;

	//std::cout << std::fixed << std::endl
	//	<< "Rt: " << std::endl << Rt.second << std::endl
	//	<< "T: " << std::endl << T.second << std::endl;
}


static bool importObj(const std::string& filename, std::vector<Eigen::Vector3d>& points3D, int max_point_count = INT_MAX)
{
	std::ifstream inFile;
	inFile.open(filename);

	if (!inFile.is_open())
	{
		std::cerr << "Error: Could not open obj input file: " << filename << std::endl;
		return false;
	}

	points3D.clear();

	int i = 0;
	while (inFile)
	{
		std::string str;

		if (!std::getline(inFile, str))
		{
			if (inFile.eof())
				return true;

			std::cerr << "Error: Problems when reading obj file: " << filename << std::endl;
			return false;
		}

		if (str[0] == 'v')
		{
			std::stringstream ss(str);
			std::vector <std::string> record;

			char c;
			double x, y, z;
			ss >> c >> x >> y >> z;

			Eigen::Vector3d p(x, y, z);
			points3D.push_back(p);
		}

		if (i++ > max_point_count)
			break;
	}

	inFile.close();
	return true;
}
static void exportObj(const std::string& filename, const std::vector<Eigen::Vector3d>& points3D)
{
	std::ofstream file;
	file.open(filename);
	for (const auto X : points3D)
	{
		file << std::fixed << "v " << X.transpose() << std::endl;
	}
	file.close();
}

static bool import_xyzd(const std::string& filename, std::vector<Eigen::Vector4d>& points_xyzd, int max_point_count = INT_MAX)
{
	std::ifstream inFile;
	inFile.open(filename);

	if (!inFile.is_open())
	{
		std::cerr << "Error: Could not open obj input file: " << filename << std::endl;
		return false;
	}

	points_xyzd.clear();

	int i = 0;
	while (inFile)
	{
		std::string str;

		if (!std::getline(inFile, str))
		{
			if (inFile.eof())
				return true;

			std::cerr << "Error: Problems when reading obj file: " << filename << std::endl;
			return false;
		}

		if (str[0] == 'v')
		{
			std::stringstream ss(str);
			std::vector <std::string> record;

			char c;
			double x, y, z, d;
			ss >> c >> x >> y >> z >> d;

			Eigen::Vector4d p(x, y, z, d);
			points_xyzd.push_back(p);
		}

		if (i++ > max_point_count)
			break;
	}

	inFile.close();
	return true;
}

static void export_xyzd(const std::string& filename, const std::vector<Eigen::Vector4d>& points_xyzd)
{
	std::ofstream file;
	file.open(filename);
	for (const auto p : points_xyzd)
	{
		file << std::fixed << "v " << p.transpose() << std::endl;
	}
	file.close();
}

int main(int argc, char* argv[])
{
	const std::string filepath = argv[1];
	int volume_size = atoi(argv[2]);
	int voxel_size = atoi(argv[3]);

	setupMatrices();
		
#if 0
	std::vector<Eigen::Vector3d> points3D;
	importObj(filepath, points3D);

	assert(	P.first.rows() == 3 && P.first.cols() == 4 && 
			P.second.rows() == 3 && P.second.cols() == 4);


	std::vector<Eigen::Vector3d> points3DProj_0, points3DProj_1;
	for (auto p3d : points3D)
	{
		Eigen::Vector3d x0 = K * Rt.first * p3d.homogeneous();
		Eigen::Vector3d x1 = K * Rt.second * p3d.homogeneous();

		points3DProj_0.push_back(x0);
		points3DProj_1.push_back(x1);
	}


	exportObj("../../data/output_proj_00.obj", points3DProj_0);
	exportObj("../../data/output_proj_90.obj", points3DProj_1);
#endif

	
#if 0
	std::vector<Eigen::Vector3d> points3D;
	importObj(filepath, points3D);


	std::vector<Eigen::Vector4d> points3DProj_0, points3DProj_1;
	for (auto p3d : points3D)
	{
		Eigen::Vector3d p3d_0 = Rt.first * p3d.homogeneous();
		Eigen::Vector3d p3d_1 = Rt.second * p3d.homogeneous();

		double d0 = p3d_0.z();
		double d1 = p3d_1.z();

		Eigen::Vector3d x0 = K * Rt.first * p3d.homogeneous();
		Eigen::Vector3d x1 = K * Rt.second * p3d.homogeneous();

		Eigen::Vector4d p0(x0.x(), x0.y(), x0.z(), 1);
		Eigen::Vector4d p1(x1.x(), x1.y(), x1.z(), 1);

		points3DProj_0.push_back(p0);
		points3DProj_1.push_back(p1);

	}

	//export_xyzd("../../data/output_proj_00.xyzd.obj", points3DProj_0);
	//export_xyzd("../../data/output_proj_90.xyzd.obj", points3DProj_1);
#endif

#if 0
	pcl::PCLPointCloud2::Ptr cloud_pcd(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PCDReader pcd;
	if (pcd.read(filepath, *cloud_pcd) < 0)
	{
		pcl::console::print_error("[PcdGrabberFromFile::loadPcdFileXYZ] Failed to load file %s", filepath.c_str());
	}
	pcl::fromPCLPointCloud2(*cloud_pcd, *cloud);

	std::cout << "Cloud point count: " << cloud->points.size() << std::endl;

	int x = 50;
	int y = 50;
	int depth_index = y * 424 + x;
	std::cout << x << ", " << y << " : " << cloud->points.at(depth_index) << std::endl;
	x = 400;
	y = 50;
	depth_index = y * 424 + x;
	std::cout << x << ", " << y << " : " << cloud->points.at(depth_index) << std::endl;
#endif

	std::vector<Eigen::Vector4d> points_xyzd;
	import_xyzd(filepath, points_xyzd);

	// Creating volume
	std::size_t sx, sy, sz;
	sx = sy = sz = volume_size / voxel_size;
	std::size_t slice_size = (sx + 1) * (sy + 1);

	std::vector<Voxeld> tsdf_volume((sx + 1) * (sy + 1) * (sz + 1));
	Eigen::Matrix4d volume_transformation = Eigen::Matrix4d::Identity();			
	volume_transformation.col(3) << -(sx / 2.0), -(sy / 2.0), -(sz / 2.0), 1.0;	// set translate

	std::cout << std::fixed << std::endl
		<< "Volume Transformation: " << std::endl << volume_transformation << std::endl;


	int i = 0;
	for (int z = 0; z <= sz; ++z)
	{
		for (int y = 0; y <= sy; ++y)
		{
			for (int x = 0; x <= sx; ++x, i++)
			{
				tsdf_volume[i].point = Eigen::Vector3d(x, y, z);
				tsdf_volume[i].weight = i;

				//std::cout << tsdf_volume[i].weight << '\t' << tsdf_volume[i].point.transpose() << std::endl;
			}
		}
	}

	for (auto it_volume = tsdf_volume.begin(); it_volume != tsdf_volume.end(); it_volume += slice_size)
	{
		auto z_slice_begin = it_volume;
		auto z_slice_end = it_volume + slice_size - 1;

		//std::cout << z_slice_begin->weight << ", " << z_slice_end->weight << " : " << z_slice_begin->point.transpose() << ", " << z_slice_end->point.transpose() << std::endl;

		//std::cout
		//	<< std::fixed << std::endl
		//	<< z_slice_begin->point.transpose() << "\t" << z_slice_end->point.transpose() << std::endl
		//	<< (volume_transformation * z_slice_begin->point.homogeneous()).transpose().head<3>() << "\t"
		//	<< (volume_transformation * z_slice_end->point.homogeneous()).transpose().head<3>()
		//	<< std::endl << std::endl;

		for (auto it = z_slice_begin; it != z_slice_end; ++it)
		{
			Eigen::Vector4d vg = volume_transformation * it->point.homogeneous();
			Eigen::Vector4d v = T.second.inverse() * vg;

			Eigen::Vector3d p = K * v.head<3>();

			//std::cout << std::fixed
			//	<< std::endl
			//	<< "pt : " << it->point.homogeneous().transpose() << std::endl
			//	<< "vg : " << vg.transpose() << std::endl
			//	<< "v  : " << v.transpose() << std::endl
			//	<< "p  : " << p.transpose() << std::endl;

			
			// if v in camera frustum
			{
				Eigen::Vector4d ti = T.second.inverse().col(3);
				//double sdf = (ti - vg).norm() - D(p)
			}


		}


	}


	return EXIT_SUCCESS;
}


