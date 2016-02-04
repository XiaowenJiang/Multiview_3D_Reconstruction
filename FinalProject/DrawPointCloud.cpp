#include"DrawPointCloud.h"

DrawPointCloud::DrawPointCloud()
{
}

DrawPointCloud::~DrawPointCloud()
{
}

DrawPointCloud::DrawPointCloud(std::vector<cv::Point3f> points3d, cv::Mat inputRGB,
	std::vector<cv::Mat> all_r, std::vector<cv::Mat> all_t)
{
	std::vector<cv::Point2f> allpoints;
	//for (int row = 0; row < inputRGB.rows; row++)
	//{
	//	for (int col = 0; col < inputRGB.cols; col += inputRGB.channels())
	//	{

	//	}
	//}
	cloud.width = points3d.size();
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width*cloud.height);
	for (size_t i = 0; i < cloud.points.size(); i++)
	{
		float temp = 255.0;
		cloud.points[i].x = points3d[i].x ;
		cloud.points[i].y = points3d[i].y ;
		cloud.points[i].z = points3d[i].z ;
	}
	//pcl::visualization::CloudViewer viewer("point cloud viewer");
	pcl::visualization::PCLVisualizer viewer("point cloud viewer");;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZ>);
	*cloudptr = cloud;
	cv::Mat R0;
	cv::Rodrigues(all_r[1],R0);
	viewer.addPointCloud(cloudptr);
	viewer.addCube(5, 5, 5, 0, 0, 0, 1.0, 0, 0, "cube", 0);
	viewer.addCoordinateSystem(1, "axis", 0);
	pcl::io::savePCDFileASCII("pointcloud.pcd", cloud);
	cv::waitKey();
}