#include<opencv2\core\core.hpp>
#include<opencv2\highgui\highgui.hpp>
#include<opencv2\imgproc\imgproc.hpp>
#include<features2d\features2d.hpp>
//3D point cloud
#include<pcl\io\pcd_io.h>
#include<pcl\point_types.h>
#include<pcl\visualization\cloud_viewer.h>
#include<vector>
#include"Utils.h"
#ifndef __DRAWPOINTCLOUD_H__
#define __DRAWPOINTCLOUD_H__
class DrawPointCloud{
private:
	pcl::PointCloud<pcl::PointXYZ> cloud;
public:
	DrawPointCloud();
	~DrawPointCloud();
	DrawPointCloud(std::vector<cv::Point3f> points3d,cv::Mat inputRGB,
		std::vector<cv::Mat>all_r,std::vector<cv::Mat>all_t);
};

#endif