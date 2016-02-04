#include"ProjectPoints.h"

ProjectPoints::ProjectPoints()
{}

ProjectPoints::~ProjectPoints()
{}

ProjectPoints::ProjectPoints(cv::Mat image,std::vector<cv::Point2f>originalpoints,std::vector<cv::Point3f> points3d, cv::Mat rotation, cv::Mat translation,
	cv::Mat intrinsic, cv::Mat distor_coeff)
{
	inputimage = image.clone();
	originpoints = originalpoints;
	projectPoints(points3d, rotation, translation, intrinsic, distor_coeff, projectpoints);
}

void ProjectPoints::ShowReprojectionErrors()
{
	for (int i = 0; i < projectpoints.size(); i++)
	{
		visibility.push_back(1);
		circle(inputimage, projectpoints[i], 3, cv::Scalar(255, 0, 0));
		DrawCrossHair(inputimage, originpoints[i]);
		//line(inputimage, projectpoints[i], originpoints[i], cv::Scalar(255, 0, 0));
	}
	double err = cv::norm(cv::Mat(originpoints), cv::Mat(projectpoints), CV_L2);
	err = std::sqrt(err*err / projectpoints.size());
	//std::cout << "reprojection error: " << err << std::endl;
	cv::imshow("Reprojection_2D", inputimage);
	cv::waitKey();
}

void ProjectPoints::ShowDifferenceSBA(std::vector<cv::Point2f> pointsbeforesba)
{
	std::vector<cv::KeyPoint> keypointsbeforesba;
	std::vector<cv::KeyPoint> keypointsaftersba;
	cv::KeyPoint::convert(pointsbeforesba, keypointsbeforesba);
	cv::KeyPoint::convert(projectpoints, keypointsaftersba);
	cv::drawKeypoints(inputimage, keypointsbeforesba, inputimage,cv::Scalar(255,255,0));
	cv::drawKeypoints(inputimage, keypointsaftersba, inputimage, cv::Scalar(0, 255, 0));
	for (int i = 0; i < projectpoints.size(); i++)
	{
		DrawCrossHair(inputimage, originpoints[i]);
		//line(inputimage, projectpoints[i], originpoints[i], cv::Scalar(255, 0, 0));
	}
	double err = cv::norm(cv::Mat(pointsbeforesba), cv::Mat(projectpoints), CV_L2);
	err = std::sqrt(err*err / projectpoints.size());
	//std::cout << "error: " << err << std::endl;
	cv::imshow("difference before-after sba",inputimage);
	cv::waitKey();
}

std::vector<cv::Point2f> ProjectPoints::getProjectpoints()
{
	return projectpoints;
}

std::vector<int> ProjectPoints::getVisibility()
{
	return visibility;
}

cv::Mat ProjectPoints::getProjectionImage()
{
	return inputimage;
}