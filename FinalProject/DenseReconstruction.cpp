#include"DenseReconstruction.h"

DenseReconstuction::DenseReconstuction()
{}

DenseReconstuction::~DenseReconstuction()
{}

DenseReconstuction::DenseReconstuction(cv::Mat fundamental, cv::Mat inputimg1,cv::Mat inputimg2,
	std::vector<cv::Point2f>points1,std::vector<cv::Point2f>points2)
{
	cv::Mat H1, H2;
	cv::Size imgsize = inputimg1.size();
	cv::stereoRectifyUncalibrated(points1, points2, fundamental, imgsize, H1, H2);
	cv::warpPerspective(inputimg1, warped1, H1, imgsize);
	cv::warpPerspective(inputimg2, warped2, H2, imgsize);
	cv::Mat twoimages(imgsize.height, imgsize.width * 2,CV_8UC1);
	cv::Mat left(twoimages, cv::Rect(0, 0, imgsize.width, imgsize.height));
	warped1.copyTo(left);
	cv::Mat right(twoimages, cv::Rect(imgsize.width, 0, imgsize.width, imgsize.height));
	warped2.copyTo(right);
	for (int i = 1; i < 10; i++)
	{
		cv::Point2f point_left(0,imgsize.height/10*i);
		cv::Point2f point_right(imgsize.width*2, imgsize.height / 10 * i);
		cv::line(twoimages, point_left, point_right, cv::Scalar(255, 0, 0));
	}
	cv::imshow("rectified_images", twoimages);
	/*cv::Size imgsize= inputimg1.size();
	cv::stereoRectify(intrinsic, distor_coeff, intrinsic, distor_coeff, imgsize, R, t, R1, R2, M1, M2, Q);
	cv::Mat new_intrinsic1 = M1.colRange(0, 3);
	cv::Mat new_intrinsic2 = M2.colRange(0, 3);
	map1.resize(2);
	map2.resize(2);
	cv::Size newsize(1500,1500);
	cv::initUndistortRectifyMap(intrinsic, distor_coeff, R1, new_intrinsic1, newsize, CV_32FC1, map1[0], map2[0]);
	cv::initUndistortRectifyMap(intrinsic, distor_coeff, R2, new_intrinsic2, newsize, CV_32FC1, map1[1], map2[1]);
	cv::remap(inputimg1, inputimg1, map1[0], map2[0], CV_INTER_LINEAR);
	cv::remap(inputimg2, inputimg2, map1[1], map2[1], CV_INTER_LINEAR);
	cv::imshow("rectified1", inputimg1);
	cv::imshow("rectified2", inputimg2);*/
}


void DenseReconstuction::ShowDisparityMap()
{
	cv::StereoBM sbm;
	//parameters for StereoBM

	sbm.state->SADWindowSize = 9;
	sbm.state->numberOfDisparities = 48;
	//normalization windowsize
	sbm.state->preFilterSize = 9;
	//bound on normalized pixel values
	sbm.state->preFilterCap = 32;
	//minimum disparity
	sbm.state->minDisparity = -25;
	//defines the minimum texture value for a pixel to be reliable
	sbm.state->textureThreshold = 800;
	//filters disparity readings based on a 
	//comparison to the next-best correlation
	//along the epipolar line
	sbm.state->uniquenessRatio = 0;
	sbm.state->speckleWindowSize = 0;
	sbm.state->speckleRange = 8;
	sbm.state->disp12MaxDiff = 1;
	sbm(warped1, warped2, Disparity);
	normalize(Disparity, Disparity, 0, 255, CV_MINMAX, CV_8U);
	imshow("Disparity", Disparity);
	imwrite("Disparity.jpg", Disparity);
	cv::waitKey();
}

cv::Mat DenseReconstuction::getDisparityMap()
{
	return Disparity;
}