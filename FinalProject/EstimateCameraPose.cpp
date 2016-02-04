#include"EstimateCameraPose.h"
EstimateCameraPose::EstimateCameraPose(){}

EstimateCameraPose::~EstimateCameraPose(){}

EstimateCameraPose::EstimateCameraPose(std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2, cv::Mat K)
{
	matchedpoints1 = points1;
	matchedpoints2 = points2;
	intrinsic = K;
	fundamental = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 3, 0.99, Mask);
	E = K.t() * fundamental * K;
}

cv::Mat EstimateCameraPose::getFundamental()
{
	return fundamental;
}

cv::Mat_<double> EstimateCameraPose::getEssential()
{
	return E;
}

void EstimateCameraPose::calcCameraPoseFromE()
{
	cv::SVD svd(E, CV_SVD_MODIFY_A);
	cv::Mat svd_u = svd.u;
	cv::Mat svd_vt = svd.vt;
	cv::Mat svd_w = svd.w;
	cv::Matx33d W(0, -1, 0, 1, 0, 0, 0, 0, 1);
	cv::Matx33d W_t;
	cv::transpose(W, W_t);
	cv::Mat_<double> R = svd_u * cv::Mat(W) * svd_vt; //HZ 9.19
	cv::Mat_<double> R2 = svd_u * cv::Mat(W_t) * svd_vt;
	cv::Mat_<double> t = svd_u.col(2); //u3
	cv::Mat_<double> t2 = -t;
	if (!CheckCoherentRotation(R)||!CheckCoherentRotation(R2)) {
		std::cout << "resulting rotation is not coherent\n";
		return;
	}
	cv::Matx34d origin = cv::Matx34d(1, 0, 0, 0 ,0, 1, 0, 0, 0, 0, 1, 0);
	M1 = intrinsic*origin;
	//four possible camera matrices
	M2.resize(4);
	M2[0] = cv::Matx34d(R(0, 0), R(0, 1), R(0, 2), t(0),
		R(1, 0), R(1, 1), R(1, 2), t(1),
		R(2, 0), R(2, 1), R(2, 2), t(2));
	M2[1] = cv::Matx34d(R(0, 0), R(0, 1), R(0, 2), t2(0),
		R(1, 0), R(1, 1), R(1, 2), t2(1),
		R(2, 0), R(2, 1), R(2, 2), t2(2));
	M2[2] = cv::Matx34d(R2(0, 0), R2(0, 1), R2(0, 2), t2(0),
		R2(1, 0), R2(1, 1), R2(1, 2), t2(1),
		R2(2, 0), R2(2, 1), R2(2, 2), t2(2));
	M2[3] = cv::Matx34d(R2(0, 0), R2(0, 1), R2(0, 2), t(0),
		R2(1, 0), R2(1, 1), R2(1, 2), t(1),
		R2(2, 0), R2(2, 1), R2(2, 2), t(2));
	for (int i = 0; i < 4; i++)
	{
		M2[i] = intrinsic*M2[i];
	}
	//homogeneous 3D points
	cv::Mat points4d;
	//record the number of zeros that are negative
	std::vector<int> falsezero={0,0,0,0};
	points3d.resize(4);
	for (int i = 0; i < M2.size(); i++)
	{
		triangulatePoints(M1, M2[i], matchedpoints1,
			matchedpoints2, points4d);
		for (int j = 0; j < points4d.cols; j++)
		{
			//convert homogeneous 3D points to 3D points
			float w = points4d.at<float>(3, j);
			float x = points4d.at<float>(0, j) / w;
			float y = points4d.at<float>(1, j) / w;
			float z = points4d.at<float>(2, j) / w;
			cv::Point3d p(x, y, z);
			if (z < 0)
			{
				falsezero[i]++;
			}
			points3d[i].push_back(p);
		}
	}
	int i = 0;
	int small = falsezero[0];
	for (int j = 1; j < 4; j++)
	{
		if (falsezero[j] < small)
		{
			small = falsezero[j];
			i = j;
		}
	}
	truepoints3d = points3d[i];
		switch (i)
		{
		case 0:
		{
			rotation = R;
			translation = t;
			break;
		}
		case 1:
		{
			rotation = R;
			translation = t2;
			break;
		}
		case 2:
		{
			rotation = R2;
			translation = t2;
			break;
		}
		case 3:
		{
			rotation = R2;
			translation = t;
			break;
		}
		default:
			break;
		}
}

cv::Mat EstimateCameraPose::getM1()
{
	return (cv::Mat)M1;
}

cv::Mat EstimateCameraPose::getM2()
{
	return (cv::Mat)M2;
}

cv::Mat EstimateCameraPose::getRotation()
{
	return rotation;
}

cv::Mat EstimateCameraPose::getTranslation()
{
	return translation;
}

void EstimateCameraPose::setRotation(cv::Mat R)
{
	rotation = R;
}

void EstimateCameraPose::setTranslation(cv::Mat t)
{
	translation = t;
}

std::vector < cv::Point3f> EstimateCameraPose::get3Dpoints()
{
	return truepoints3d;
}