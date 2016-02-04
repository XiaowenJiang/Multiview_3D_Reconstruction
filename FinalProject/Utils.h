#include<vector>
#include<opencv2\core\core.hpp>
#include<opencv\cv.h>
#include <opencv/highgui.h>
#include <opencv2/nonfree/features2d.hpp>
#ifndef __UTILS_H__
#define __UTILS_H__

cv::Mat GetFromYML(std::string filename, std::string blockname);

cv::Mat ReadImg(int i, std::string directory);

cv::Mat ReadImgRGB(int i, std::string directory);

void WriteImg(int i, std::string directory, std::string nameprefix, cv::Mat img);

void WriteYML(std::string filename, std::string blockname, cv::Mat matrix);

std::vector<cv::Point2f> KeypointsToPoints(std::vector<cv::KeyPoint> keypoints);

bool CheckCoherentRotation(cv::Mat_<double>& R);

void DrawCrossHair(cv::Mat img, cv::Point2f point);
#endif