#include<nonfree\nonfree.hpp>
#include<nonfree\features2d.hpp>
#include<opencv2\highgui\highgui.hpp>
#include<opencv2\features2d\features2d.hpp>
#include<core\core.hpp>
#include<video\tracking.hpp>
#include<vector>
#include<unordered_map>
#include"Utils.h"
#ifndef ___SURFMATCH_H__
#define ___SURFMATCH_H__
class SURFMATCH
{
private:
	//input images
	cv::Mat input1, input2;
	//keypoints from two views
	std::vector<cv::KeyPoint> keypoints1, keypoints2;
	//SURF detector
	cv::SurfFeatureDetector detector = cv::SurfFeatureDetector(100);
	//descriptors
	cv::Mat descriptors1, descriptors2;
	//FLANN matcher
	cv::FlannBasedMatcher matcher;
	//matched image
	cv::Mat img_matches;
	//matching pairs
	std::vector<cv::DMatch> matches;
	//selected goodmatches
	std::vector< cv::DMatch > good_matches;
	// all query IDs
	std::vector<int> queryids;
	// unordered set
	std::unordered_map<int, int> idmap;
	// all  train IDs
	std::vector<int> trainids;
	//matching points
	std::vector<cv::KeyPoint> matched_keypoints1;
	std::vector<cv::KeyPoint> matched_keypoints2;
public:
	SURFMATCH();
	~SURFMATCH();
	SURFMATCH(cv::Mat img1, cv::Mat img2);
	std::vector<cv::KeyPoint> getKeypoints1();
	void match(std::vector<cv::KeyPoint>keypoints1,std::vector<int>trainids);
	void setMatches(std::vector<cv::DMatch> matches);
	std::vector<cv::DMatch> getMatches();
	std::vector<cv::Point2f> getMatchedPoints1();
	std::vector<cv::Point2f> getMatchedPoints2();
	std::vector<cv::KeyPoint> getMatchedKeyPoints1();
	std::vector<cv::KeyPoint> getMatchedKeyPoints2();
	std::vector<int> getQueryIDs();
	std::vector<int> getTrainIDs();
	std::unordered_map<int, int> getIDmap();
	cv::Mat getMatchedImg();
};

#endif