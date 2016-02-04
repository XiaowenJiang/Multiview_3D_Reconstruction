#include"SURFMatch.h"

SURFMATCH::SURFMATCH()
{}

SURFMATCH::~SURFMATCH()
{}

SURFMATCH::SURFMATCH(cv::Mat inputimg1, cv::Mat inputimg2)
{
	// check if images are correctly read
	if (!inputimg1.data || !inputimg2.data)
	{
		std::cout << "no data in input matrices\n";
		return;
	}
	input1 = inputimg1;
	input2 = inputimg2;
	
	//detect keypoints using SURF detector
	detector.detect(input1, keypoints1);
	detector.detect(input2, keypoints2);
}

std::vector<cv::KeyPoint> SURFMATCH::getKeypoints1()
{
	return keypoints1;
}

void SURFMATCH::match(std::vector<cv::KeyPoint>prepoints1,std::vector<int>preids)
{
	//to tab if the keypoint is found in the second image
	std::vector<uchar>vstatus;
	//error of this keypoint
	std::vector<float>verror;
	std::vector<cv::Point2f> points1 = KeypointsToPoints(prepoints1);
	std::vector<cv::Point2f> points2 = KeypointsToPoints(keypoints2);
	std::vector<cv::Point2f> opticalflow_points2;
	std::vector<cv::Point2f> opticalflow_points2_refined;
	std::vector<int> opticalflow_points2_refined_originalindex;
	cv::calcOpticalFlowPyrLK(input1, input2, points1, opticalflow_points2, vstatus, verror);
	for (int i = 0; i < vstatus.size(); i++)
	{
		
		if (vstatus[i] && verror[i] < 9)
		{
			if (preids.size()!=0)
			{ 
				idmap.insert(std::make_pair(i, preids[i]));
				opticalflow_points2_refined_originalindex.push_back(preids[i]);
			}
			else
			{
				opticalflow_points2_refined_originalindex.push_back(i);
			}
			opticalflow_points2_refined.push_back(opticalflow_points2[i]);
		}
	}
	cv::Mat points2_refined_flat = cv::Mat(opticalflow_points2_refined).reshape(1, opticalflow_points2_refined.size());
	cv::Mat points2_flat = cv::Mat(points2).reshape(1, points2.size());
	matcher.match(points2_refined_flat, points2_flat, matches);
	std::set<int> index_found;
	for (int i = 0; i < matches.size(); i++)
	{
		cv::DMatch m = matches[i];
		m.queryIdx = opticalflow_points2_refined_originalindex[m.queryIdx];
		if (index_found.find(m.trainIdx) == index_found.end())
		{
			good_matches.push_back(m);
			index_found.insert(m.trainIdx);
		}
		else if (preids.size()!=0)
		{
			idmap.erase(m.queryIdx);
		}
	}
	
	//store the Keypoints corresponding to the matches
	long num_matches = good_matches.size();
	for (int i = 0; i<num_matches; i++)
	{
		int idx1 = good_matches[i].queryIdx;
		int idx2 = good_matches[i].trainIdx;
		trainids.push_back(idx2);
		queryids.push_back(idx1);
	    matched_keypoints1.push_back(keypoints1[idx1]);
		matched_keypoints2.push_back(keypoints2[idx2]);
	}
	// drawing the matching

	cv::namedWindow("matches", 1);
	drawMatches(input1, keypoints1, input2, keypoints2, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector< char >(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	cv::imshow("matches", img_matches);
	cv::waitKey();
}

cv::Mat SURFMATCH::getMatchedImg()
{
	return img_matches;
}

void SURFMATCH::setMatches(std::vector<cv::DMatch> matches)
{
	good_matches = matches;
	trainids.clear();
	queryids.clear();
	matched_keypoints1.clear();
	matched_keypoints2.clear();
	long num_matches = good_matches.size();
	for (int i = 0; i<num_matches; i++)
	{
		int idx1 = good_matches[i].queryIdx;
		int idx2 = good_matches[i].trainIdx;
		trainids.push_back(idx2);
		queryids.push_back(idx1);
		matched_keypoints1.push_back(keypoints1[idx1]);
		matched_keypoints2.push_back(keypoints2[idx2]);
	}
}

std::vector<cv::DMatch> SURFMATCH::getMatches()
{
	return good_matches;
}

//return all queryIDs in this match
std::vector<int> SURFMATCH::getQueryIDs()
{
	return queryids;
}

std::vector<int> SURFMATCH::getTrainIDs()
{
	return trainids;
}

//get Matched Keypoints from the first frame
std::vector<cv::KeyPoint> SURFMATCH::getMatchedKeyPoints1()
{
	return matched_keypoints1;
}

//get Matched Keypoints from the second frame
std::vector<cv::KeyPoint> SURFMATCH::getMatchedKeyPoints2()
{	
	return matched_keypoints2;
}

std::vector<cv::Point2f> SURFMATCH::getMatchedPoints1()
{
	return KeypointsToPoints(matched_keypoints1);
}

std::vector<cv::Point2f> SURFMATCH::getMatchedPoints2()
{
	return KeypointsToPoints(matched_keypoints2);
}

std::unordered_map<int, int> SURFMATCH::getIDmap()
{
	return idmap;
}

