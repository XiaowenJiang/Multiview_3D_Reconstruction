#include<iostream>
#include<opencv2\calib3d\calib3d.hpp>
#include<opencv2\core\core.hpp>
#include<opencv2\highgui\highgui.hpp>
#include<opencv2\imgproc\imgproc.hpp>
#include<features2d\features2d.hpp>
#include<opencv2\ml\ml.hpp>
#include<vector>
//sub_classes
#include"Calibration.h"
#include"SURFMatch.h"
#include"EstimateCameraPose.h"
#include"ProjectPoints.h"
#include"DrawPointCloud.h"
#include"DenseReconstruction.h"
//sparse bundle adjustment
#include"cvsba-1.0.0\src\cvsba.h"
//3D point cloud
#include<pcl\io\pcd_io.h>
#include<pcl\point_types.h>
#include<pcl\visualization\cloud_viewer.h>

using namespace std;
using namespace cv;
using namespace cvsba;
using namespace pcl;


int main()
{
	try
	{
		//STEP 1 : Calibrate camera
		/*Calibration calibration;
		calibration.FindCorners();
		calibration.Calibrate();
		calibration.Undistort();
		calibration.computeReprojectionErrors();*/

		//get intrinsic matrix from YML record
		Mat intrinsic = GetFromYML("CalibrationResults.yml", "intrinsic");
		Mat dist_coeff = GetFromYML("CalibrationResults.yml", "distortion_coeff");
		cout << "Camera Matrix :\n" << intrinsic << endl;
		cout << "Distortion Coefficients: \n" << dist_coeff << endl << endl;

		//STEP 2 : Find matching points of two views using SURF and FLANN matcher
		string imgpath = "input_images";
		int img_num = 8;
		vector<Mat> inputimages;
		vector<Mat> inputimagesRGB;
		vector<vector<KeyPoint>> keypointsarray;
		vector<SURFMATCH> surfmatches;
		vector<vector< DMatch >> matchesarray;
		int i = 1;
		for (i; i <= img_num; i++)
		{
			Mat input = ReadImg(i, imgpath);
			Mat inputRGB = ReadImgRGB(i, imgpath);
			resize(input, input, cv::Size(), 0.2, 0.2);
			resize(inputRGB, inputRGB, cv::Size(), 0.2, 0.2);
			inputimages.push_back(input);
			inputimagesRGB.push_back(inputRGB);
			if (i >= 2)
			{
				surfmatches.push_back(SURFMATCH(inputimages[i-2], inputimages[i-1]));
				if (i == 2)
				{
					vector<int> empty;
					surfmatches[i-2].match(surfmatches[i-2].getKeypoints1(),empty);
				}
				else
				{
					surfmatches[i - 2].match(surfmatches[i - 3].getMatchedKeyPoints2(),surfmatches[i-3].getTrainIDs());
				}
				keypointsarray.push_back(surfmatches[i - 2].getKeypoints1());
				matchesarray.push_back(surfmatches[i - 2].getMatches());
				cout << "There are " + to_string(matchesarray[i-2].size()) + " matches in " + to_string(i-1) + ".\n";
				WriteImg(i - 1, "matches", "matched", surfmatches[i-2].getMatchedImg());
			}
		}

		//STEP 3 : filter the matches, remove those cannot be tracked
		//during the whole sequence
		vector<int> temp;
		vector<DMatch> tempmatches;
		
		for (int i = matchesarray.size() - 2; i >= 0; i--)
		{
			tempmatches.clear();
			vector<int> next_queryids = surfmatches[i + 1].getQueryIDs();
			vector<int> queryids;
			for (int & c : next_queryids)
			{
				if (temp.size()==0)
				{
					queryids.push_back(c);
				}
				else
				{
					for (int & b : temp)
					{
						if (b == c)
						{
							queryids.push_back(c);
							break;
						}
					}
				}
			}
			temp.clear();
			for (int j = 0; j < matchesarray[i].size(); j++)
			{
				for (int count=0; count < queryids.size(); count++)
				{
					if (queryids[count] == matchesarray[i][j].trainIdx)
					{
						temp.push_back(matchesarray[i][j].queryIdx);
						tempmatches.push_back(matchesarray[i][j]);
						break;
					}
				}
			}
			matchesarray[i] = tempmatches;
			surfmatches[i].setMatches(tempmatches);
		}
		cout << "After filtering, there are " + to_string(matchesarray[0].size())
			+ " matches of key points left.\n";

		//STEP 4 : Sparse 3D reconstruct
		// use HZ to estimate the first two camera poses
		// then use the 3D model reconstructed from the first
		// two views and solvePNPRansac to estimate other
		// cameras
		vector<EstimateCameraPose> estimates;
		vector<ProjectPoints> project2d;
		vector<vector<Point2f>> all_project_2dpoints;
		vector<Mat> all_rotations;
		vector<Mat> all_translations;
		Mat Identity = (Mat)Matx33d(1, 0, 0, 0, 1, 0, 0, 0, 1);
		Mat zerotrans = (Mat)Matx13d(0, 0, 0);
		//used for test sba
		vector<Point3f> global3dpoints;
		vector<vector<Point2f>> matchedpoints;
		for (int i = 0; i < surfmatches.size(); i++)
		{
			if (i==0)
			{
				matchedpoints.push_back(surfmatches[i].getMatchedPoints1());
				matchedpoints.push_back(surfmatches[i].getMatchedPoints2());
				estimates.push_back(EstimateCameraPose(matchedpoints[0], matchedpoints[1], intrinsic));
				estimates[i].calcCameraPoseFromE();
				global3dpoints = estimates[0].get3Dpoints();
				all_rotations.push_back(Identity);
				all_rotations.push_back(estimates[0].getRotation());
				all_translations.push_back(zerotrans);
				all_translations.push_back(estimates[0].getTranslation());
				for (int i = 0; i <= 1; i++)
				{	
					project2d.push_back(ProjectPoints(inputimages[i], matchedpoints[i], global3dpoints,
						all_rotations[i], all_translations[i], intrinsic, dist_coeff));
					project2d[i].ShowReprojectionErrors();
					WriteImg(i, "projection_2D_beforebundle", "projection2D", project2d[i].getProjectionImage());
					all_project_2dpoints.push_back(project2d[i].getProjectpoints());
				}
								
			}
			else
			{
				matchedpoints.push_back(surfmatches[i].getMatchedPoints2());
				Mat R, t;
				solvePnPRansac(global3dpoints, matchedpoints[i+1], intrinsic, dist_coeff,
					R, t);
				Mat rotation33;
				cv::Rodrigues(R, rotation33);
				all_rotations.push_back(rotation33);
				all_translations.push_back(t);
				project2d.push_back(ProjectPoints(inputimages[i + 1], matchedpoints[i+1], global3dpoints,
					R,t, intrinsic, dist_coeff));
				project2d[i + 1].ShowReprojectionErrors();
				WriteImg(i+1, "projection_2D_beforebundle", "projection2D", project2d[i+1].getProjectionImage());
				all_project_2dpoints.push_back(project2d[i + 1].getProjectpoints());
			}
		}
		vector<vector<int>> visibility(surfmatches.size() + 1, project2d[0].getVisibility());
		vector<Mat> all_intrinsics(surfmatches.size()+1, intrinsic);
		vector<Mat> all_dis_coeffs(surfmatches.size()+1, dist_coeff);

		//STEP 5 : sparse bundle adjustment on 3D points and all
		//camera poses
		waitKey();

		cout << "\nStart Running Sparse Bundle Adjustment.\n";
		Sba sba;
		sba.run(global3dpoints, all_project_2dpoints, visibility, all_intrinsics,
			all_rotations, all_translations, all_dis_coeffs);
		cout << "initial reprojection error: " << sba.getInitialReprjError() << endl;
		cout << "final reprojection error: " << sba.getFinalReprjError() << endl;
		for (int i = 0; i < img_num; i++)
		{
			ProjectPoints testsba(inputimages[i], matchedpoints[i], global3dpoints, all_rotations[i], all_translations[i],
				all_intrinsics[i], all_dis_coeffs[i]);
			testsba.ShowDifferenceSBA(all_project_2dpoints[i]);
			//testsba.ShowReprojectionErrors();
			WriteImg(i, "projection_2D_afterbundle", "projection2D", testsba.getProjectionImage());
		}
		vector<Point2f> matpoints;
		
		//STEP 6 : Dense 3D reconstruction (disparity map) 
		for (int i = 0; i < img_num - 1; i++)
		{
			EstimateCameraPose es(matchedpoints[i], matchedpoints[i + 1], intrinsic);
			DenseReconstuction dense(es.getFundamental(), inputimages[i], inputimages[i+1],
				matchedpoints[i], matchedpoints[i+1]);
			dense.ShowDisparityMap();
			WriteImg(i, "dense_reconstruction", "dense", dense.getDisparityMap());
		}
		

		Mat global3dpointsmat = Mat(3, global3dpoints.size(),CV_32FC1);
		for (int i = 0; i < global3dpoints.size(); i++)
		{
			global3dpointsmat.at<float>(0, i) = global3dpoints[i].x;
			global3dpointsmat.at<float>(1, i) = global3dpoints[i].y;
			global3dpointsmat.at<float>(2, i) = global3dpoints[i].z;
		}
		WriteYML("points3d.yml", "points3d", global3dpointsmat);
		vector<vector<Point2f>> all_project_2dpoints_beforesba = all_project_2dpoints;


		//STEP 7 : Draw 3D pointcloud (sparse)
		DrawPointCloud pointcloud(global3dpoints,inputimagesRGB[0],all_rotations,all_translations);
		
				
	}
	catch(Exception & e){
		cout << e.msg<<endl;
		waitKey();
	}
	
	return 0;
}