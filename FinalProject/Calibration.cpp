#include"Calibration.h"
Calibration::Calibration()
{
}

Calibration ::~Calibration()
{
}

//find chessboard corners in 20 input images
void Calibration::FindCorners()
{
	cv::Mat input;
	std::vector<cv::Point2f> corners;
	std::string imagepath = "calibration\\IMG_";
	std::string subfix = ".JPG";
	for (int i = 1; i <= 20; i++)
	{
		std::string filename = imagepath + std::to_string(i) + subfix;
		input = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
		int findresult = cv::findChessboardCorners(input, ChessBoardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH);
		//corner position in sub pixel
		cv::cornerSubPix(input, corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.01));
		cv::drawChessboardCorners(input, ChessBoardSize, corners, findresult);
		allcorners.push_back(corners);
		cv::imshow("detected corners", input);
		cv::waitKey();
	}
}

void Calibration::InitCorners3D(std::vector<std::vector<cv::Point3f>>* Corners3D, cv::Size ChessBoardSize, int NImages, float SquareSize)

{
	int CurrentImage = 0;
	int CurrentRow = 0;
	int CurrentColumn = 0;
	int NPoints = ChessBoardSize.height*ChessBoardSize.width;

	float * temppoints = new float[NImages*NPoints * 3];

	for (CurrentImage = 0; CurrentImage < NImages; CurrentImage++)
	{
		std::vector<cv::Point3f> one_Corners3D;

		cv::Mat temppointstemp;

		for (CurrentRow = 0; CurrentRow < ChessBoardSize.height; CurrentRow++)

		{

			for (CurrentColumn = 0; CurrentColumn < ChessBoardSize.width; CurrentColumn++)
			{
				temppoints[(CurrentImage*NPoints * 3) + (CurrentRow*ChessBoardSize.width +

					CurrentColumn) * 3] = (float)CurrentRow*SquareSize;

				temppoints[(CurrentImage*NPoints * 3) + (CurrentRow*ChessBoardSize.width +

					CurrentColumn) * 3 + 1] = (float)CurrentColumn*SquareSize;

				temppoints[(CurrentImage*NPoints * 3) + (CurrentRow*ChessBoardSize.width +

					CurrentColumn) * 3 + 2] = 0.f;

			}

		}

		temppointstemp = cv::Mat(NPoints, 3, CV_32FC1, temppoints);

		one_Corners3D = cv::Mat_<cv::Point3f>(temppointstemp);

		(*Corners3D).push_back((cv::Mat)one_Corners3D);
	}
}

//calibration
void Calibration::Calibrate()
{
	InitCorners3D(&object_points, ChessBoardSize, img_nums, square_width);
	cv::FileStorage fs("ChessboardCorners.yml", cv::FileStorage::WRITE);
	fs << "object" << object_points;
	fs << "allcorners" << allcorners;
	fs.release();
	cv::calibrateCamera(object_points, allcorners, cv::Size(image_width, image_height),
		intrinsic, distortion_coeff, rotation_vectors, translation_vectors);
	std::cout << "Intrinsic Matrix:\n" << intrinsic<<"\n";
	std::cout << "Distortion Coefficients:\n" << distortion_coeff<<"\n";
	cv::waitKey();
	fs = cv::FileStorage("CalibrationResults.yml", cv::FileStorage::WRITE);
	fs << "intrinsic" << intrinsic;
	fs << "distortion_coeff" << distortion_coeff;
	fs << "rotation_matrices" << rotation_vectors;
	fs << "translation_matrices" << translation_vectors;
	fs.release();
}

void Calibration::Undistort()
{
	cv::Mat srcun, dstun;
	for (int i = 1; i <= 20; i++)
	{
		srcun = ReadImg(i, "calibration");
		cv::undistort(srcun, dstun, intrinsic, distortion_coeff);
		std::string name = "undistort_chessboard\\IMG_";
		std::string subfix = ".JPG";
		std::string filename = name + std::to_string(i) + subfix;
		cv::imshow("undistort image", dstun);
		cv::imwrite(filename, dstun);
		cv::waitKey();
	}
}

double Calibration::computeReprojectionErrors()
{
	std::vector<cv::Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(object_points.size());

	for (i = 0; i < (int)object_points.size(); ++i)
	{
		projectPoints(cv::Mat(object_points[i]), rotation_vectors[i], translation_vectors[i], intrinsic,
			distortion_coeff, imagePoints2);
		err = cv::norm(cv::Mat(allcorners[i]), cv::Mat(imagePoints2), CV_L2);

		int n = (int)object_points
			[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}
	cv::FileStorage fs("ReprojectionErrors.yml", cv::FileStorage::WRITE);
	fs << "Per_View_Reprojection_Errors" << perViewErrors;
	fs << "Avg_Reprojection_Error" << std::sqrt(totalErr / totalPoints);
	fs.release();
	return std::sqrt(totalErr / totalPoints);
}