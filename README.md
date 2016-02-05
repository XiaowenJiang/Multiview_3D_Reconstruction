# Multiview_3D_Reconstruction

This project is aimed at reconstruct 3D scene through 8 views of a certain scene.
There are several classes in the project for different functions:

Class Headers: 
----Calibration.h   //Camera calibration using 20 input images of chessboard images taken by the same camera as the input images

----SURFMatch.h   //Detect keypoints using SURF detector, track points using optical flow and match points by FlannBasedMatcher

----EstimateCameraPose.h  //Estimate camera pose and generate 3D points 

----ProjectPoints.h   //Project 3D points to 2D and show the reprojection difference with the original 2D points

----DenseReconstruction.h   //Draw disparity map from every two views 

----DrawPointCloud.h   //Generate 3D point cloud with PCL library

----Utils.h   //some global functions such as load and store YML files

SBA:
This project takes use of cvsba library, which can be found in the following link:

http://www.uco.es/investiga/grupos/ava/node/39

To use the library, you might have to install BLAS and LAPACK, which may require MinGW. Below is a tutorial:

https://icl.cs.utk.edu/lapack-for-windows/lapack/

Note that to run this project you have to modify the import path of the libraries and property sheets.
