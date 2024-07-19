#ifndef VOXEL_CARVING_HPP
#define VOXEL_CARVING_HPP

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <chrono>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

cv::Point3f VoxelToWorld(double x, double y, double z) {
    cv::Point3f point; //actual coordinates in bounding box

    double bbX = 3; //physical length of the bounding box
    double bbY = 3;
    double bbZ = 3;

    double stepX = bbX / 110; // step size in x direction, denominator depends on grid definition
    double stepY = bbY / 110; // step size in y direction
    double stepZ = bbZ / 110; // step size in z direction

    double startX = -bbX / 2; // center the bounding box at origin
    double startY = -bbY / 2;
    double startZ = -bbZ / 2;

    point.x = startX + stepX * (x + 0.5);
    point.y = startY + stepY * (y + 0.5);
    point.z = startZ + stepZ * (z + 0.5);
    //std::cout << "Currently grid ID: " << x << " " << y << " " << z << " " << std::endl;
    //std::cout << "Coordinate: " << point << std::endl;
    return point;
}

// Function to convert cv::Mat to cv::Vec3d
cv::Vec3d matToVec3d(const cv::Mat& mat) {
    CV_Assert(mat.rows == 3 && mat.cols == 1 && mat.type() == CV_64F); // Ensure the mat is 3x1 and of type double
    return cv::Vec3d(mat.at<double>(0, 0), mat.at<double>(1, 0), mat.at<double>(2, 0));
}

cv::Mat computeProjectionMatrix(const cv::Mat& intrinsicMatrix, const cv::Mat& rvec, const cv::Mat& tvec) {
    // Convert rotation vector to rotation matrix
    cv::Mat rotationMatrix;
    cv::Rodrigues(rvec, rotationMatrix); //3x3

    // Create the 3x4 extrinsic matrix
    cv::Mat extrinsicMatrix = cv::Mat::zeros(3, 4, CV_64F);
    rotationMatrix.copyTo(extrinsicMatrix(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(extrinsicMatrix(cv::Rect(3, 0, 1, 3)));

    // Invert the rotation matrix (transpose it)
    //cv::Mat R_inv = rotationMatrix.t();
    // Invert the translation vector
    //cv::Mat t_inv = -R_inv * tvec;
    //R_inv.copyTo(extrinsicMatrix(cv::Rect(0, 0, 3, 3)));
    //t_inv.copyTo(extrinsicMatrix(cv::Rect(3, 0, 1, 3)));
    
    //std::cout << extrinsicMatrix << endl;

    // Compute the projection matrix by multiplying the intrinsic matrix with the extrinsic matrix
    cv::Mat projectionMatrix = intrinsicMatrix * extrinsicMatrix;

    // Convert the 3x4 projection matrix to a 4x4 matrix
    cv::Mat projectionMatrix4x4 = cv::Mat::eye(4, 4, CV_64F); // Start with an identity matrix
    projectionMatrix.copyTo(projectionMatrix4x4(cv::Rect(0, 0, 4, 3)));

    return projectionMatrix4x4;
}


// Function to project voxel to image coordinates
cv::Point2f projectVoxelToImage(const cv::Point3f& voxelCenter, const cv::Mat& projectionMatrix) {
    cv::Mat voxelHomogeneous = (cv::Mat_<double>(4, 1) << voxelCenter.x, voxelCenter.y, voxelCenter.z, 1);
    cv::Mat imagePointHomogeneous = projectionMatrix * voxelHomogeneous;
    return cv::Point2f(imagePointHomogeneous.at<double>(0) / imagePointHomogeneous.at<double>(2),
        imagePointHomogeneous.at<double>(1) / imagePointHomogeneous.at<double>(2));
}

// Function to perform voxel carving
void voxelCarving(std::vector<std::vector<std::vector<bool>>>& voxelGrid,
    const std::vector<cv::Mat>& cameraMatrices,
    const std::vector<cv::Mat>& foregroundMasks) {
    int gridX = voxelGrid.size();
    int gridY = voxelGrid[0].size();
    int gridZ = voxelGrid[0][0].size();
    int numCameras = cameraMatrices.size();

    for (size_t camIdx = 0; camIdx < cameraMatrices.size(); ++camIdx) {
        Mat whiteMatrix = foregroundMasks[camIdx];
        //int outOfBoundsCount = 0;
        for (int x = 0; x < gridX; ++x) {
            for (int y = 0; y < gridY; ++y) {
                for (int z = 0; z < gridZ; ++z) {
                    bool visibleInAllViews = true;
                    cv::Point3f voxelCenter(x, y, z);
                    cv::Point3f voxel_coor = VoxelToWorld(x, y, z);
                    cv::Point2f imagePoint = projectVoxelToImage(voxel_coor, cameraMatrices[camIdx]);
                    int u = static_cast<int>(imagePoint.x);
                    int v = static_cast<int>(imagePoint.y);

                    if (u >= 0 && v >= 0 && u < foregroundMasks[camIdx].cols && v < foregroundMasks[camIdx].rows) {
                        uchar maskValue = foregroundMasks[camIdx].at<uchar>(v, u);
                        if (foregroundMasks[camIdx].at<Vec3b>(v, u).val[0] == 0 && foregroundMasks[camIdx].at<Vec3b>(v, u).val[1] == 0 && foregroundMasks[camIdx].at<Vec3b>(v, u).val[2] == 0) {
                            visibleInAllViews = false;
                        }
                        else {
                            cv::Point center(u, v);
                            int radius = 5;
                            cv::Scalar color(255, 0, 0);
                            int thickness = -1;
                            cv::circle(whiteMatrix, center, radius, color, thickness);

                        }
                    }
                    if (visibleInAllViews == false) {
                        voxelGrid[x][y][z] = false;
                    }
                }
            }
        }
        //if (camIdx % 100 == 0) {
            cv::resize(whiteMatrix, whiteMatrix, Size(1200, 1000));
            cv::imshow("debug", whiteMatrix);
            cv::waitKey(0);
        //}
    }
}
#endif