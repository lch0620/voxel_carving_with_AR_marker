#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "pose_estimation.hpp"

using namespace std;
using namespace cv;

int main() {
    string imageFolder = "C:/Users/holde/Desktop/voxel_carving_test/Data/Ferrari";
    string imageFile = "C:/Users/holde/Desktop/voxel_carving_test/Data/Ferrari/0FDE6606-15FD-4B6D-82B7-D4ECB058F06E.jpeg";
    string calibrationFile = "C:/Users/holde/Desktop/voxel_carving_test/intrinsic_matrix.yml";
    string outputFilename = "C:/Users/holde/Desktop/voxel_carving_test/extrinsic_matrix.yml";
    float markerLength = 0.02f; // In meters
    float markerSeparation = 0.018f;
    int targetWidth = 1200;
    int targetHeight = 1000;

    Mat cameraMatrix, distCoeffs;
    loadCameraParams(calibrationFile, cameraMatrix, distCoeffs);

    Mat image = imread(imageFile);
    Mat resized_image;
    resize(image, resized_image, Size(targetWidth, targetHeight));

    if (image.empty()) {
        cerr << "Could not open or find the image: " << imageFile << endl;
        return -1;
    }

    cv::imshow("Original image: ", resized_image);
    waitKey(5000);

    aruco::DetectorParameters detectorParams = aruco::DetectorParameters();
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
    aruco::GridBoard board(Size(3, 7), markerLength, markerSeparation, dictionary);
    cv::Vec3d rvec, tvec;

    board_detectAndPose(image, cameraMatrix, distCoeffs, dictionary, detectorParams, markerLength, board, rvec, tvec, outputFilename);
    //individual_detectAndPose(image, cameraMatrix, distCoeffs, dictionary, detectorParams, markerLength, outputFilename);
    return 0;
}
