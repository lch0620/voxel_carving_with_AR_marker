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

    string imageFolder = "C:/Assignments/3DScanning/Project/environment/Exercise-2/Data/Owl";
    string imageFile = "C:/Assignments/3DScanning/Project/environment/Exercise-2/Data/Owl/4F75CFBB-A5E1-402B-A7F4-DD04B8AC2C64.jpeg";
    string calibrationFile = "C:/Assignments/3DScanning/Project/environment/Exercise-2/output.yml";
    float markerLength = 0.02f; // In meters
    float markerSeparation = 0.018f;
    // Define target width and height for resizing images
    int targetWidth = 1200;
    int targetHeight = 1000;
    

    Mat cameraMatrix, distCoeffs;
    loadCameraParams(calibrationFile, cameraMatrix, distCoeffs);
    
    Mat image = imread(imageFile);
    // Resize the image for display
    Mat resized_image;
    resize(image, resized_image, Size(targetWidth, targetHeight));


    if (image.empty()) {
        cerr << "Could not open or find the image: " << imageFile << endl;
        return -1;
    }

    cv::imshow("Original image: ", resized_image);
    waitKey(5000);

    // Board detection, pose estimation parameters
    aruco::DetectorParameters detectorParams = aruco::DetectorParameters();
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
    aruco::GridBoard board(Size(3, 7), markerLength, markerSeparation, dictionary);
    cv::Vec3d rvec, tvec;

    board_detectAndPose(image, cameraMatrix, distCoeffs, dictionary, detectorParams, markerLength, board, rvec, tvec);
    //individual_detectAndPose(image, cameraMatrix, distCoeffs, dictionary, detectorParams, markerLength, board);
    return 0;
}
