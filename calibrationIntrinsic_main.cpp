#include <iostream>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <filesystem>
#include "aruco_samples_utility.hpp"

using namespace std;
using namespace cv;
namespace fs = filesystem;

namespace {
    const char* about =
        "Calibration using a ChArUco board\n"
        "  To capture a frame for calibration, press 'c',\n"
        "  If input comes from video, press any key for next frame\n"
        "  To finish capturing, press 'ESC' key and calibration starts.\n";
}

int _main() {
    std::cout << "Started calibration main..." << std::endl;
    int squaresX = 8; // Number of squares in X direction
    int squaresY = 11; // Number of squares in Y direction
    float squareLength = 0.015f; // Square side length (in meters)
    float markerLength = 0.011f; // Marker side length (in meters)
    string outputFile = "output.yml"; // Output file with calibrated camera parameters
    string imageFolder = "C:/Assignments/3DScanning/Project/environment/Exercise-2/Data/checkerboard"; // Input folder with images
    bool showChessboardCorners = true; // Show detected chessboard corners after calibration

    int calibrationFlags = 0;
    float aspectRatio = 1;
    bool fixAspectRatio = false;
    bool zeroTangentialDistortion = false;
    bool fixPrincipalPoint = false;

    // Setting calibration flags
    if (fixAspectRatio) {
        calibrationFlags |= CALIB_FIX_ASPECT_RATIO;
    }
    if (zeroTangentialDistortion) {
        calibrationFlags |= CALIB_ZERO_TANGENT_DIST;
    }
    if (fixPrincipalPoint) {
        calibrationFlags |= CALIB_FIX_PRINCIPAL_POINT;
    }

    aruco::DetectorParameters detectorParams; // Use default detector parameters
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50); // Use a predefined dictionary

    aruco::CharucoParameters charucoParams;
    bool refindStrategy = false;
    if (refindStrategy) {
        charucoParams.tryRefineMarkers = true;
    }

    // Define target width and height for resizing images
    int targetWidth = 1200;
    int targetHeight = 1000;

    // Create charuco board object and CharucoDetector
    aruco::CharucoBoard board(Size(squaresX, squaresY), squareLength, markerLength, dictionary);
    aruco::CharucoDetector detector(board, charucoParams, detectorParams);

    // Collect data from each frame
    vector<Mat> allCharucoCorners, allCharucoIds;
    vector<vector<Point2f>> allImagePoints;
    vector<vector<Point3f>> allObjectPoints;
    vector<Mat> allImages;
    Size imageSize;

    for (const auto& entry : fs::directory_iterator(imageFolder)) {
        if (entry.is_regular_file()) {
            Mat image = imread(entry.path().string());

            if (image.empty()) {
                cerr << "Could not open or find the image: " << entry.path().string() << endl;
                continue;
            }

            Mat imageCopy;
            vector<int> markerIds;
            vector<vector<Point2f>> markerCorners;
            Mat currentCharucoCorners, currentCharucoIds;
            vector<Point3f> currentObjectPoints;
            vector<Point2f> currentImagePoints;

            // Detect ChArUco board
            detector.detectBoard(image, currentCharucoCorners, currentCharucoIds);

            // Draw results on the original image
            image.copyTo(imageCopy);
            if (!markerIds.empty()) {
                aruco::drawDetectedMarkers(imageCopy, markerCorners);
            }

            if (currentCharucoCorners.total() > 3) {
                aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
            }

            // Resize the image for display
            Mat resizedImage;
            resize(imageCopy, resizedImage, Size(targetWidth, targetHeight));

            putText(resizedImage, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
                Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

            imshow("out", resizedImage);
            resizeWindow("out", resizedImage.cols, resizedImage.rows); // Resize window to fit image size

            // Wait for key pressed
            char key = (char)waitKey(0);

            if (key == 27) {
                break;
            }

            if (key == 'c' && currentCharucoCorners.total() > 3) {
                // Match image points
                board.matchImagePoints(currentCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints);

                if (currentImagePoints.empty() || currentObjectPoints.empty()) {
                    cout << "Point matching failed, try again." << endl;
                    continue;
                }

                cout << "Frame captured" << endl;

                allCharucoCorners.push_back(currentCharucoCorners);
                allCharucoIds.push_back(currentCharucoIds);
                allImagePoints.push_back(currentImagePoints);
                allObjectPoints.push_back(currentObjectPoints);
                allImages.push_back(image);

                imageSize = image.size();
            }
        }
    }

    if (allCharucoCorners.size() < 4) {
        cerr << "Not enough corners for calibration" << endl;
        return 0;
    }

    Mat cameraMatrix, distCoeffs;

    if (calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 0) = aspectRatio;
    }

    // Calibrate camera using ChArUco
    double repError = calibrateCamera(allObjectPoints, allImagePoints, imageSize, cameraMatrix, distCoeffs,
        noArray(), noArray(), noArray(), noArray(), noArray(), calibrationFlags);

    bool saveOk = saveCameraParams(outputFile, imageSize, aspectRatio, calibrationFlags,
        cameraMatrix, distCoeffs, repError);

    if (!saveOk) {
        cerr << "Cannot save output file" << endl;
        return 0;
    }

    cout << "Rep Error: " << repError << endl;
    cout << "Calibration saved to " << outputFile << endl;

    // Show interpolated charuco corners for debugging
    if (showChessboardCorners) {
        for (size_t frame = 0; frame < allImages.size(); frame++) {
            Mat imageCopy = allImages[frame].clone();

            if (allCharucoCorners[frame].total() > 0) {
                aruco::drawDetectedCornersCharuco(imageCopy, allCharucoCorners[frame], allCharucoIds[frame]);
            }

            // Resize the image for display
            Mat resizedImage;
            resize(imageCopy, resizedImage, Size(targetWidth, targetHeight));

            imshow("out", resizedImage);
            resizeWindow("out", resizedImage.cols, resizedImage.rows); // Resize window to fit image size

            char key = (char)waitKey(0);
            if (key == 27) {
                break;
            }
        }
    }
    return 0;
}
