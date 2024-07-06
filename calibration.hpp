#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

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
namespace fs = std::filesystem;

class Calibration {
public:
    Calibration(const string& imageFolder, int squaresX, int squaresY, float squareLength, float markerLength, const string& outputFile, bool showChessboardCorners);
    void runCalibration();

private:
    string imageFolder;
    int squaresX;
    int squaresY;
    float squareLength;
    float markerLength;
    string outputFile;
    bool showChessboardCorners;
    int targetWidth = 1200;
    int targetHeight = 1000;

    void drawMarkerIds(Mat& image, const vector<int>& markerIds, const vector<vector<Point2f>>& markerCorners);
    void drawCharucoCorners(Mat& image, const Mat& charucoCorners, const Mat& charucoIds, double fontSize, Scalar fontColor, int thickness);
};

Calibration::Calibration(const string& imageFolder, int squaresX, int squaresY, float squareLength, float markerLength, const string& outputFile, bool showChessboardCorners)
    : imageFolder(imageFolder), squaresX(squaresX), squaresY(squaresY), squareLength(squareLength), markerLength(markerLength), outputFile(outputFile), showChessboardCorners(showChessboardCorners) {}

void Calibration::runCalibration() {
    aruco::DetectorParameters detectorParams; // Use default detector parameters
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50); // Use a predefined dictionary

    aruco::CharucoParameters charucoParams;
    bool refindStrategy = false;
    if (refindStrategy) {
        charucoParams.tryRefineMarkers = true;
    }

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
                drawMarkerIds(imageCopy, markerIds, markerCorners); // Draw larger marker IDs
            }

            if (currentCharucoCorners.total() > 3) {
                drawCharucoCorners(imageCopy, currentCharucoCorners, currentCharucoIds, 1.0, Scalar(255, 0, 0), 2);
            }

            // Resize the image for display
            Mat resizedImage;
            resize(imageCopy, resizedImage, Size(targetWidth, targetHeight));

            putText(resizedImage, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
                Point(10, 40), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 2); // Increased font size and thickness

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
        return;
    }

    Mat cameraMatrix, distCoeffs;
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
        return;
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
}

void Calibration::drawMarkerIds(Mat& image, const vector<int>& markerIds, const vector<vector<Point2f>>& markerCorners) {
    for (size_t i = 0; i < markerIds.size(); i++) {
        Point2f center(0, 0);
        for (const auto& corner : markerCorners[i]) {
            center += corner;
        }
        center /= 4.0;

        putText(image, "id: " + to_string(markerIds[i]), center, FONT_HERSHEY_SIMPLEX, 2.0, Scalar(255, 0, 0), 3);
    }
}

void Calibration::drawCharucoCorners(Mat& image, const Mat& charucoCorners, const Mat& charucoIds, double fontSize, Scalar fontColor, int thickness) {
    for (int i = 0; i < charucoCorners.rows; i++) {
        Point2f corner = charucoCorners.at<Point2f>(i, 0);
        int id = charucoIds.at<int>(i, 0);
        circle(image, corner, 5, Scalar(0, 0, 255), -1);
        putText(image, "id: " + to_string(id), corner + Point2f(5, -5), FONT_HERSHEY_SIMPLEX, fontSize, fontColor, thickness);
    }
}

#endif // CALIBRATION_HPP
