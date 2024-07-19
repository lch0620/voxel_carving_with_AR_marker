#ifndef CALIBRATION_NORMAL_CHECKERBOARD_HPP
#define CALIBRATION_NORMAL_CHECKERBOARD_HPP

#include <iostream>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>

using namespace std;
using namespace cv;
namespace fs = std::filesystem;

class Calibration2 {
public:
    Calibration2(const string& imageFolder, int squaresX, int squaresY, float squareLength, const string& outputFile, bool showChessboardCorners);
    void runCalibration2();

private:
    string imageFolder;
    int squaresX;
    int squaresY;
    float squareLength;
    string outputFile;
    bool showChessboardCorners;
    int targetWidth = 1200;
    int targetHeight = 1000;

    void drawChessboardCorners(Mat& image, const vector<Point2f>& corners, Size boardSize);
};

Calibration2::Calibration2(const string& imageFolder, int squaresX, int squaresY, float squareLength, const string& outputFile, bool showChessboardCorners)
    : imageFolder(imageFolder), squaresX(squaresX), squaresY(squaresY), squareLength(squareLength), outputFile(outputFile), showChessboardCorners(showChessboardCorners) {}

void Calibration2::runCalibration2() {
    // Collect data from each frame
    vector<vector<Point2f>> allImagePoints;
    vector<vector<Point3f>> allObjectPoints;
    vector<Mat> allImages;
    Size imageSize;

    Size boardSize(squaresX, squaresY);

    for (const auto& entry : fs::directory_iterator(imageFolder)) {
        if (entry.is_regular_file()) {
            Mat image = imread(entry.path().string());

            if (image.empty()) {
                cerr << "Could not open or find the image: " << entry.path().string() << endl;
                continue;
            }

            Mat imageCopy;
            vector<Point2f> imagePoints;
            bool found = findChessboardCorners(image, boardSize, imagePoints,
                CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

            // Draw results on the original image
            image.copyTo(imageCopy);
            if (found) {
                drawChessboardCorners(imageCopy, imagePoints, boardSize);

                // Refine corner locations
                Mat gray;
                cvtColor(image, gray, COLOR_BGR2GRAY);
                cornerSubPix(gray, imagePoints, Size(11, 11), Size(-1, -1),
                    TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
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

            if (key == 'c' && found) {
                cout << "Frame captured" << endl;

                vector<Point3f> objectPoints;
                for (int i = 0; i < squaresY; i++) {
                    for (int j = 0; j < squaresX; j++) {
                        objectPoints.push_back(Point3f(j * squareLength, i * squareLength, 0));
                    }
                }

                allImagePoints.push_back(imagePoints);
                allObjectPoints.push_back(objectPoints);
                allImages.push_back(image);

                imageSize = image.size();
            }
        }
    }

    if (allImagePoints.size() < 4) {
        cerr << "Not enough corners for Calibration2" << endl;
        return;
    }

    Mat cameraMatrix, distCoeffs;
    int Calibration2Flags = 0;
    float aspectRatio = 1;
    bool fixAspectRatio = false;
    bool zeroTangentialDistortion = false;
    bool fixPrincipalPoint = false;

    // Setting Calibration2 flags
    if (fixAspectRatio) {
        Calibration2Flags |= CALIB_FIX_ASPECT_RATIO;
    }
    if (zeroTangentialDistortion) {
        Calibration2Flags |= CALIB_ZERO_TANGENT_DIST;
    }
    if (fixPrincipalPoint) {
        Calibration2Flags |= CALIB_FIX_PRINCIPAL_POINT;
    }

    if (Calibration2Flags & CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 0) = aspectRatio;
    }

    // Calibrate camera
    double repError = calibrateCamera(allObjectPoints, allImagePoints, imageSize, cameraMatrix, distCoeffs,
        noArray(), noArray(), Calibration2Flags);

    // Save the Calibration2 results
    FileStorage fs(outputFile, FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "camera_matrix" << cameraMatrix;
        fs << "distortion_coefficients" << distCoeffs;
        fs << "reprojection_error" << repError;
        fs.release();
        cout << "Calibration2 saved to " << outputFile << endl;
    }
    else {
        cerr << "Cannot save output file" << endl;
        return;
    }

    cout << "Rep Error: " << repError << endl;

    // Show chessboard corners for debugging
    if (showChessboardCorners) {
        for (size_t frame = 0; frame < allImages.size(); frame++) {
            Mat imageCopy = allImages[frame].clone();

            if (!allImagePoints[frame].empty()) {
                drawChessboardCorners(imageCopy, allImagePoints[frame], boardSize);
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

void Calibration2::drawChessboardCorners(Mat& image, const vector<Point2f>& corners, Size boardSize) {
    for (size_t i = 0; i < corners.size(); i++) {
        Point2f corner = corners[i];
        circle(image, corner, 5, Scalar(0, 0, 255), -1);
        putText(image, to_string(i), corner + Point2f(5, -5), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 2);
    }
}

#endif // CALIBRATION_NORMAL_CHECKERBOARD_HPP
