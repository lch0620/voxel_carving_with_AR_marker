#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

void saveProjectionMatrix(const std::string& filename, const cv::Mat& intrinsic, const cv::Mat& extrinsic) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    cv::Mat projection = intrinsic * extrinsic;
    for (int i = 0; i < projection.rows; ++i) {
        for (int j = 0; j < projection.cols; ++j) {
            file << projection.at<double>(i, j) << " ";
        }
        file << std::endl;
    }

    file.close();
}

int main() {
    // Parameters for the calibration pattern
    int numCornersHor = 9;
    int numCornersVer = 6;
    int numImages = 20; // Number of images used for calibration
    float squareSize = 1.0; // Size of a square in your defined unit (point, millimeter, etc.)

    // Create a vector to store points in world coordinates and image coordinates
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;

    // Prepare the object points (0,0,0), (1,0,0), (2,0,0), ..., (8,5,0)
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < numCornersVer; ++i) {
        for (int j = 0; j < numCornersHor; ++j) {
            obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    // Load calibration images
    std::string image_directory = "C:/Users/holde/Desktop/voxel_carving_test/Data/Ferrari"; // Adjust this path
    std::vector<cv::String> filenames;
    cv::glob(image_directory + "/*.jpeg", filenames);

    cv::Size imageSize;

    for (const auto& filename : filenames) {
        cv::Mat image = cv::imread(filename);
        if (image.empty()) {
            std::cerr << "Failed to load image: " << filename << std::endl;
            continue;
        }

        imageSize = image.size();
        std::vector<cv::Point2f> corners;

        // Find the chessboard corners
        bool found = cv::findChessboardCorners(image, cv::Size(numCornersHor, numCornersVer), corners);

        if (found) {
            // Improve the found corners' coordinate accuracy with subpixel refinement
            cv::Mat gray;
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));

            // Draw the corners
            cv::drawChessboardCorners(image, cv::Size(numCornersHor, numCornersVer), corners, found);
            cv::imshow("Chessboard", image);
            cv::waitKey(100);

            imagePoints.push_back(corners);
            objectPoints.push_back(obj);
        }
    }
    cv::destroyAllWindows();

    // Camera calibration
    cv::Mat intrinsic = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, intrinsic, distCoeffs, rvecs, tvecs);

    std::cout << "Calibration RMS error: " << rms << std::endl;

    // Save projection matrices
    std::string matrix_directory = "C:/Users/holde/Desktop/voxel_carving_test/Data/Matrix"; // Adjust this path
    fs::create_directories(matrix_directory);

    for (size_t i = 0; i < filenames.size(); ++i) {
        cv::Mat rotation;
        cv::Rodrigues(rvecs[i], rotation);
        cv::Mat extrinsic = cv::Mat::eye(4, 4, CV_64F);
        rotation.copyTo(extrinsic(cv::Rect(0, 0, 3, 3)));
        tvecs[i].copyTo(extrinsic(cv::Rect(3, 0, 1, 3)));

        std::string filename = matrix_directory + "/matrix" + std::to_string(i + 1) + ".txt";
        saveProjectionMatrix(filename, intrinsic, extrinsic(cv::Rect(0, 0, 4, 3)));
    }

    std::cout << "Projection matrices saved." << std::endl;

    return 0;
}
