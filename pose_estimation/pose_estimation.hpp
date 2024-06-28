#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

void loadCameraParams(const std::string& filename, cv::Mat& cameraMatrix, cv::Mat& distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
}

cv::Mat computeExtrinsicMatrix(const cv::Vec3d& rvec, const cv::Vec3d& tvec) {
    cv::Mat rotationMatrix;
    cv::Rodrigues(rvec, rotationMatrix);

    cv::Mat extrinsicMatrix = cv::Mat::eye(4, 4, rotationMatrix.type());
    rotationMatrix.copyTo(extrinsicMatrix(cv::Rect(0, 0, 3, 3)));
    cv::Mat(tvec).copyTo(extrinsicMatrix(cv::Rect(3, 0, 1, 3)));

    return extrinsicMatrix;
}

void saveExtrinsicMatrix(const std::string& filename, const cv::Mat& extrinsicMatrix) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Failed to open " << filename << " for writing." << std::endl;
        return;
    }
    fs << "extrinsic_matrix" << extrinsicMatrix;
    fs.release();
}

void board_detectAndPose(const cv::Mat& image, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    const cv::aruco::Dictionary& dictionary, const cv::aruco::DetectorParameters& detectorParams,
    float markerLength, const cv::aruco::Board& board, cv::Vec3d& rvec, cv::Vec3d& tvec,
    const std::string& outputFilename) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    detector.detectMarkers(image, markerCorners, markerIds);

    if (markerIds.empty()) {
        std::cout << "No markers detected." << std::endl;
        return;
    }

    int markersOfBoardDetected = 0;
    cv::Mat objPoints, imgPoints;

    if (!markerIds.empty()) {
        board.matchImagePoints(markerCorners, markerIds, objPoints, imgPoints);
        cv::solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs, rvec, tvec);
        markersOfBoardDetected = static_cast<int>(objPoints.total() / 4);
    }

    cv::Mat outputImage = image.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

    if (markersOfBoardDetected > 0)
        cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvec, tvec, markerLength * 1.5f);

    int targetWidth = 1200;
    int targetHeight = 1000;
    cv::Mat resized_image;
    cv::resize(outputImage, resized_image, cv::Size(targetWidth, targetHeight));
    cv::imshow("Pose Estimation", resized_image);
    cv::imwrite("Pose_Estimation.jpg", resized_image);
    cv::waitKey(0);

    // Compute and save extrinsic matrix
    cv::Mat extrinsicMatrix = computeExtrinsicMatrix(rvec, tvec);
    saveExtrinsicMatrix(outputFilename, extrinsicMatrix);
}

void individual_detectAndPose(const cv::Mat& image, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    const cv::aruco::Dictionary& dictionary, const cv::aruco::DetectorParameters& detectorParams,
    float markerLength, const std::string& outputFilename) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    detector.detectMarkers(image, markerCorners, markerIds);

    if (markerIds.empty()) {
        std::cout << "No markers detected." << std::endl;
        return;
    }

    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

    size_t nMarkers = markerCorners.size();
    std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

    if (!markerIds.empty()) {
        for (size_t i = 0; i < nMarkers; i++) {
            cv::solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
        }
    }

    cv::Mat outputImage = image.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

    for (int i = 0; i < markerIds.size(); i++) {
        cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
        std::cout << "Marker ID: " << markerIds[i] << " rvec: " << rvecs[i] << " tvec: " << tvecs[i] << std::endl;

        // Compute and save extrinsic matrix for each marker
        cv::Mat extrinsicMatrix = computeExtrinsicMatrix(rvecs[i], tvecs[i]);
        std::string individualOutputFilename = outputFilename + "_marker_" + std::to_string(markerIds[i]) + ".yml";
        saveExtrinsicMatrix(individualOutputFilename, extrinsicMatrix);
    }

    int targetWidth = 1200;
    int targetHeight = 1000;
    cv::Mat resized_image;
    cv::resize(outputImage, resized_image, cv::Size(targetWidth, targetHeight));
    cv::imshow("Pose Estimation", resized_image);
    cv::waitKey(0);
}
