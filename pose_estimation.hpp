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

void detectAndEstimatePose(const cv::Mat& image, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    const cv::aruco::Dictionary& dictionary, const cv::aruco::DetectorParameters& detectorParams,
    float markerLength) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    //std::vector<Vec3d> rvecs, tvecs;
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    detector.detectMarkers(image, markerCorners, markerIds);

    if (markerIds.empty()) {
        std::cout << "No markers detected." << std::endl;
        return;
    }

    // set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

    //cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

    size_t nMarkers = markerCorners.size();
    std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

    if (!markerIds.empty()) {
        // Calculate pose for each marker
        for (size_t i = 0; i < nMarkers; i++) {
            solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
        }
    }

    // Draw Markers and Frame Axes
    cv::Mat outputImage = image.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
    
    for (int i = 0; i < markerIds.size(); i++) {
        cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
        std::cout << "Marker ID: " << markerIds[i] << " rvec: " << rvecs[i] << " tvec: " << tvecs[i] << std::endl;
    }
    
    cv::imshow("Pose Estimation", outputImage);
    cv::waitKey(0);
}