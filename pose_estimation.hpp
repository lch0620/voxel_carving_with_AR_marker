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
    float markerLength, const cv::aruco::Board& board, cv::Vec3d& rvec, cv::Vec3d& tvec)
    /*
        Return: the pose of the board

    */

    ///Return: the pose of the board
{
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    //std::vector<Vec3d> rvecs, tvecs;
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);


    detector.detectMarkers(image, markerCorners, markerIds);

    if (markerIds.empty()) {
        std::cout << "No markers detected." << std::endl;
        return;
    }

    // Estimate board pose
    int markersOfBoardDetected = 0;

    // Get object and image points for the solvePnP function
    cv::Mat objPoints, imgPoints;
    board.matchImagePoints(markerCorners, markerIds, objPoints, imgPoints);

    // Estimate board pose
    if (!markerIds.empty()) {
        // Get object and image points for the solvePnP function
        cv::Mat objPoints, imgPoints;
        board.matchImagePoints(markerCorners, markerIds, objPoints, imgPoints);

        // Find pose
        cv::solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs, rvec, tvec);

        markersOfBoardDetected = (int)objPoints.total() / 4;
    }


    /*
    ------ Individual markers detection
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
            cv::solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
        }
    }
    */


    // Draw Markers and Frame Axes
    cv::Mat outputImage = image.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
    
    /*
    Draw for individual markers
    for (int i = 0; i < markerIds.size(); i++) {
        cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 0.5f);
        std::cout << "Marker ID: " << markerIds[i] << " rvec: " << rvecs[i] << " tvec: " << tvecs[i] << std::endl;
    }
    */

    // draw for AruCo Board
    if (markersOfBoardDetected > 0)
        cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvec, tvec, markerLength*1.5f);
    
    int targetWidth = 1200;
    int targetHeight = 1000;

    // Resize the image for display
    cv::Mat resized_image;
    cv::resize(outputImage, resized_image, cv::Size(targetWidth, targetHeight));

    cv::imshow("Pose Estimation", resized_image);
    cv::waitKey(0);
}