#include "foreground_segmentation.hpp"
#include "calibration_charucoboard.hpp"
#include "calibration_normal_checkerboard.hpp"
#include "pose_estimation.hpp"
#include "voxel_carving.hpp"
#include "obj_generation.hpp"
#include "functions_read_extrinsic_from_txt.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <regex>


// Function to extract the numerical part from the filename (for NeRF data)
int extractNumber(const std::string& filename) {
    std::regex number_pattern(R"(\d+)");
    std::smatch match;
    if (std::regex_search(filename, match, number_pattern)) {
        return std::stoi(match.str());
    }
    return -1;  // Return -1 if no number is found
}


int main(int argc, char** argv) {
 
    /********************************* For our data, uncomment this section **********************************************/

    /* 1. calibration by ChArUcoboard to get the intrinsic */
    /*
    string imageFolder_cal = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/checkerboard";
    int squaresX = 8; // Number of squares in X direction
    int squaresY = 11; // Number of squares in Y direction
    float squareLength = 0.015f; // Square side length (in meters)
    float markerLength = 0.011f; // Marker side length (in meters)
    string outputFile = "output_charucoboard.yml"; // Output file with calibrated camera parameters
    bool showChessboardCorners = false; // Show detected chessboard corners after calibration

    Calibration calibration(imageFolder_cal, squaresX, squaresY, squareLength, markerLength, outputFile, showChessboardCorners);
    calibration.runCalibration();
    */
    /* 1. calibration by ChArUcoboard to get the intrinsic */



    /* 2. calibration by normal checkerboard to get the intrinsic */
    /*
    string imageFolder_cal = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/normal_checkerboard4";
    int squaresX = 6; // Number of squares in X direction
    int squaresY = 9; // Number of squares in Y direction
    float squareLength = 0.0225f; // Square side length (in meters)
    string outputFile = "output_normal_checkerboard.yml"; // Output file with calibrated camera parameters
    bool showChessboardCorners = false; // Show detected chessboard corners after calibration

    Calibration2 calibration(imageFolder_cal, squaresX, squaresY, squareLength, outputFile, showChessboardCorners);
    calibration.runCalibration2();
    */
    /* 2. calibration by normal checkerboard to get the intrinsic */



    /* 3. segmenting images and save them */
    /*
    string imageFolder_seg = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/Ferrari";
    string outputFolder = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/SegmentedImages_Ferrari_blackwhite";
    Scalar COLOR_MIN(15, 45, 45);
    Scalar COLOR_MAX(65, 255, 255);
    int targetWidth = 1200;
    int targetHeight = 1000;

    ImageSegmentation segmenter(imageFolder_seg, outputFolder, COLOR_MIN, COLOR_MAX, targetWidth, targetHeight);
    segmenter.performSegmentation();
    */
    /* 3. segmenting images and save them */
    
    

    /* 4. get board pose, retrieve camera extrinsic and get the final projection matrices */
    /*
    string inputFolder_oriImg = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/Ferrari";
    float markerLength = 0.02f;
    float markerSeparation = 0.018f;

    // hard code the intrinsic and distortion coefficent retrieved from calibration part
    float scale = 1; // 0.125;
    cv::Mat intrinsicMatrix = (cv::Mat_<double>(3, 3) << 
        3038.248502423437  * scale, 0., 2048.6800329991179 * scale,   // can get correct poses
        0., 3049.5886788585231 * scale, 1501.8827722101619 * scale, 
        0., 0., 1.);

    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) <<
        0.17046182312224414,
        -0.69286205708434745,
        -0.0059834074266772496,
        0.0027923172265777603,
        0.9916983456943318);

    aruco::DetectorParameters detectorParams = aruco::DetectorParameters();
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
    aruco::GridBoard board(Size(5, 7), markerLength, markerSeparation, dictionary);
    cv::Vec3d rvec, tvec;

    // Placeholder for camera matrices (projection matrices)
    std::vector<cv::Mat> projectionMatrices; // List of projection matrices
    int j = 0;
    // Load camera matrix
    for (const auto& entry : fs::directory_iterator(inputFolder_oriImg)) {
        if (cv::imread(entry.path().string()).empty()) {
            cerr << "Could not open or find the image: " << cv::imread(entry.path().string()) << endl;
          return -1;
        }
        
        if (entry.is_regular_file()) {
            board_detectAndPose(cv::imread(entry.path().string()), intrinsicMatrix, distCoeffs, dictionary, detectorParams, markerLength, board, rvec, tvec);
            cv::Mat rvec_m = cv::Mat(rvec); // rvec and tvec are model_to_camera
            //std::cout << rvec_m << endl;
            cv::Mat tvec_m = cv::Mat(tvec);
            //std::cout << tvec_m << endl;
            // Compute the projection matrix
            cv::Mat projectionMatrix = computeProjectionMatrix(intrinsicMatrix, rvec_m, tvec_m);
            projectionMatrices.push_back(cv::Mat(projectionMatrix));
            std::cout << "projection matrix " << j++ << " loaded" << endl;
        }
    }
    std::cout << "Load projection matrices finish." << endl;
    for (int i = 0; i < projectionMatrices.size(); i++) {
        std::cout << "projection matrix " << i << ": ";
        std::cout << (projectionMatrices.at(i)) << endl;
    }
    */
    /* 4. get board pose, retrieve camera extrinsic and get the final projection matrices */



    /* 5. retrieve foreground masks */
    /*
    std::string image_directory = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/SegmentedImages_Ferrari_blackwhite"; // Adjust your image directory here
    std::cout << "Loading segmented images from: " << image_directory << std::endl;
    std::vector<cv::Mat> foregroundMasks;

    for (const auto& entry : fs::directory_iterator(image_directory)) {
        if (entry.is_regular_file()) {
            cv::Mat img = cv::imread(entry.path().string());
            std::cout << "Loaded image: " << fs::path(entry.path().string()) << std::endl;
            foregroundMasks.push_back(img);
        }
    }
    */
    /* 5. retrieve foreground masks */



    /* 6. voxel carving and output .obj file and execution time measurement */
    /*
    int gridX = 50, gridY = 50, gridZ = 50;
    std::vector<std::vector<std::vector<bool>>> voxelGrid(gridX, std::vector<std::vector<bool>>(gridY, std::vector<bool>(gridZ, true)));

    // Get the start time
    auto start = std::chrono::high_resolution_clock::now();

    voxelCarving(voxelGrid, projectionMatrices, foregroundMasks);

    // Get the end time
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate the duration
    std::chrono::duration<double> duration = end - start;

    // Print the duration
    std::cout << "The function took " << duration.count() << " seconds to execute." << std::endl;
    std::cout << voxelGrid.size() << endl;

    // Extract the surface voxels
    std::vector<cv::Point3f> surfaceVoxels = extractSurfaceVoxels(voxelGrid);

    // Save the surface voxels to an OBJ file
    std::string output_obj_file = "carved_object_ferrari.obj";
    saveToOBJ(output_obj_file, surfaceVoxels);
    std::cout << "Voxel carving completed and saved to " << output_obj_file << std::endl;
    */
    /* 6. voxel carving and output .obj file and execution time measurement */

    /********************************* For our data, uncomment this section **********************************************/


    /*-----------------------------------------------------------------------------------------------------------------------------------*/


    /********************************* For NeRF data, uncomment this section **********************************************/
    
    /* 1. intrinsic and extrinsic of NeRF are provided in dataset, so directly hard code and read from file then get projection matrices */
    float scale = 1;// 0.125;
    cv::Mat intrinsicMatrix = (cv::Mat_<double>(3, 3) <<
       1.11111103e+03 * scale, 0., 4.00000000e+02 * scale,
        0., 1.11111103e+03 * scale, 4.00000000e+02 * scale,
        0., 0., 1.);

    std::string filename = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/NeRF_chair_extrinsics.csv";
    std::vector<std::vector<double>> data = readCSV(filename);
    std::vector<cv::Mat> extrinsicMatrices = convertToMatrices(data);

    std::vector<cv::Mat> exMatrices;
    for (int i = 0; i < extrinsicMatrices.size(); i++) {
        std::cout << "extrinsic matrix " << i << ": ";
        std::cout << (extrinsicMatrices.at(i)) << endl;
    }

    std::vector<cv::Mat> projectionMatrices;
    for (int i = 0; i < extrinsicMatrices.size(); i++) {
        cv::Mat projectionMatrix = intrinsicMatrix * extrinsicMatrices.at(i);
        cv::Mat projectionMatrix4x4 = cv::Mat::eye(4, 4, CV_64F); // Start with an identity matrix
        projectionMatrix.copyTo(projectionMatrix4x4(cv::Rect(0, 0, 4, 3)));
        projectionMatrices.push_back(projectionMatrix4x4);
        std::cout << "projection matrix " << i << ": ";
        std::cout << (projectionMatrices.at(i)) << endl;
    }
    /* 1. intrinsic and extrinsic of NeRF are provided in dataset, so directly hard code and read from file then get projection matrices */



    /* 2. retrieve foreground masks */
    std::string image_directory = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/NeRF_chair_test";
    std::cout << "Loading segmented images from: " << image_directory << std::endl;
    std::vector<cv::Mat> foregroundMasks;

    std::regex image_pattern(R"(r_\d+\.png)");

    // Vector to store filenames and their extracted numbers
    std::vector<std::pair<int, std::string>> numbered_filenames;

    // Iterate over files in the directory and collect matching filenames
    for (const auto& entry : fs::directory_iterator(image_directory)) {
        if (entry.is_regular_file()) {
            std::string filename = entry.path().filename().string();
            if (std::regex_match(filename, image_pattern)) {
                int number = extractNumber(filename);
                numbered_filenames.emplace_back(number, entry.path().string());
            }
        }
    }

    // Sort filenames based on the extracted numbers
    std::sort(numbered_filenames.begin(), numbered_filenames.end(),
        [](const std::pair<int, std::string>& a, const std::pair<int, std::string>& b) {
            return a.first < b.first;
        });

    // Load images based on sorted filenames
    for (const auto& numbered_filename : numbered_filenames) {
        cv::Mat img = cv::imread(numbered_filename.second);
        foregroundMasks.push_back(img);
        std::cout << "Loaded image: " << fs::path(numbered_filename.second).filename().string() << std::endl;
    }

    // Verify that the images are loaded correctly
    std::cout << "Total images loaded: " << foregroundMasks.size() << std::endl;
    /* 2. retrieve foreground masks */



    /* 3. voxel carving and output .obj file and execution time measurement */  
    int gridX = 50, gridY = 50, gridZ = 50;
    std::vector<std::vector<std::vector<bool>>> voxelGrid(gridX, std::vector<std::vector<bool>>(gridY, std::vector<bool>(gridZ, true)));

    // Get the start time
    auto start = std::chrono::high_resolution_clock::now();

    voxelCarving(voxelGrid, projectionMatrices, foregroundMasks);

    // Get the end time
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate the duration
    std::chrono::duration<double> duration = end - start;

    // Print the duration
    std::cout << "The function took " << duration.count() << " seconds to execute." << std::endl;
    std::cout << voxelGrid.size() << endl;

    // Extract the surface voxels
    std::vector<cv::Point3f> surfaceVoxels = extractSurfaceVoxels(voxelGrid);

    // Save the surface voxels to an OBJ file
    std::string output_obj_file = "carved_object_chair.obj";
    saveToOBJ(output_obj_file, surfaceVoxels);
    std::cout << "Voxel carving completed and saved to " << output_obj_file << std::endl;

    /* 3. voxel carving and output .obj file and execution time measurement */


    /********************************* For NeRF data, uncomment this section **********************************************/

    return 0;
}


