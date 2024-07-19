#include "foreground_segmentation.hpp"
//#include "calibration.hpp"
#include "pose_estimation.hpp"
#include <fstream>

// Function to convert cv::Mat to cv::Vec3d
cv::Vec3d matToVec3d(const cv::Mat& mat) {
    CV_Assert(mat.rows == 3 && mat.cols == 1 && mat.type() == CV_64F); // Ensure the mat is 3x1 and of type double
    return cv::Vec3d(mat.at<double>(0, 0), mat.at<double>(1, 0), mat.at<double>(2, 0));
}

cv::Point3f VoxelToWorld(double x, double y, double z) {
    cv::Point3f point; //actual coordinates in bounding box

    double bbX = 8; //physical length of the bounding box
    double bbY = 8;
    double bbZ = 8;

    double stepX = bbX / 50; // step size in x direction, denominator depends on grid definition
    double stepY = bbY / 50; // step size in y direction
    double stepZ = bbZ / 50; // step size in z direction

    double startX = -bbX / 2; // center the bounding box at origin
    double startY = -bbY / 2; 
    double startZ = -bbZ / 2; 

    point.x = startX + stepX * (x + 0.5);
    point.y = startY + stepY * (y + 0.5);
    point.z = startZ + stepZ * (z + 0.5);
    //std::cout << "Currently grid ID: " << x << " " << y << " " << z << " " << std::endl;
    //std::cout << "Coordinate: " << point << std::endl;
    return point;
}

cv::Mat computeProjectionMatrix(const cv::Mat& intrinsicMatrix, const cv::Mat& rvec, const cv::Mat& tvec) {
    // Convert rotation vector to rotation matrix
    cv::Mat rotationMatrix;
    cv::Rodrigues(rvec, rotationMatrix); //3x3

    // Invert the rotation matrix (transpose it)
   cv::Mat R_inv = rotationMatrix.t();

    // Invert the translation vector
    cv::Mat t_inv = -R_inv * tvec;

    // Create the 3x4 extrinsic matrix
    cv::Mat extrinsicMatrix = cv::Mat::zeros(3, 4, CV_64F);
    rotationMatrix.copyTo(extrinsicMatrix(cv::Rect(0, 0, 3, 3)));
    //t_inv.copyTo(extrinsicMatrix(cv::Rect(3, 0, 1, 3)));
    //t_inv.copyTo(extrinsicMatrix(cv::Rect(3, 0, 1, 3)));
    //rotationMatrix.copyTo(extrinsicMatrix(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(extrinsicMatrix(cv::Rect(3, 0, 1, 3)));

    //cv::Mat Inv_extrinsic;
    //cv::invert(extrinsicMatrix, Inv_extrinsic);

    //std::cout << "R_inv is: " << R_inv << endl;
    //std::cout << "t_inv is: " << t_inv << endl;
    std::cout<< "extrinsicMatrix is: " << extrinsicMatrix << endl;

    // Compute the projection matrix by multiplying the intrinsic matrix with the extrinsic matrix
    cv::Mat projectionMatrix = intrinsicMatrix * extrinsicMatrix;

    // Convert the 3x4 projection matrix to a 4x4 matrix
    cv::Mat projectionMatrix4x4 = cv::Mat::eye(4, 4, CV_64F); // Start with an identity matrix
    projectionMatrix.copyTo(projectionMatrix4x4(cv::Rect(0, 0, 4, 3)));

    cv::Mat Inv_projection;
    //cv::invert(projectionMatrix4x4, Inv_projection);

    std::cout << "projectionMatrix4x4 is: " << projectionMatrix4x4 << endl;
    //std::cout << "Inv_projection is: " << Inv_projection << endl;

    return projectionMatrix4x4;
}

// Function to project voxel to image coordinates
cv::Point2f projectVoxelToImage(const cv::Point3f& voxelCenter, const cv::Mat& projectionMatrix) {

    //cv::Point3f v_ = VoxelToWorld(voxelCenter.x, voxelCenter.y, voxelCenter.z);

    cv::Mat voxelHomogeneous = (cv::Mat_<double>(4, 1) << voxelCenter.x, voxelCenter.y, voxelCenter.z, 1);
    //cv::Mat voxelHomogeneous = (cv::Mat_<double>(4, 1) << v_.x, v_.y, v_.z, 1);
    cv::Mat imagePointHomogeneous = projectionMatrix * voxelHomogeneous;
    return cv::Point2f(imagePointHomogeneous.at<double>(0) / imagePointHomogeneous.at<double>(2),
        imagePointHomogeneous.at<double>(1) / imagePointHomogeneous.at<double>(2));
}

// Function to perform voxel carving
void voxelCarving(std::vector<std::vector<std::vector<bool>>>& voxelGrid,
    const std::vector<cv::Mat>& cameraMatrices,
    const std::vector<cv::Mat>& foregroundMasks) {
    int gridX = voxelGrid.size();
    int gridY = voxelGrid[0].size();
    int gridZ = voxelGrid[0][0].size();

    //debugging
    Mat whiteMatrix(4032, 3024, CV_8UC3, Scalar(255, 255, 255));//Declaring a white matrix

    for (int x = 0; x < gridX; ++x) {
        for (int y = 0; y < gridY; ++y) {
            for (int z = 0; z < gridZ; ++z) {
                cv::Point3f voxelCenter(x, y, z);
                bool visibleInAllViews = false;

                cv::Point3f voxel_coor = VoxelToWorld(voxelCenter.x, voxelCenter.y, voxelCenter.z);

                std::cout << "Currently grid ID: " << voxelCenter.x << " " << voxelCenter.y << " " << voxelCenter.z << " " << std::endl;
                std::cout << "Coordinate: " << voxel_coor << std::endl;

                for (size_t camIdx = 0; camIdx < cameraMatrices.size(); ++camIdx) { //cameraMatrices.size()  
                    int count = 0;
                    cv::Point2f imagePoint = projectVoxelToImage(voxelCenter, cameraMatrices[camIdx]);
                    //cv::imshow(imagePoint);

                    int u = static_cast<int>(imagePoint.x);
                    int v = static_cast<int>(imagePoint.y);
                    int compare = 0;

                    //uchar maskValue = foregroundMasks[camIdx].at<uchar>(v, u);
                    //cv::Vec3b maskValue = foregroundMasks[camIdx].at<Vec3b>(v, u);
                    //std::cout << "maskValue is: " << +maskValue[1] << std::endl;                         //static_cast<int>(maskValue[1])
                    
                    if (u >= 0 && v >= 0 && u < foregroundMasks[camIdx].cols && v < foregroundMasks[camIdx].rows) {
                        uchar maskValue = foregroundMasks[camIdx].at<uchar>(v, u);
                        if (+maskValue ==0) {
                            //visibleInAllViews = true;
                            continue;
                            //break;
                            //std::cout << "Continue" << std::endl;
                        
                        }
                        
                        else {
                            visibleInAllViews = true;
                            //std::cout << "maskValue is: " << +maskValue[0]<<" " << +maskValue[1] << " " << +maskValue[2] << std::endl;
                            //std::cout << "maskValue is: " << +maskValue << std::endl;
                            cv::Point center(u, v);
                            int radius = 15;
                            cv::Scalar line_Color = cv::Scalar(0, 0, 0);
                            int thickness = -2;
                            //cv::circle(whiteMatrix, center, radius, line_Color, thickness);
                        }
                    }
                    /*
                    else if (maskValue != 0) {
                        cv::Point center(u, v);
                        int radius = 15;
                        cv::Scalar line_Color = (255, 0, 0);
                        int thickness = -2;
                        cv::circle(whiteMatrix, center, radius, line_Color, thickness);
                    }*/
                }
                
                if (visibleInAllViews==true) {
                    voxelGrid[x][y][z] = true;
                }
            }
        }
    }
    //cv::resize(whiteMatrix, whiteMatrix, Size(800, 600));
    //cv::imshow("Debugging 2D image", whiteMatrix);
    std::cout << "Line of showing photos." << std::endl;
    //cv::waitKey(0);
}



// Function to extract the surface voxels
std::vector<cv::Point3f> extractSurfaceVoxels(const std::vector<std::vector<std::vector<bool>>>& voxelGrid) {
    std::vector<cv::Point3f> surfaceVoxels;
    int gridX = voxelGrid.size();
    int gridY = voxelGrid[0].size();
    int gridZ = voxelGrid[0][0].size();
    std::cout << "The size of gridX is " << gridX << std::endl;
    std::cout << "The size of gridY is " << gridY << std::endl;
    std::cout << "The size of gridZ is " << gridZ << std::endl;

    for (int x = 0; x < gridX; ++x) {
        for (int y = 0; y < gridY; ++y) {
            for (int z = 0; z < gridZ; ++z) {
                if (voxelGrid[x][y][z]) {
                    surfaceVoxels.push_back(cv::Point3f(x,y,z));
                }
            }
        }
    }
    return surfaceVoxels;
}

// Function to save the surface voxels as an OBJ file
void saveToOBJ(const std::string& filename, const std::vector<cv::Point3f>& surfaceVoxels) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    for (const auto& voxel : surfaceVoxels) {
        file << "v " << voxel.x << " " << voxel.y << " " << voxel.z << std::endl;
    }
    file.close();
    // Print number of voxels saved
    std::cout << "Number of voxels saved to " << filename << ": " << surfaceVoxels.size() << std::endl;
}

int main(int argc, char** argv) {
 
    /********************** calibration to get the intrinsic ************************/
    /*
    string imageFolder_cal = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/checkerboard";
    int squaresX = 8; // Number of squares in X direction
    int squaresY = 11; // Number of squares in Y direction
    float squareLength = 0.015f; // Square side length (in meters)
    float markerLength = 0.011f; // Marker side length (in meters)
    string outputFile = "output1511.yml"; // Output file with calibrated camera parameters
    bool showChessboardCorners = false; // Show detected chessboard corners after calibration

    Calibration calibration(imageFolder_cal, squaresX, squaresY, squareLength, markerLength, outputFile, showChessboardCorners);
    calibration.runCalibration();
    */

    /********************** segment images from folder ************************/
    /*
    string imageFolder_seg = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/Ferrari";
    string outputFolder = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/SegmentedImages_original_size";
    Scalar COLOR_MIN(15, 45, 45);
    Scalar COLOR_MAX(65, 255, 255);
    int targetWidth = 1200;
    int targetHeight = 1000;

    ImageSegmentation segmenter(imageFolder_seg, outputFolder, COLOR_MIN, COLOR_MAX, targetWidth, targetHeight);
    segmenter.performSegmentation();
    */
    
    /********************** get board pose, retrieve camera extrinsic and get the final projection matrices ************************/
    //string imageFile = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/Ferrari/0FDE6606-15FD-4B6D-82B7-D4ECB058F06E.jpeg";
    //string imageFile = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/Owl/B8B14F3E-3C63-4282-A347-75FEBE726EF9.jpeg";
    
    /**/
    string inputFolder_oriImg = "C:/Assignments/3DScanning/Project/environment/Exercise-2/Data/Ferrari";
    //string calibrationFile = "C:/Assignments/3DScanning/Project/environment/Exercise-2/output_chessboard_final.yml";
    float markerLength = 0.02f;
    float markerSeparation = 0.018f;
    //cv::Mat intrinsicMatrix = cv::Mat_<double>(3, 3);
    //cv::Mat distCoeffs = cv::Mat_<double>(5, 1);
    
    float scale = 1;// 0.125;
    cv::Mat intrinsicMatrix = (cv::Mat_<double>(3, 3) << 
        3038.248502423437 * scale, 0., 2048.6800329991179 * scale, //took away scale of optical center
        0., 3049.5886788585231 * scale, 1501.8827722101619 * scale,
        0., 0., 1.);

    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 
        0.17046182312224414,
        -0.69286205708434745,
        -0.0059834074266772496,
        0.0027923172265777603,
        0.9916983456943318);
    
    //loadCameraParams(calibrationFile, intrinsicMatrix, distCoeffs);



    aruco::DetectorParameters detectorParams = aruco::DetectorParameters();
    aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);
    aruco::GridBoard board(Size(5, 7), markerLength, markerSeparation, dictionary);
    cv::Vec3d rvec, tvec;

    // Placeholder for camera matrices (projection matrices)
    std::vector<cv::Mat> cameraMatrices; // List of projection matrices

    // Load camera matrix
    for (const auto& entry : fs::directory_iterator(inputFolder_oriImg)) {
        if (cv::imread(entry.path().string()).empty()) {
            cerr << "Could not open or find the image: " << cv::imread(entry.path().string()) << endl;
          return -1;
        }
        if (entry.is_regular_file()) {
            board_detectAndPose(cv::imread(entry.path().string()), intrinsicMatrix, distCoeffs, dictionary, detectorParams, markerLength, board, rvec, tvec);
            cv::Mat rvec_m = cv::Mat(rvec);
            cv::Mat tvec_m = cv::Mat(tvec);
            // Compute the projection matrix
            cv::Mat projectionMatrix = computeProjectionMatrix(intrinsicMatrix, rvec_m, tvec_m);
            cameraMatrices.push_back(cv::Mat(projectionMatrix));
        }
    }
    std::cout << "Load projection matrices finish." << endl;
    for (int i = 0; i < cameraMatrices.size(); i++) {
        std::cout << "projection matrix " << i << ": ";
        std::cout << (cameraMatrices.at(i)) << endl;
    }

    /********************** retrieve foreground masks ************************/
    // Placeholder for foreground masks
    std::cout << "Entered foreground" << endl;
    std::vector<cv::Mat> foregroundMasks; // List of foreground segmentation masks
    
    // Load foreground masks 
    string inputFolder_segImg = "C:/Assignments/3DScanning/Project/environment/Exercise-2/Data/SegmentedImages_original_size";

    for (const auto& entry : fs::directory_iterator(inputFolder_segImg)) {
        if (entry.is_regular_file()) {
            cv::Mat img = cv::imread(entry.path().string());
            // Convert image to HSV color space
            cv::Mat hsv;
            cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

            // Define range for yellow color and apply mask
            cv::Mat mask;
            cv::inRange(hsv, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), mask);

            foregroundMasks.push_back(mask);
            //std::cout << mask.size() << endl;
            // Display the mask
            Mat mask_copy;
            mask.copyTo(mask_copy);
            cv::Mat mask_copy2;
            //resize(mask_copy, mask_copy2, Size(1200, 1000));
            //cv::imshow("Segmented Image", mask_copy2);
            //cv::waitKey(0);  // Wait for a key press to proceed      
            //string outputFilePath = outputFolder + "/" + fs::path(entry.path().string()).filename().string();
            //imwrite(outputFilePath, mask);
        }
    }
    std::cout << "Load segmentation image finish." << endl;
    

    /********************** voxel carving ************************/
    // Define the volume bounds and resolution
    int gridX = 50, gridY = 50, gridZ = 50; //only the number of grids, these are just the IDs of the grid
    std::vector<std::vector<std::vector<bool>>> voxelGrid(gridX, std::vector<std::vector<bool>>(gridY, std::vector<bool>(gridZ, false)));

    //std::vector<cv::Point3f> surfaceVoxels2 = extractSurfaceVoxels(voxelGrid);
    //std::string output_obj_file2 = "carved_object2.obj";
    //saveToOBJ(output_obj_file2, surfaceVoxels2);
    //std::cout << "Voxel carving completed and saved to " << output_obj_file2 << std::endl;

    // Perform voxel carving
    voxelCarving(voxelGrid, cameraMatrices, foregroundMasks);
    //std::cout << voxelGrid.size() << endl;
    // Extract the surface voxels
    std::vector<cv::Point3f> surfaceVoxels = extractSurfaceVoxels(voxelGrid);

    // Save the surface voxels to an OBJ file
    std::string output_obj_file = "carved_object0628.obj";
    saveToOBJ(output_obj_file, surfaceVoxels);
    std::cout << "Voxel carving completed and saved to " << output_obj_file << std::endl;

    
    return 0;
}


