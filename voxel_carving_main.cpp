#include "foreground_segmentation.hpp"
#include "calibration.hpp"
#include "pose_estimation.hpp"
#include <fstream>

// Function to convert cv::Mat to cv::Vec3d
cv::Vec3d matToVec3d(const cv::Mat& mat) {
    CV_Assert(mat.rows == 3 && mat.cols == 1 && mat.type() == CV_64F); // Ensure the mat is 3x1 and of type double
    return cv::Vec3d(mat.at<double>(0, 0), mat.at<double>(1, 0), mat.at<double>(2, 0));
}

cv::Mat computeProjectionMatrix(const cv::Mat& intrinsicMatrix, const cv::Mat& rvec, const cv::Mat& tvec) {
    // Convert rotation vector to rotation matrix
    cv::Mat rotationMatrix;
    cv::Rodrigues(rvec, rotationMatrix); //3x3

    // Invert the rotation matrix (transpose it)
    //cv::Mat R_inv = rotationMatrix.t();

    // Invert the translation vector
    //cv::Mat t_inv = -R_inv * tvec;

    // Create the 3x4 extrinsic matrix
    cv::Mat extrinsicMatrix = cv::Mat::zeros(3, 4, CV_64F);
    //R_inv.copyTo(extrinsicMatrix(cv::Rect(0, 0, 3, 3)));
    //t_inv.copyTo(extrinsicMatrix(cv::Rect(3, 0, 1, 3)));
    rotationMatrix.copyTo(extrinsicMatrix(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(extrinsicMatrix(cv::Rect(3, 0, 1, 3)));
    //std::cout << extrinsicMatrix << endl;

    // Compute the projection matrix by multiplying the intrinsic matrix with the extrinsic matrix
    cv::Mat projectionMatrix = intrinsicMatrix * extrinsicMatrix;

    // Convert the 3x4 projection matrix to a 4x4 matrix
    cv::Mat projectionMatrix4x4 = cv::Mat::eye(4, 4, CV_64F); // Start with an identity matrix
    projectionMatrix.copyTo(projectionMatrix4x4(cv::Rect(0, 0, 4, 3)));

    return projectionMatrix4x4;
}

// Function to project voxel to image coordinates
cv::Point2f projectVoxelToImage(const cv::Point3f& voxelCenter, const cv::Mat& projectionMatrix) {
    cv::Mat voxelHomogeneous = (cv::Mat_<double>(4, 1) << voxelCenter.x, voxelCenter.y, voxelCenter.z, 1);
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

    for (int x = 0; x < gridX; ++x) {
        for (int y = 0; y < gridY; ++y) {
            for (int z = 0; z < gridZ; ++z) {
                cv::Point3f voxelCenter(x, y, z);
                bool visibleInAllViews = true;

                for (size_t camIdx = 0; camIdx < cameraMatrices.size(); ++camIdx) {
                    int count = 0;
                    cv::Point2f imagePoint = projectVoxelToImage(voxelCenter, cameraMatrices[camIdx]);
                    int u = static_cast<int>(imagePoint.x);
                    int v = static_cast<int>(imagePoint.y);
                                     
                    uchar maskValue = foregroundMasks[camIdx].at<uchar>(v, u);
                    /*
                    //std::cout << "Voxel: (" << x << ", " << y << ", " << z << ") "
                    //    << "Projected to: (" << u << ", " << v << ")" << std::endl;
                    //if (u >= 0 && v >= 0 && u < foregroundMasks[camIdx].cols && v < foregroundMasks[camIdx].rows) {
                    //    uchar maskValue = foregroundMasks[camIdx].at<uchar>(v, u);
                    //    std::cout << "Mask value at (" << u << ", " << v << "): " << static_cast<int>(maskValue) << std::endl;
                    //}
                    */
                    if (u < 0 || v < 0 || u >= foregroundMasks[camIdx].cols || v >= foregroundMasks[camIdx].rows ||  // cols=4032, rows=3024
                        foregroundMasks[camIdx].at<uchar>(v, u) == 0) {
                        visibleInAllViews = false;
                        //if (maskValue != 0) {
                        //    visibleCount++;
                        //}
                        break;
                    }

                    // Debugging information
                    
                    //if ((u < 4032 && u > 0) && (v < 3024 && v > 0) && maskValue != 0) {
                        //std::cout << "Voxel: (" << x << ", " << y << ", " << z << ") "
                        //    << "Projected to: (" << u << ", " << v << ")" << std::endl;
                        //std::cout << "Mask value at (" << u << ", " << v << "): " << static_cast<int>(maskValue) << std::endl;
                        //visibleInAllViews = true;
                        //break;
                        //count++;
                        //std::cout << count << endl; always 1
                    //}
                    //if (count == 18) {
                    //    std::cout << "visible in all views!!!!!!!!!!" << endl;
                    //}
                }
                //if (visibleCount < minVisibleViews) {
                //    voxelGrid[x][y][z] = false;
                //}

                //if (visibleInAllViews == true) {
                //    voxelGrid[x][y][z] = true;
                    //std::cout << x << "         " << y << "         " << z << endl;
                //}
                if (!visibleInAllViews) {
                    voxelGrid[x][y][z] = false;
                }
            }
        }
    }
}



// Function to extract the surface voxels
std::vector<cv::Point3f> extractSurfaceVoxels(const std::vector<std::vector<std::vector<bool>>>& voxelGrid) {
    std::vector<cv::Point3f> surfaceVoxels;
    int gridX = voxelGrid.size();
    int gridY = voxelGrid[0].size();
    int gridZ = voxelGrid[0][0].size();
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
    string inputFolder_oriImg = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/Ferrari";
    float markerLength = 0.02f;
    float markerSeparation = 0.018f;

    float scale = 1;// 0.125;
    cv::Mat intrinsicMatrix = (cv::Mat_<double>(3, 3) << 
        15143.583436112649 * scale, 0., 2015.9447652911815 * scale,
        0., 6563.2025977861031 * scale, 1511.9929369688591 * scale,
        0., 0., 1.);

    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 
        -0.13799205234917988, 
        -19.125534852072558, 
        0.0047282771949379793, 
        -0.0003367841102912744,
        -1256.5526777489752);

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
    std::vector<cv::Mat> foregroundMasks; // List of foreground segmentation masks
    
    // Load foreground masks 
    string inputFolder_segImg = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/SegmentedImages_original_size";
    for (const auto& entry : fs::directory_iterator(inputFolder_segImg)) {
        if (entry.is_regular_file()) {
            foregroundMasks.push_back(cv::imread(entry.path().string()));
        }
    }
    std::cout << "Load segmentation image finish." << endl;
    

    /********************** voxel carving ************************/
    // Define the volume bounds and resolution
    int gridX = 400, gridY = 400, gridZ = 200;
    std::vector<std::vector<std::vector<bool>>> voxelGrid(gridX, std::vector<std::vector<bool>>(gridY, std::vector<bool>(gridZ, true)));

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


