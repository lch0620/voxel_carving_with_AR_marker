#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

// Struct for representing a voxel
struct Voxel {
    bool occupied;
    cv::Point3f position;
};

// Function to initialize the voxel grid
std::vector<Voxel> initializeVoxelGrid(int grid_size_x, int grid_size_y, int grid_size_z, float voxel_size, const cv::Point3f& min_corner) {
    std::vector<Voxel> voxelGrid;
    for (int x = 0; x < grid_size_x; ++x) {
        for (int y = 0; y < grid_size_y; ++y) {
            for (int z = 0; z < grid_size_z; ++z) {
                Voxel voxel;
                voxel.occupied = true;
                voxel.position = min_corner + cv::Point3f(x * voxel_size, y * voxel_size, z * voxel_size);
                voxelGrid.push_back(voxel);
            }
        }
    }
    return voxelGrid;
}

// Function to project a voxel into the image space using intrinsic and extrinsic matrices
cv::Point2f projectVoxel(const cv::Mat& intrinsic_matrix, const cv::Mat& distortion_coefficients, const cv::Mat& extrinsic_matrix, const cv::Point3f& voxel_position) {
    std::vector<cv::Point3f> object_points = { voxel_position };
    std::vector<cv::Point2f> image_points;

    cv::projectPoints(object_points, extrinsic_matrix(cv::Rect(0, 0, 3, 3)), extrinsic_matrix(cv::Rect(3, 0, 1, 3)),
        intrinsic_matrix, distortion_coefficients, image_points);

    return image_points[0];
}

// Function to carve the voxels based on the segmented images
void carveVoxels(std::vector<Voxel>& voxelGrid, const std::vector<cv::Mat>& images, const cv::Mat& intrinsic_matrix, const cv::Mat& distortion_coefficients, const std::vector<cv::Mat>& extrinsic_matrices) {
    std::cout << "Performing voxel carving..." << std::endl;
    int numImages = images.size();
    int numVoxels = voxelGrid.size();

    for (int i = 0; i < numImages; ++i) {
        std::cout << "Processing image " << i + 1 << " / " << numImages << std::endl;

        for (auto& voxel : voxelGrid) {
            if (!voxel.occupied) continue;

            cv::Point2f projected = projectVoxel(intrinsic_matrix, distortion_coefficients, extrinsic_matrices[i], voxel.position);
            int x = static_cast<int>(projected.x);
            int y = static_cast<int>(projected.y);

            if (x < 0 || x >= images[i].cols || y < 0 || y >= images[i].rows || images[i].at<uchar>(y, x) == 0) {
                voxel.occupied = false;
            }
        }
    }

    std::cout << "Voxel carving completed." << std::endl;
}

// Function to extract the surface voxels
std::vector<cv::Point3f> extractSurfaceVoxels(const std::vector<Voxel>& voxelGrid) {
    std::vector<cv::Point3f> surfaceVoxels;
    for (const auto& voxel : voxelGrid) {
        if (voxel.occupied) {
            surfaceVoxels.push_back(voxel.position);
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

// Function to load segmented images and convert to grayscale based on yellow color threshold
std::vector<cv::Mat> loadImages(const std::string& directory) {
    std::vector<cv::Mat> images;
    for (const auto& entry : fs::directory_iterator(directory)) {
        if (entry.path().extension() == ".png" || entry.path().extension() == ".jpg") {
            cv::Mat img = cv::imread(entry.path().string());
            if (!img.empty()) {
                // Convert image to HSV color space
                cv::Mat hsv;
                cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

                // Define range for yellow color and apply mask
                cv::Mat mask;
                cv::inRange(hsv, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), mask);

                // Display the mask
                //cv::imshow("Segmented Image", mask);
                cv::waitKey(0);  // Wait for a key press to proceed

                images.push_back(mask);
            }
            else {
                std::cerr << "Failed to load image: " << entry.path().string() << std::endl;
            }
        }
    }
    return images;
}


std::vector<cv::Mat> loadExtrinsicMatrices(const std::string& filename) {
    std::vector<cv::Mat> extrinsic_matrices;
    std::ifstream file(filename);
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line, '[')) {
            //if (line.empty()) continue; // Skip empty lines

            cv::Mat extrinsic_matrix = cv::Mat::zeros(3, 4, CV_64F);
            std::getline(file, line, ']');
            std::istringstream ss(line);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 4; ++j) {
                    ss >> extrinsic_matrix.at<double>(i, j);
                    if (ss.peek() == ',' || ss.peek() == ';')
                        ss.ignore();
                }
            }
            std::cout << extrinsic_matrix << std::endl;
            extrinsic_matrices.push_back(extrinsic_matrix);

            // Consume the remaining characters until next '[' or EOF
            if (file.peek() == ']') file.ignore();
        }
        file.close();
    }
    else {
        std::cerr << "Failed to open extrinsic matrix file: " << filename << std::endl;
    }
    return extrinsic_matrices;
}



int main() {
    // Adjusted parameters (example)
    int grid_size_x = 100;
    int grid_size_y = 50;
    int grid_size_z = 50;
    float voxel_size = 0.002f; // 2mm voxel size

    // Bounding box adjusted for the object size and position
    cv::Point3f min_corner(0.035f, 0.00f, -0.01f); // Based on object center and size
    cv::Point3f max_corner(0.085f, 0.02f, 0.01f);  // Based on object center and size

    // Initialize the voxel grid
    std::cout << "Initializing voxel grid..." << std::endl;
    std::vector<Voxel> voxelGrid = initializeVoxelGrid(grid_size_x, grid_size_y, grid_size_z, voxel_size, min_corner);

    // Load segmented images
    std::string image_directory = "C:/Users/holde/Desktop/voxel_carving_test/Data/Segmented"; // Adjust your image directory here
    std::cout << "Loading segmented images from: " << image_directory << std::endl;
    std::vector<cv::Mat> images = loadImages(image_directory);

    // Hardcoded intrinsic matrix from YAML file
    cv::Mat intrinsic_matrix = (cv::Mat_<double>(3, 3) <<
        15143.583436112649, 0., 2015.9447652911815,
        0., 6563.2025977861031, 1511.9929369688591,
        0., 0., 1.);

    // Distortion coefficients from YAML file
    cv::Mat distortion_coefficients = (cv::Mat_<double>(1, 5) <<
        -0.13799205234917988, -19.125534852072558,
        0.0047282771949379793, -0.0003367841102912744,
        -1256.5526777489752);

    // Load extrinsic matrices from files
    std::string extrinsic_file = "C:/Users/holde/Desktop/voxel_carving_test/extrinsic_matrix.txt";
    std::vector<cv::Mat> extrinsic_matrices = loadExtrinsicMatrices(extrinsic_file);

    // Perform voxel carving
    carveVoxels(voxelGrid, images, intrinsic_matrix, distortion_coefficients, extrinsic_matrices);

    // Extract the surface voxels
    std::vector<cv::Point3f> surfaceVoxels = extractSurfaceVoxels(voxelGrid);

    // Save the surface voxels to an OBJ file
    std::string output_obj_file = "carved_object.obj";
    saveToOBJ(output_obj_file, surfaceVoxels);

    std::cout << "Voxel carving completed and saved to " << output_obj_file << std::endl;

    return 0;
}
