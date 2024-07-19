#ifndef OBJ_GENERATION_HPP
#define OBJ_GENERATION_HPP

#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>

// Function to extract the surface voxels
std::vector<cv::Point3f> extractSurfaceVoxels(const std::vector<std::vector<std::vector<bool>>>& voxelGrid) {
    std::vector<cv::Point3f> surfaceVoxels;
    int gridX = voxelGrid.size();
    int gridY = voxelGrid[0].size();
    int gridZ = voxelGrid[0][0].size();
    for (int x = 0; x < gridX; ++x) {
        for (int y = 0; y < gridY; ++y) {
            for (int z = 0; z < gridZ; ++z) {
                if (voxelGrid[x][y][z] == true) {
                    cv::Point3f voxelCenter(x, y, z);
                    cv::Point3f voxel_coor = VoxelToWorld(voxelCenter.x, voxelCenter.y, voxelCenter.z);
                    surfaceVoxels.push_back(voxel_coor);
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

#endif