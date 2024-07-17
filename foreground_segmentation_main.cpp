#include "foreground_segmentation.hpp"

int main(int argc, char** argv) {
    string imageFolder = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/Ferrari";
    string outputFolder = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/SegmentedImages_original_size";
    Scalar COLOR_MIN(15, 45, 45);
    Scalar COLOR_MAX(65, 255, 255);
    int targetWidth = 1200;
    int targetHeight = 1000;

    ImageSegmentation segmenter(imageFolder, , outputFolder, COLOR_MIN, COLOR_MAX, targetWidth, targetHeight);
    segmenter.performSegmentation();

    return 0;
}