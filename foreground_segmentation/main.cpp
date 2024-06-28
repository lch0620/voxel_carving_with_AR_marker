#include "foreground_segmentation.hpp"

int main(int argc, char** argv) {
    string imageFolder = "C:/Users/holde/Desktop/voxel_carving_test/Data/Ferrari";
    Scalar COLOR_MIN(15, 45, 45);
    Scalar COLOR_MAX(65, 255, 255);
    int targetWidth = 1200;
    int targetHeight = 1000;

    ImageSegmentation segmenter(imageFolder, COLOR_MIN, COLOR_MAX, targetWidth, targetHeight);
    segmenter.performSegmentation();

    return 0;
}