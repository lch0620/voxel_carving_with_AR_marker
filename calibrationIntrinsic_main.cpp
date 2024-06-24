#include "calibration.hpp"

int main() {
    string imageFolder = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/checkerboard";
    int squaresX = 8; // Number of squares in X direction
    int squaresY = 11; // Number of squares in Y direction
    float squareLength = 0.015f; // Square side length (in meters)
    float markerLength = 0.011f; // Marker side length (in meters)
    string outputFile = "output1511.yml"; // Output file with calibrated camera parameters
    bool showChessboardCorners = false; // Show detected chessboard corners after calibration

    Calibration calibration(imageFolder, squaresX, squaresY, squareLength, markerLength, outputFile, showChessboardCorners);
    calibration.runCalibration();

    return 0;
}