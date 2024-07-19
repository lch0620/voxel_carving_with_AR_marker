#include "calibration.hpp"
#include "calibration2.hpp"

int main() {
    /* 
    // calibration by Charucoboard
    string imageFolder = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/checkerboard";
    int squaresX = 8; // Number of squares in X direction
    int squaresY = 11; // Number of squares in Y direction
    float squareLength = 0.015f; // Square side length (in meters)
    float markerLength = 0.011f; // Marker side length (in meters)
    string outputFile = "output1511.yml"; // Output file with calibrated camera parameters
    bool showChessboardCorners = false; // Show detected chessboard corners after calibration

    Calibration calibration(imageFolder, squaresX, squaresY, squareLength, markerLength, outputFile, showChessboardCorners);
    calibration.runCalibration();
    */

    // calibration by normal checkboard
    string imageFolder_cal = "C:/Users/user/OneDrive/桌面/3D scanning/Exercises/voxel_carving_with_AR_marker/Data/normal_checkerboard";
    int squaresX = 6; // Number of squares in X direction
    int squaresY = 9; // Number of squares in Y direction
    float squareLength = 0.0225f; // Square side length (in meters)
    string outputFile = "output_chessboard.yml"; // Output file with calibrated camera parameters
    bool showChessboardCorners = false; // Show detected chessboard corners after calibration

    Calibration2 calibration(imageFolder_cal, squaresX, squaresY, squareLength, outputFile, showChessboardCorners);
    calibration.runCalibration2();

    return 0;
}
