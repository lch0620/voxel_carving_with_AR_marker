#ifndef FOREGROUND_SEGMENTATION_HPP
#define FOREGROUND_SEGMENTATION_HPP

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

class ImageSegmentation {
public:
    ImageSegmentation(const string& inputFolder, const string& outputFolder, const Scalar& minColor, const Scalar& maxColor, int targetWidth, int targetHeight)
        : inputFolder(inputFolder), outputFolder(outputFolder), COLOR_MIN(minColor), COLOR_MAX(maxColor), targetWidth(targetWidth), targetHeight(targetHeight) {}

    void performSegmentation() {
        fs::create_directories(outputFolder);  // Create the output directory if it doesn't exist

        for (const auto& entry : fs::directory_iterator(inputFolder)) {
            if (entry.is_regular_file()) {
                processImage(entry.path().string());
            }
        }
    }

private:
    string inputFolder;
    string outputFolder;
    Scalar COLOR_MIN;
    Scalar COLOR_MAX;
    int targetWidth;
    int targetHeight;

    void processImage(const string& imagePath) {
        Mat src = imread(imagePath);
        if (src.empty()) {
            cerr << "Could not open or find the image: " << imagePath << endl;
            return;
        }

        Mat hsv_img, mask, gray_img, initial_thresh, add_res, second_thresh, and_thresh, xor_thresh, result_thresh, final_thresh, rr_thresh;

        cvtColor(src, hsv_img, COLOR_BGR2HSV);
        inRange(hsv_img, COLOR_MIN, COLOR_MAX, mask);
        cvtColor(src, gray_img, COLOR_BGR2GRAY);
        adaptiveThreshold(gray_img, initial_thresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 257, 2);

        add(mask, initial_thresh, add_res);
        erode(add_res, add_res, Mat(), Point(-1, -1), 1);
        dilate(add_res, add_res, Mat(), Point(-1, -1), 5);

        threshold(gray_img, second_thresh, 170, 255, THRESH_BINARY_INV | THRESH_OTSU);
        bitwise_and(add_res, second_thresh, and_thresh);
        bitwise_xor(add_res, second_thresh, xor_thresh);
        bitwise_or(and_thresh, xor_thresh, result_thresh);
        bitwise_and(add_res, result_thresh, final_thresh);

        erode(final_thresh, final_thresh, Mat(), Point(-1, -1), 8);
        bitwise_and(src, src, rr_thresh, final_thresh);

        cv::Mat hsv;
        cv::cvtColor(rr_thresh, hsv, cv::COLOR_BGR2HSV);

        // Define range for yellow color and apply mask
        cv::Mat mask_final;
        cv::inRange(hsv, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), mask_final);

        Mat copy;
        mask_final.copyTo(copy);
        resize(copy, copy, Size(targetWidth, targetHeight));

        string outputFilePath = outputFolder + "/" + fs::path(imagePath).filename().string();
        imwrite(outputFilePath, mask_final);

        imshow("Segmented Image", copy);
        waitKey(0);
    }
};

#endif // FOREGROUND_SEGMENTATION_HPP
