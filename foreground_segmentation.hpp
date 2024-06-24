#ifndef FOREGROUND_SEGMENTATION_HPP
#define FOREGROUND_SEGMENTATION_HPP

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <iostream>
#include <opencv2\core.hpp>
#include <opencv2\highgui.hpp>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

class ImageSegmentation {
public:
    ImageSegmentation(const string& folder, const Scalar& minColor, const Scalar& maxColor, int targetWidth, int targetHeight)
        : imageFolder(folder), COLOR_MIN(minColor), COLOR_MAX(maxColor), targetWidth(targetWidth), targetHeight(targetHeight) {}

    void performSegmentation() {
        for (const auto& entry : fs::directory_iterator(imageFolder)) {
            if (entry.is_regular_file()) {
                processImage(entry.path().string());
            }
        }
    }

private:
    string imageFolder;
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

        erode(final_thresh, final_thresh, Mat(), Point(-1, -1), 5);
        bitwise_and(src, src, rr_thresh, final_thresh);

        Mat rrCopy;
        rr_thresh.copyTo(rrCopy);
        Mat rr_thresh2;
        resize(rrCopy, rr_thresh2, Size(targetWidth, targetHeight));

        imshow("Segmented Image", rr_thresh2);
        imwrite("Segmented_Image.jpg", rr_thresh2);

        waitKey(0);
    }
};

#endif // FOREGROUND_SEGMENTATION_HPP
