#ifndef FUNCTIONS_READ_EXTRINSIC_FROM_TXT_HPP
#define FUNCTIONS_READ_EXTRINSIC_FROM_TXT_HPP

#include <fstream>
#include <iostream>

std::vector<std::vector<double>> readCSV(const std::string& filename) {
    std::vector<std::vector<double>> data;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string value;
        while (std::getline(ss, value, ',')) {
            row.push_back(std::stod(value));
        }
        data.push_back(row);
    }

    return data;
}

std::vector<cv::Mat> convertToMatrices(const std::vector<std::vector<double>>& data) {
    std::vector<cv::Mat> matrices;
    for (const auto& row : data) {
        cv::Mat matrix(3, 4, CV_64F);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                matrix.at<double>(i, j) = row[i * 4 + j];
            }
        }
        matrices.push_back(matrix);
    }
    return matrices;
}

#endif