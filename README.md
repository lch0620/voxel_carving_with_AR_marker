# voxel_carving_with_AR_marker
![ezgif com-crop](https://github.com/user-attachments/assets/50a687f6-18b2-4b01-b19e-2d7957a73438)

This repository contains the code for a 3D reconstruction project using techniques like camera calibration, pose estimation, foreground segmentation, and voxel carving.
## Repository Link

The code for this project is available on GitHub: [voxel_carving_with_AR_marker](https://github.com/lch0620/voxel_carving_with_AR_marker)

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Description of Files](#description-of-files)
- [References](#references)
- [Important Notes](#important-notes)

## Overview

The project performs 3D reconstruction of an object using a series of images taken from different angles. The process includes the following steps:
1. **Camera Calibration**: Uses checkerboard images to retrieve the camera intrinsic and distortion coefficients.
2. **Pose Estimation**: Uses images with ArUco markers to estimate the camera extrinsics.
3. **Foreground Segmentation**: Segments the foreground (object and markers) from the images.
4. **Voxel Carving**: Uses the intrinsic, extrinsic parameters, and foreground masks to perform voxel carving and reconstruct the 3D model of the object.

``` 
voxel_carving_with_AR_marker/
└── Data/
│   ├── charucoboard/
│   ├── checkerboard/
│   ├── Ferrari/
│   ├── Owl/
│   └── NeRF_chair_test/
├── calibration_charucoboard.hpp
├── calibration_normal_checkerboard.hpp
├── foreground_segmentation.hpp
├── pose_estimation.hpp
├── voxel_carving.hpp
├── main.hpp
└── README.md  
```


## Dependencies

- OpenCV (version 4.10.0 or higher)

## Installation

1. **Clone the repository**:
    ```bash
    git clone https://github.com/your_username/voxel_carving_with_AR_marker.git
    cd voxel_carving_with_AR_marker
    ```

2. **Install OpenCV**:
    Follow the instructions to install OpenCV from the [official documentation](https://docs.opencv.org/master/df/d65/tutorial_table_of_content_introduction.html).

3. **Set OpenCV Library Path**:
    Ensure that OpenCV is properly installed and its library path is set in your environment variables.

## Usage

1. **Prepare your images**:
    - Place the checkerboard images in the `Data/checkerboard` directory.
    - Create a directory `Data/your_object` and place the images with object and ArUco markers in it.

2. **Compile the project**:
    ```bash
    g++ -o voxel_carving_with_AR_marker main.cpp `pkg-config --cflags --libs opencv4`
    ```

3. **Run the executable**:
    ```bash
    ./voxel_carving_with_AR_marker
    ```

## Description of Files

- **calibration.hpp**:
    - Contains functions for performing camera calibration using checkerboard images.
    - Input: Checkerboard images
    - Output: Camera intrinsic and distortion coefficients

- **segmentation.hpp**:
    - Contains functions for performing foreground segmentation on images with ArUco markers.
    - Input: Images with ArUco markers
    - Output: Foreground masks

- **pose_estimation.hpp**:
    - Contains functions for estimating camera pose using images with ArUco markers.
    - Input: Images with ArUco markers, camera intrinsic, and distortion coefficients
    - Output: Camera extrinsic parameters

- **voxel_carving.hpp**:
    - Contains functions for performing voxel carving using intrinsic, extrinsic parameters, and foreground masks.
    - Input: Camera intrinsic and extrinsic parameters, foreground masks
    - Output: 3D voxel model of the object

- **main.cpp**:
    - Entry point of the project. It orchestrates the calibration, pose estimation, segmentation, and voxel carving processes.
    - The `main.cpp` file includes sections for two different datasets. To run the code for a specific dataset, uncomment the corresponding section. Detailed instructions and comments are provided inside the code.

## References
Some parts of this project reference the following OpenCV tutorials:

1. [Camera Calibration](https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html)
2. [Pose Estimation using ArUco Markers](https://docs.opencv.org/4.x/db/da9/tutorial_aruco_board_detection.html)

## Authors:
- **Ling-Hsuan Hsu**
- **Matthew Chi Hug Lau**

For any questions or issues, please open an issue in the repository or contact the maintainer.

---

This README file provides a comprehensive guide to understanding and using the 3D reconstruction project. Ensure that the instructions are followed correctly to achieve the desired results.

