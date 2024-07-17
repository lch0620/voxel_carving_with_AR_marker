# voxel_carving_with_AR_marker

This repository contains the code for a 3D reconstruction project using techniques like camera calibration, pose estimation, foreground segmentation, and voxel carving.

## Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [Description of Files](#description-of-files)
- [Important Notes](#important-notes)

## Overview

The project performs 3D reconstruction of an object using a series of images taken from different angles. The process includes the following steps:
1. **Camera Calibration**: Uses checkerboard images to retrieve the camera intrinsic and distortion coefficients.
2. **Pose Estimation**: Uses images with ArUco markers to estimate the camera extrinsics.
3. **Foreground Segmentation**: Segments the foreground (object and markers) from the images.
4. **Voxel Carving**: Uses the intrinsic, extrinsic parameters, and foreground masks to perform voxel carving and reconstruct the 3D model of the object.

## Project Structure
voxel_carving_with_AR_marker/
|
├── Data/
    ├── charucoboard/
    ├── checkerboard/
    ├── Ferrari/
    ├── Owl/  
├── calibration.hpp
├── segmentation.hpp
├── pose_estimation.hpp
├── voxel_carving.hpp
├── main.cpp
├── README.md


## Dependencies

- OpenCV (version 4.x.x or higher)

## Installation

1. **Clone the repository**:
    ```bash
    git clone https://github.com/your_username/3d_reconstruction.git
    cd 3d_reconstruction
    ```

2. **Install OpenCV**:
    Follow the instructions to install OpenCV from the [official documentation](https://docs.opencv.org/master/df/d65/tutorial_table_of_content_introduction.html).

3. **Set OpenCV Library Path**:
    Ensure that OpenCV is properly installed and its library path is set in your environment variables.

## Usage

1. **Prepare your images**:
    - Place the checkerboard images in the `images/checkerboard` directory.
    - Place the images with ArUco markers in the `images/aruco_markers` directory.

2. **Compile the project**:
    ```bash
    g++ -o 3d_reconstruction main.cpp `pkg-config --cflags --libs opencv4`
    ```

3. **Run the executable**:
    ```bash
    ./3d_reconstruction
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

## Important Notes

- Do not include any heavy files (more than 15 MB) in this repository. Instead, upload them to any cloud storage and leave a link here if necessary.

For any questions or issues, please open an issue in the repository or contact the maintainer.

---

This README file provides a comprehensive guide to understanding and using the 3D reconstruction project. Ensure that the instructions are followed correctly to achieve the desired results.

