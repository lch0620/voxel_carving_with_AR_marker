#include <stdio.h>
#include <opencv2/opencv.hpp>

//using namespace cv;
int main(int argc, char** argv )
{

    cv::Mat image;
    image = cv::imread("C:/Users/holde/Desktop/voxel_carving_with_AR_marker/testing.png");
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Display Image", image);
    printf("Hello");
    cv::waitKey(0);
    return 0;
}
