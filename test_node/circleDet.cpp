#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

/** @function main */
int main(int argc, char** argv)
{
    if(argc <= 1)
    {
        std::cout << "ERROR INPUT PARAMS\n";
        return -1; //error
    }
    Mat src, src_gray;
    /// Read the image
    {
        std::stringstream ss;
        ss << "/home/rtrk/Desktop/" << argv[1];
        src = imread( ss.str(), 1 );
    }
    if( !src.data )
    { 
        std::cout << "ERROR IMAGE CAN'T BE READEN\n";
        return -1; 
    }

    /// Convert it to gray
    cvtColor( src, src_gray, CV_BGR2GRAY );

    /// Reduce the noise so we avoid false circle detection
    GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );

    std::vector<cv::Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );

    /// Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( src, center, radius, Scalar(0,255,0), 3, 8, 0 );
    }

    /// Show your results
    namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
    imshow( "Hough Circle Transform Demo", src );
    cv::imwrite("/home/rtrk/Desktop/final.jpg", src);

    waitKey(0);
    return 0;
}