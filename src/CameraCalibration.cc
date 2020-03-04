#include <bachelor/CameraCalibration.h>

CameraCalibration CamCalSolution1 = 
{
    .resizePercentage = 0.6f,
    .originalSize = cv::Size(1920,1080),
    .size = CamCalSolution1.size,
    .type = CV_8UC1,
    .pt_BotomLeft = cv::Point(290, 555),
    .pt_BotomRight = cv::Point(895, 555),
    /*
    .pt_TopLeft = cv::Point(475,425),
    .pt_TopRight = cv::Point(604,425),
    */
    //*
    .pt_TopLeft = cv::Point(465,445),
    .pt_TopRight = cv::Point(615,445),
    //*/
    .pt_RefDot = cv::Point(539, 445)
};

CameraCalibration CamCalSolution2 = 
{
    .resizePercentage = 0.7f,
    .originalSize = cv::Size(1344,756),
    .size = CamCalSolution2.size,
    .type = CV_8UC1,
    .pt_BotomLeft = cv::Point(395, 445),
    .pt_BotomRight = cv::Point(945, 445),
    .pt_TopLeft = cv::Point(595, 255),
    .pt_TopRight = cv::Point(695, 255),
    .pt_RefDot = cv::Point(637, 255)
};