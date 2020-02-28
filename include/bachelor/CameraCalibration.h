#ifndef BACHELOR_CAMERACALIBRATION_H_
#define BACHELOR_CAMERACALIBRATION_H_

#include <opencv2/opencv.hpp>

struct CameraCalibration
{
    const float resizePercentage;
    const cv::Size size;
    const int type;
    const cv::Point pt_BotomLeft, pt_BotomRight, pt_TopLeft, pt_TopRight;
    const cv::Point pt_RefDot;
}; 

extern CameraCalibration CamCalSolution1;
extern CameraCalibration CamCalSolution2;

#endif //BACHELOR_CAMERACALIBRATION_H_