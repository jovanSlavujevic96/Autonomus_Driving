#ifndef BACHELOR_FRAME_H_
#define BACHELOR_FRAME_H_

#include <opencv2/opencv.hpp>

struct Frame
{
    cv::Mat *MatFrame;
    std::vector<std::vector<cv::Point>> Dots;
};

#endif//BACHELOR_FRAME_H_