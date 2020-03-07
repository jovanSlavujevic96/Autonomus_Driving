#ifndef BACHELOR_MESSAGE_COORDMESSAGE_HPP_
#define BACHELOR_MESSAGE_COORDMESSAGE_HPP_

#include "IMessage.hpp"
#include <opencv2/opencv.hpp>

class CoordMessage : public IMessage
{
public:
    Topic topic;
    std::vector<std::vector<cv::Point>> coordinates;
};

#endif //BACHELOR_MESSAGE_COORDMESSAGE_HPP_