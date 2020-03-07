#ifndef BACHELOR_MESSAGE_IMAGEMESSAGE_HPP_
#define BACHELOR_MESSAGE_IMAGEMESSAGE_HPP_

#include "IMessage.hpp"
#include <opencv2/opencv.hpp>

class ImageMessage : public IMessage
{
public:
    Topic topic;
    cv::Mat image;
};

#endif //BACHELOR_MESSAGE_IMAGEMESSAGE_HPP_