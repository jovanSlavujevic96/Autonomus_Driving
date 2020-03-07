#ifndef BACHELOR_IMAGEPROCESSOR_MOCKIMAGEPROCESSOR_HPP
#define BACHELOR_IMAGEPROCESSOR_MOCKIMAGEPROCESSOR_HPP

#include "IImageProcessor.hpp"
#include <gmock/gmock.h>

class MockImageProcessor : public IImageProcessor
{
    MOCK_METHOD1(setFrame, void(const sensor_msgs::Image& frame));
    MOCK_CONST_METHOD0(getProcessedFrame, sensor_msgs::Image(void));
    MOCK_CONST_METHOD0(getWatchdogTopic, Topic(void));
    MOCK_CONST_METHOD0(getCoordinateTopic, Topic(void));
    MOCK_CONST_METHOD0(getECUTopic, Topic(void));
};

#endif //BACHELOR_IMAGEPROCESSOR_MOCKIMAGEPROCESSOR_HPP
