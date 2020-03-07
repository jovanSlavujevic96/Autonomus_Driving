#ifndef BACHELOR_IMAGEPROCESSOR_LANEPROCESSOR_HPP
#define BACHELOR_IMAGEPROCESSOR_LANEPROCESSOR_HPP

#include "IImageProcessor.hpp"

#include <bachelor/CameraCalibration.h>

#include <bachelor/Message/CoordMessage.hpp>
#include <bachelor/Message/ImageMessage.hpp>
#include <bachelor/Message/StringMessage.hpp>

class LaneProcessor : 
    public IImageProcessor
{
private:
    CoordMessage m_Coordinates;
    ImageMessage m_Frame;
    StringMessage m_Detection;

    cv::Mat m_FrameMask;
    bool m_LeftFlag, m_RightFlag;
    const CameraCalibration m_CameraCalibration;
    cv::Point m_MeasuredDot;

    void resize(cv::Mat& image, const float resizeFactor);
    cv::Mat deNoise(const cv::Mat& image) const;
    cv::Mat1b edges(const cv::Mat& image) const;
    cv::Mat1b colorSegmentation(const cv::Mat& image) const;
    void createMask(const cv::Mat& image);
    cv::Mat getROI(const cv::Mat& image) const;
    std::vector<cv::Vec4i> houghLines(const cv::Mat& image) const;
    std::vector<std::vector<cv::Vec4i>> lineSeparation(const cv::Mat& image, const std::vector<cv::Vec4i>& lines);
    std::vector<cv::Point> regression(const cv::Mat& image, const std::vector<std::vector<cv::Vec4i>>& lines);
    void setMessages(std::vector<cv::Point>& lanePts, const float resizeFactor);
    
    void plotLane(cv::Mat& image, const std::vector<cv::Point>& lanePts, const float resizeFactor);

public:
    LaneProcessor(const CameraCalibration& camCal);
    virtual ~LaneProcessor() = default;

    void setFrame(const IMessage* frame) override;
    Topic getWatchdogTopic(void) const override;
    Topic getCoordinateTopic(void) const override;
    Topic getECUTopic(void) const override;
    const IMessage* getCoordinateMessage(void) const override;
    const IMessage* getProcFrameMessage(void) const override;
    const IMessage* getDetectionMessage(void) const override;
};

#endif //BACHELOR_IMAGEPROCESSOR_ROADLANEPROCESSOR_HPP