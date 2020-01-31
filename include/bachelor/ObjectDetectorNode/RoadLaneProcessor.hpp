#ifndef BACHELOR_OBJECTDETECTORNODE_ROADLANEPROCESSOR_HPP
#define BACHELOR_OBJECTDETECTORNODE_ROADLANEPROCESSOR_HPP

#include "IImageProcessor.hpp"
#include <opencv2/opencv.hpp>

class RoadLaneProcessor : public IImageProcessor
{
private:
    cv::Mat m_InputFrame, m_FrameMask;
    bool m_LeftFlag, m_RightFlag;
    cv::Point m_RefDot, m_MeasuredDot;

    cv::Mat deNoise(const cv::Mat &inputImage) const;
    cv::Mat edges(const cv::Mat &inputImage) const;
    void createMask(const cv::Mat &inputImage);
    cv::Mat getROI(const cv::Mat &inputImage) const;
    std::vector<cv::Vec4i> houghLines(const cv::Mat &inputImage) const;
    std::vector<std::vector<cv::Vec4i>> lineSeparation(const std::vector<cv::Vec4i> &lines);
    std::vector<cv::Point> regression(const std::vector<std::vector<cv::Vec4i>> &lines);
    void plotLane(const std::vector<cv::Point> &lanePts);

public:
    RoadLaneProcessor();
    virtual ~RoadLaneProcessor() = default;

    virtual void setFrame(sensor_msgs::Image &rawFrame) override;
    virtual sensor_msgs::Image getProcessedFrame(void) const override;
    virtual bool getDetection(void) const override;
    virtual std::string getResult(void) const override;
    virtual std::string getProcessorName(void) const override;
};

#endif //BACHELOR_OBJECTDETECTORNODE_ROADLANEPROCESSOR_HPP