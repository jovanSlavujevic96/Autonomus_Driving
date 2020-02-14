#ifndef BACHELOR_IMAGEPROCESSOR_LANEPROCESSOR_HPP
#define BACHELOR_IMAGEPROCESSOR_LANEPROCESSOR_HPP

#include "IImageProcessor.hpp"

#include <opencv2/opencv.hpp>

class LaneProcessor : public IImageProcessor
{
private:
    cv::Mat m_Frame, m_FrameMask;
    bool m_LeftFlag, m_RightFlag;
    cv::Point m_RefDot, m_MeasuredDot;
    std::vector<std::vector<int>> m_Coordinates;

    void resize(cv::Mat &image, const float resizeFactor);
    cv::Mat deNoise(const cv::Mat &image) const;
    cv::Mat edges(const cv::Mat &image) const;
    void createMask(const cv::Mat &image);
    cv::Mat getROI(const cv::Mat &image) const;
    std::vector<cv::Vec4i> houghLines(const cv::Mat &image) const;
    std::vector<std::vector<cv::Vec4i>> lineSeparation(const cv::Mat &image, const std::vector<cv::Vec4i> &lines);
    std::vector<cv::Point> regression(const cv::Mat &image, const std::vector<std::vector<cv::Vec4i>> &lines);
    void CalculateCoordinates(std::vector<cv::Point> &lanePts, const float resizeFactor);
    void plotLane(cv::Mat &image, const std::vector<cv::Point> &lanePts, const float resizeFactor);

public:
    LaneProcessor();
    virtual ~LaneProcessor() = default;

    void setFrame(const sensor_msgs::Image &Frame) override;
    sensor_msgs::Image getProcessedFrame(void) const override;
    bool getDetection(void) const override;
    std::string getResult(void) const override;
    std::vector<std::vector<int>> getCoordinates(void) const override;
};

#endif //BACHELOR_IMAGEPROCESSOR_ROADLANEPROCESSOR_HPP