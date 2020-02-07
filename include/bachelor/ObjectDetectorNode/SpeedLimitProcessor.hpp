#ifndef BACHELOR_OBJECTDETECTORNODE_SPEEDLIMITPROCESSOR_HPP
#define BACHELOR_OBJECTDETECTORNODE_SPEEDLIMITPROCESSOR_HPP

#include "IImageProcessor.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/text/ocr.hpp>

class SpeedLimitProcessor : public IImageProcessor
{
private:
    cv::CascadeClassifier m_SpeedClassifier;
    cv::CascadeClassifier m_LimitRecognizeClassifier[2];
    cv::Mat m_Frame, m_ImageMask;
    bool m_SpeedLimitDetected;
    int m_SpeedValue;

    void loadCascade(cv::CascadeClassifier *cascade, const int size, const std::string *path);
    void resize(cv::Mat &image, const float resizeFactor);
    void createMask(const cv::Mat &image);
    cv::Mat makeROI(const cv::Mat &image) const;
    void redColorSegmentation(const cv::Mat &sample, cv::Mat1b &result);
    std::vector<cv::Rect> getRedContours(const cv::Mat1b &hueImage) const;
    void eraseFromContour(std::vector<cv::Rect> &contours);
    std::vector<cv::Rect> getDetectedSpeedLimitContours(const cv::Mat &image, std::vector<cv::Rect> &contours);
    std::vector<std::string> getRecognizedClassifier(const cv::Mat &image);
    void drawLocations(cv::Mat &image, std::vector<cv::Rect> &contours, const float resizeFactor,
        const cv::Scalar color, const std::string text);

public:
    SpeedLimitProcessor();
    virtual ~SpeedLimitProcessor() = default;

    virtual void setFrame(sensor_msgs::Image &rawFrame) override;
    virtual sensor_msgs::Image getProcessedFrame(void) const override;
    virtual bool getDetection(void) const override;
    virtual std::string getResult(void) const override;
    virtual std::string getProcessorName(void) const override;
};

#endif //BACHELOR_OBJECTDETECTORNODE_SPEEDLIMITPROCESSOR_HPP