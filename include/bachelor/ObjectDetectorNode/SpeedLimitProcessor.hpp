#ifndef BACHELOR_OBJECTDETECTORNODE_SPEEDLIMITPROCESSOR_HPP
#define BACHELOR_OBJECTDETECTORNODE_SPEEDLIMITPROCESSOR_HPP

#include "IImageProcessor.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/text/ocr.hpp>

class SpeedLimitProcessor : public IImageProcessor
{
private:
    cv::Mat m_InputFrame, m_HelpProcFrame, m_RedHueFrame;
    cv::CascadeClassifier m_SpeedClassifier;
    cv::Ptr<cv::text::OCRTesseract> m_OCR;
    std::vector<cv::Rect> m_SpeedLimitContours;
    std::vector<std::string> m_Strings;
    bool m_SpeedLimitDetected;
    unsigned int m_LimitValue, m_NumOfResizing;

    void resize(const unsigned int limit);
    void crop(void);
    void setRedHueFrame(const cv::Mat &sample, cv::Mat &result);
    std::vector<cv::Rect> getRedHueContours(void) const;
    std::vector<cv::Rect> getSpeedLimitContours(void);
    void setContoursByOCRcheckAndStrings(void);
    void drawLocations(cv::Mat &img, const cv::Scalar color );

public:
    SpeedLimitProcessor();
    virtual ~SpeedLimitProcessor() = default;

    virtual void setFrame(sensor_msgs::Image &rawFrame) override;
    virtual sensor_msgs::Image getProcessedFrame(void) const override;
    virtual bool getDetection(void) const override;
    virtual int getValue(void) const override;
    virtual std::string getProcessingName(void) const override;
};

#endif //BACHELOR_OBJECTDETECTORNODE_SPEEDLIMITPROCESSOR_HPP