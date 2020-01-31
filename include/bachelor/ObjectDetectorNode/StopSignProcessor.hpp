#ifndef BACHELOR_OBJECTDETECTORNODE_STOPSIGNPROCESSOR_HPP
#define BACHELOR_OBJECTDETECTORNODE_STOPSIGNPROCESSOR_HPP

#include "IImageProcessor.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/text/ocr.hpp>

class StopSignProcessor : public IImageProcessor
{
private:
    cv::Mat m_InputFrame, m_HelpProcFrame, m_RedHueFrame;
    cv::CascadeClassifier m_StopClassifier;
    cv::Ptr<cv::text::OCRTesseract> m_OCR;
    std::vector<cv::Rect> m_StopSignContours;   //via Neural network and Red Color Segmentation
    std::vector<bool> m_OCRdetection;           //it follows m_StopSignContours and check is some contour Stop sign via OCR 
    bool m_StopDetected;                        //stop detected on frame or not
    unsigned int m_NumOfResizing;

    void resize(const unsigned int limit);
    void crop(void);
    void setRedHueFrame(const cv::Mat &sample, cv::Mat &result);
    std::vector<cv::Rect> getRedHueContours(void) const;
    void setStopSignContours(void);
    std::vector<cv::Mat> getPreprocessedImagesForOCR(void);
    void setOCRdetection(void);
   	void drawLocations(cv::Mat &img, const cv::Scalar color, const std::string text);

public:
    StopSignProcessor(); 
    virtual ~StopSignProcessor() = default;

    virtual void setFrame(sensor_msgs::Image &rawFrame) override;
    virtual sensor_msgs::Image getProcessedFrame(void) const override;
    virtual bool getDetection(void) const override;
    virtual std::string getResult(void) const override;
    virtual std::string getProcessorName(void) const override;
};

#endif //BACHELOR_OBJECTDETECTORNODE_STOPSIGNPROCESSOR_HPP
