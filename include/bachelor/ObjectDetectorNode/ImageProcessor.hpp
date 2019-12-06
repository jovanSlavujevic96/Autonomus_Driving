#ifndef BACHELOR_OBJECTDETECTOR_NODE_IMAGEPROCESSOR_HPP
#define BACHELOR_OBJECTDETECTOR_NODE_IMAGEPROCESSOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/text/ocr.hpp>
#include <image_transport/image_transport.h>

class ImageProcessor 
{
private:
    unsigned int m_NumOfResizing;
    cv::Mat m_OrigSizeFrame;
    cv::Mat m_ProcFrame;
    cv::Mat m_RedHueFrame;
    cv::CascadeClassifier m_StopClassifier;
    std::vector<cv::Rect> m_StopSignContours;
    std::vector<bool> m_OCRdetection;
    bool m_StopDetected;
    cv::Ptr<cv::text::OCRTesseract> m_OCR;

    void resize(const unsigned int limit);
    void setRedHueFrame(const cv::Mat &sample, cv::Mat &result);
    std::vector<cv::Rect> getRedHueContours(void) const;
    void setStopSignContours(void);
    std::vector<cv::Mat> getPreprocessedImagesForOCR(void);
    void setOCRdetection(void);
   	void drawLocations(cv::Mat &img, const cv::Scalar color, const std::string text);

public:
    ImageProcessor(); 
    ~ImageProcessor();

    void setFrame(sensor_msgs::Image &rawFrame);
    sensor_msgs::Image getProcessedFrame(void) const;
    bool getDetection(void) const;
};

#endif //BACHELOR_OBJECTDETECTOR_NODE_IMAGEPROCESSOR_HPP
