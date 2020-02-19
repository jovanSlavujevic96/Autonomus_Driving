#ifndef BACHELOR_IMAGEPROCESSOR_LIMITPROCESSOR_HPP
#define BACHELOR_IMAGEPROCESSOR_LIMITPROCESSOR_HPP

#include "IImageProcessor.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/text/ocr.hpp>

class LimitProcessor : public IImageProcessor
{
private:
    cv::CascadeClassifier m_SpeedClassifier;
    //cv::CascadeClassifier m_LimitRecognizeClassifier[2];
    cv::Mat m_Frame, m_ImageMask;
    bool m_SpeedLimitDetected;
    int m_LimitValue;
    cv::Ptr<cv::text::OCRTesseract> m_OCR;
    //std::map<int, cv::Scalar> m_ColorMap;
    //const int m_NumOfClassifiers;
    //const int m_PossibleLimitValues[5];

    void loadCascade(cv::CascadeClassifier* cascade, const int size, const std::string* path);
    
    void resize(cv::Mat& imageToResize, const float resizeFactor);
    void resize(std::vector<cv::Rect>& contoursToResize, const float resizeFactor);

    void createROImask(const cv::Mat& frame);
    cv::Mat getROIframe(const cv::Mat& frame) const;
    
    void redColorSegmentationMask(const cv::Mat& sample, cv::Mat1b& result);
    std::vector<cv::Rect> getRedContours(const cv::Mat1b& hueImage) const;

    void increaseRectsForClassification(const cv::Mat& frame, std::vector<cv::Rect>& contours);
    std::vector<cv::Rect> getDetectedLimitContours(const cv::Mat& frame, const std::vector<cv::Rect>& contours);
    
    cv::Mat approximateCircle(cv::Mat& binaryMask);
    std::vector<cv::Mat> getTextImagesForOCR(const cv::Mat& image, const cv::Mat1b& hueImage, std::vector<cv::Rect>& contours);
    std::vector<bool> getOCRdetection(const std::vector<cv::Mat>& images);

    //std::vector<cv::Rect> getSpeedLimitValues(const cv::Mat &image, const std::vector<cv::Rect> &contours);
    void drawLocations(cv::Mat& image, const std::vector<cv::Rect>& contours,
        const cv::Scalar& colorText, const cv::Scalar& colorEdge, const std::string& text);

public:
    LimitProcessor();
    virtual ~LimitProcessor();

    void setFrame(const sensor_msgs::Image& frame) override;
    sensor_msgs::Image getProcessedFrame(void) const override;
    bool getDetection(void) const override;
    std::string getResult(void) const override;
    std::vector<std::vector<int>> getCoordinates(void) const override;
};

#endif //BACHELOR_IMAGEPROCESSOR_SPEEDLIMITPROCESSOR_HPP