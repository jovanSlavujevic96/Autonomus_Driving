#ifndef BACHELOR_IMAGEPROCESSOR_STOPPROCESSOR_HPP
#define BACHELOR_IMAGEPROCESSOR_STOPPROCESSOR_HPP

#include "IImageProcessor.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/text/ocr.hpp>

class StopProcessor : public IImageProcessor
{
private:
    cv::Mat m_Frame;
    cv::CascadeClassifier m_StopClassifier;
    cv::Ptr<cv::text::OCRTesseract> m_OCR;
    bool m_StopDetected;     /*stop detected on frame or not*/
    std::vector<std::vector<int>> m_Coordinates;

    void loadCascade(cv::CascadeClassifier* cascade, const int size, const std::string* path);
    int resize(cv::Mat& image, const int limit);
    void crop(cv::Mat& image);
    void redColorSegmentation(const cv::Mat& sample, cv::Mat& result);
    std::vector<cv::Rect> getRedContours(const cv::Mat1b& hueImage) const;
    std::vector<cv::Rect> getDetectedStopContours(const cv::Mat& image, std::vector<cv::Rect>& contours);
    std::vector<cv::Mat> getTextImagesForOCR(const int numOfResizing, std::vector<cv::Rect>& contours);
    std::vector<bool> getDetectionPerRectFromOCR(const std::vector<cv::Mat>& images);
    void setDetection(const std::vector<bool>& detection, const std::vector<cv::Rect>& contours);
    void setCoordinates(const std::vector<bool>& detection, const std::vector<cv::Rect>& contours);
    
   	void drawLocations(cv::Mat& image, const std::vector<bool>& detection, const std::vector<cv::Rect>& contours,
        const cv::Scalar& colorEdge, const cv::Scalar& colorText, const std::string& text);

public:
    StopProcessor(); 
    virtual ~StopProcessor() = default;

    void setFrame(const sensor_msgs::Image& frame) override;
    sensor_msgs::Image getProcessedFrame(void) const override;
    bool getDetection(void) const override;
    std::string getResult(void) const override;
    std::vector<std::vector<int>> getCoordinates(void) const override;
};

#endif //BACHELOR_OBJECTDETECTORNODE_STOPSIGNPROCESSOR_HPP