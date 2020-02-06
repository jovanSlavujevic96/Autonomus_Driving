#ifndef BACHELOR_OBJECTDETECTORNODE_STOPSIGNPROCESSOR_HPP
#define BACHELOR_OBJECTDETECTORNODE_STOPSIGNPROCESSOR_HPP

#include "IImageProcessor.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/text/ocr.hpp>

class StopSignProcessor : public IImageProcessor
{
private:
    cv::Mat m_Frame;
    std::unique_ptr<cv::CascadeClassifier> m_StopClassifier;
    cv::Ptr<cv::text::OCRTesseract> m_OCR;
    bool m_StopDetected;     /*stop detected on frame or not*/

    void loadCascade(cv::CascadeClassifier *cascade, const int size, const std::string *path);
    int resize(cv::Mat &image, const int limit);
    void crop(cv::Mat &image);
    void redColorSegmentation(const cv::Mat &sample, cv::Mat &result);
    std::vector<cv::Rect> getRedContours(const cv::Mat1b &hueImage) const;
    std::vector<cv::Rect> getStopSignContours(const cv::Mat &image, std::vector<cv::Rect> &contours);
    std::vector<cv::Mat> getTextImagesForOCR(const int numOfResizing, std::vector<cv::Rect> &contours);
    std::vector<bool> getDetectionFromOCR(const std::vector<cv::Mat> &images);
   	void drawLocations(cv::Mat &img, const std::vector<bool> &detetcion, const std::vector<cv::Rect> &contours,
        const cv::Scalar color, const std::string text);

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