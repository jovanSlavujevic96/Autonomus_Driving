#ifndef BACHELOR_IMAGEPROCESSOR_LIMITPROCESSOR_HPP
#define BACHELOR_IMAGEPROCESSOR_LIMITPROCESSOR_HPP

#include "IImageProcessor.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/text/ocr.hpp>

#include <bachelor/Message/CoordMessage.hpp>
#include <bachelor/Message/ImageMessage.hpp>
#include <bachelor/Message/StringMessage.hpp>

#define NumOfClassifiers 2

class LimitProcessor : 
    public IImageProcessor
{
private:
    CoordMessage m_Coordinates;
    ImageMessage m_Frame;
    StringMessage m_Detection;
    
    cv::CascadeClassifier m_SpeedClassifier[NumOfClassifiers];
    cv::Mat m_ImageMask;
    cv::Ptr<cv::text::OCRTesseract> m_OCR;
    int m_LimitValue;
    static bool m_DeleteFile;

    cv::Mat saveThenLoad(const cv::Mat& image);
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
    void changeString(std::string& string);
    void getValueFromOCRstring(std::string& string, int& value, std::string& word);
    std::vector<int> getOCR(std::vector<cv::Mat>& images, std::vector<cv::Rect>& contours);
    int getMode(const std::vector<int>& value);
    void setMessages(const std::vector<cv::Rect>& contours);

    void drawLocations(cv::Mat& image, const std::vector<cv::Rect>& contours,
        const cv::Scalar& colorText, const cv::Scalar& colorEdge, const std::string& text);

public:
    LimitProcessor();
    virtual ~LimitProcessor();

    void setFrame(const IMessage* frame) override;    
    Topic getWatchdogTopic(void) const override;
    Topic getCoordinateTopic(void) const override;
    Topic getECUTopic(void) const override;
    const IMessage* getCoordinateMessage(void) const override;
    const IMessage* getProcFrameMessage(void) const override;
    const IMessage* getDetectionMessage(void) const override;
};

#endif //BACHELOR_IMAGEPROCESSOR_SPEEDLIMITPROCESSOR_HPP