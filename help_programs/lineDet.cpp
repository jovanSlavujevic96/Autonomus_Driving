#include <opencv2/opencv.hpp>

#define image_path "/home/rtrk/Desktop/linije/1.jpg"

cv::Mat1b colorSegmentation(const cv::Mat& image);
cv::Mat1b edges(const cv::Mat& image);

int main(int argc, char **argv)
{
    auto image = cv::imread(image_path);
    cv::imshow("image", image);
    auto segmentation = colorSegmentation(image);
    cv::imshow("segmentation", segmentation);
    auto edge = edges(image);
    cv::imshow("edge", edge);
    
    auto vers = segmentation & edge;
    cv::imshow("vers", vers);

    cv::waitKey();
    return 0;
}

cv::Mat1b colorSegmentation(const cv::Mat& image)
{
    cv::Mat mask;

    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat whiteRange1,whiteRange2;
    cv::inRange(hsv, cv::Scalar(105,5,90), cv::Scalar(125,25,130), whiteRange1);
    cv::inRange(hsv, cv::Scalar(65,-5,70), cv::Scalar(85,15,105), whiteRange2);
    cv::addWeighted(whiteRange1, 1.0f, whiteRange2, 1.0f, 0.0f, mask);
    return mask;
}

cv::Mat1b edges(const cv::Mat& image)
{
    cv::Mat gray, blur, canny;
    cv::cvtColor(image, gray, CV_RGB2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5,5),0);
    cv::Canny(blur, canny, 50, 150);
    return canny;
}