#include <opencv2/opencv.hpp>

cv::Mat createMask(cv::Mat &inputImage);
cv::Mat getImageROI(const cv::Mat &mask, const cv::Mat &inputImage);

int main(int argc, char **argv)
{
    //cv::Mat image = cv::imread("/home/rtrk/Pictures/panda.jpg");
    //cv::imshow("panda", image);
    cv::VideoCapture cap;
    {
        const std::string VideoFilePath = "/home/rtrk/Videos/testVideos/LimitTest2.mp4";
        cap = cv::VideoCapture(VideoFilePath);
        if(!cap.isOpened() )
        {
            std::cout << "There's no video at: " << VideoFilePath << std::endl;
            return -1; 
        }
    }
    cv::Mat frame;
    cap >> frame;
    cv::resize(frame, frame, cv::Size(frame.cols*0.6, frame.rows*0.6));

    cv::Mat inputImage = frame.clone();
    cv::Mat mask = createMask(inputImage);

    const int play = 30, pause = 0; 
    int state = play;
    while(!frame.empty() )
    {
        cv::Mat outputImage = getImageROI(mask,inputImage);

        // display for debugging purpose
        cv::imshow("frame", frame);
        cv::imshow("outputImage", outputImage);
        int btn = cv::waitKey(state);  
        if(btn == 27) break;
        else if(btn == 32)
        {
            if(state == play) state = pause;
            else if(state == pause) state = play;
        }
        //cv::waitKey();
        cap >> frame;
        cv::resize(frame, frame, cv::Size(frame.cols*0.6, frame.rows*0.6));
        inputImage = frame.clone();
    }
    cap.release();
    cv::destroyAllWindows();
    return 0;
}

cv::Mat createMask(cv::Mat &inputImage)
{
    if (inputImage.channels() > 1)
        cv::cvtColor(inputImage, inputImage, CV_RGB2GRAY);

    const int x0 = 310, y0 = 555;
    const int x1 = 845, y1 = y0;
    const int x2 = 595, y2 = 445;
    const int x3 = 485, y3 = y2;

    // then create a line masking using these three points
    cv::Mat lineMask = cv::Mat::zeros(inputImage.size(), inputImage.type());
    cv::line(lineMask, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(255, 255, 0), 1, 8, 0);
    cv::line(lineMask, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 0), 1, 8, 0);
    cv::line(lineMask, cv::Point(x2, y2), cv::Point(x3, y3), cv::Scalar(255, 255, 0), 1, 8, 0);
    cv::line(lineMask, cv::Point(x3, y3), cv::Point(x0, y0), cv::Scalar(255, 255, 0), 1, 8, 0);

    // perform contour detection on your line mask
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(lineMask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // calculate the distance to the contour
    cv::Mat raw_dist(lineMask.size(), CV_32FC1);

    for (int i = 0; i < lineMask.rows; i++)
        for (int j = 0; j < lineMask.cols; j++)
            raw_dist.at<float>(i, j) = cv::pointPolygonTest(contours[0], cv::Point2f(j, i), true);

    double minVal; double maxVal;
    cv::minMaxLoc(raw_dist, &minVal, &maxVal, 0, 0, cv::Mat());
    minVal = std::abs(minVal);
    maxVal = std::abs(maxVal);

    // depicting the distances graphically
    cv::Mat mask = cv::Mat::zeros(inputImage.size(), CV_8UC1);

    for (int i = 0; i < mask.rows; i++)
        for (int j = 0; j < mask.cols; j++)
        {
            if (raw_dist.at<float>(i, j) < 0)
            {
                mask.at<uchar>(i, j) = static_cast<uchar>(0);
                continue;
            }           
            mask.at<uchar>(i, j) = static_cast<uchar>(255);
        }
    cv::imshow("mask", mask);
    return mask;
}

cv::Mat getImageROI(const cv::Mat &mask, const cv::Mat &inputImage)
{
     // inverse the input image
    cv::Mat invInput;   
    cv::bitwise_not(inputImage, invInput);

    // then get only the region of your triangle
    cv::Mat outputImage;
    invInput.copyTo(outputImage, mask);
    cv::bitwise_not(outputImage, outputImage);;
    
    return outputImage;
}

cv::Mat colorSegmentation(const cv::Mat &inputImage)
{
    cv::Mat maskYellow, maskWhite;
        
    cv::inRange(inputImage, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), maskYellow);
    cv::inRange(inputImage, cv::Scalar(150, 150, 150), cv::Scalar(255, 255, 255), maskWhite);
    
    cv::Mat mask, processed;
    cv::bitwise_or(maskYellow, maskWhite, mask); //Combine the two masks
    cv::bitwise_and(inputImage, mask, processed); //Extrect what matches
    
    
    //Blur the image a bit so that gaps are smoother
    const cv::Size kernelSize = cv::Size(9, 9);
    cv::GaussianBlur(processed, processed, kernelSize, 0);
}