#include <opencv2/opencv.hpp>

#define path "/home/rtrk/Desktop/saved_frames/"
#define play 30
#define pause 0

void removeShadows(cv::Mat &image);
cv::Mat blackColorSegmentation(const cv::Mat &image);
cv::Mat whiteColorSegmentation(const cv::Mat &image);

int main(int argc, char **argv)
{
    /*
    cv::Mat image;
    if(argc == 2)
    {
        std::stringstream ss;
        ss << path << argv[1];
        image = cv::imread(ss.str() );
        if(image.empty() ) 
            return -1;
    }
    else 
        return -1;
    */
    cv::VideoCapture cap("/home/rtrk/Videos/testVideos/LimitTest2.mp4");
    if(!cap.isOpened() )
        return -1;
    int state = play;
    while(cap.isOpened() )
    {
        cv::Mat frame;
        cap >> frame;
        cv::resize(frame, frame, cv::Size(std::round(frame.cols/2),std::round(frame.rows/2)));
        cv::imshow("with shadow", frame);
        removeShadows(frame);

        /*
        auto segm1 = blackColorSegmentation(frame);
        auto segm2 = whiteColorSegmentation(frame);
    
        cv::imshow("black segm", segm1);
        cv::imshow("white segm", segm2);
        */
        cv::imshow("without shadow", frame);
        auto btn = cv::waitKey(state);
        if(btn == 27 || btn == 'q') break;
        else if(btn == 32 || btn == 'p')
        {
            if(state == play)   state = pause;
            else state = play;
        }
    }
    return 0;
}

void removeShadows(cv::Mat &image)
{
    cv::Mat gray;
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    // get full available range
    double minGray,maxGray;
    cv::minMaxLoc(gray, &minGray, &maxGray);
    //suppose current range is 30...220

    bool useRelative = false;
    if(useRelative)
    {
        // Relative clipping dark range to black
        double clipPercent = 10; 
        minGray = cvRound(minGray * (1 + clipPercent/100.0) );
        //all below minGray+10% will become black
    }
    else
    {
        //absolute clipping. Use a fixed lower bound for dark regions
        double minGrayWanted = 50; 
        minGray = minGrayWanted;
        //all below 50 will become black
    }

    // current range
    float inputRange = maxGray - minGray;

    float alpha = 255.0 / inputRange; // alpha expands current range. MaxGray will be 255
    float beta = -minGray * alpha;    // beta shifts current range so that minGray will go to 0

    image.convertTo(image, -1, alpha, beta);
}

cv::Mat blackColorSegmentation(const cv::Mat &image)
{
    cv::Mat hsv, imgThreshold;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 30), imgThreshold);

    return imgThreshold;    
}

cv::Mat whiteColorSegmentation(const cv::Mat &image)
{
    cv::Mat hsv, imgThreshold;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(180, 255, 255), imgThreshold);

    return imgThreshold;
}
