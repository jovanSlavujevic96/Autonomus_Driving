#include <opencv2/opencv.hpp>
#include <opencv2/text/ocr.hpp>

#include <memory>
#include <cv_bridge/cv_bridge.h> 
#include <sstream>

//rosrun bachelor LimitProc video.mp4
#define VideoLib "/home/rtrk/Videos/testVideos/"
//rosrun bachelor LimitProc image.jpg
#define PictureLib "/home/rtrk/Pictures/"

//classifier
#define SpeedLimitCascade "/home/rtrk/myROSworkspace/src/bachelor/cascade/speed_limit/classifier/cascade.xml"

//DetectedObjects save directory
#define SaveDetectedFramesTo "/home/rtrk/Pictures/DetectedObjects/SpeedLimitSigns/"

void ConditionalResize(cv::Mat &inputFrame, const int ResolutionLimit, int &ResizingIncrement);
void RedColorSegmentation(const cv::Mat &inputFrame, cv::Mat &outputHueFrame);
void RedColorCropping(const cv::Mat &RedHueFrame, std::vector<cv::Rect> &outputCrops);
void SpeedLimitDetCropping(const cv::Mat &inputFrame, const std::vector<cv::Rect> &inputCrops, 
    cv::CascadeClassifier &HAAR, std::vector<cv::Rect> &outputCrops);
void PrepareForOCR(const cv::Mat &inputFrame, std::vector<cv::Rect> &inputCrops, 
    std::vector<cv::Mat> &outputMatCrops, const int resizing);
void DetectionDrawing(cv::Mat &finalFrame, const std::vector<cv::Rect> &DrawingCrops, cv::Scalar color, std::string text);

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        std::cerr << "argc: " << argc << std::endl;
        return -1;
    }
    cv::VideoCapture capture;
    {
        std::stringstream ss;
        ss << VideoLib << argv[1];
        capture = cv::VideoCapture(ss.str() );
        bool info = capture.isOpened();
        if( !info )
        {
            std::cerr << "capture is opened: " << std::boolalpha << info << std::endl;
            return -1;
        }
    }

    cv::CascadeClassifier cascade;
    {
        bool info = cascade.load(SpeedLimitCascade);
        if( !info )
        {
            std::cerr << "cascade load: " << std::boolalpha << info << std::endl;
            return -1;
        }
    }

    cv::Mat frame, redHueFrame;
    capture >> frame;
    char btn = 0;
    int state = 15;
    while(btn != 27 && !frame.empty() )
    {
        cv::Mat resizedFrame = frame.clone();
        int ResIncr;
        std::vector<cv::Rect> redCrops, speedCrops;
        std::vector<cv::Mat> OCRmatCrops;

        ConditionalResize(resizedFrame, 1000, ResIncr);
        RedColorSegmentation(resizedFrame, redHueFrame);
        RedColorCropping(redHueFrame, redCrops);
        SpeedLimitDetCropping(resizedFrame, redCrops, cascade, speedCrops);
        PrepareForOCR(frame, speedCrops, OCRmatCrops, ResIncr);
        DetectionDrawing(frame, speedCrops, cv::Scalar(0,0,255), "speed Limit" );

        cv::imshow(argv[1], frame);
        btn = cv::waitKey(state);
        if(btn == 32)
        {
            if(!state)
                state = 15;
            else
                state = 0;            
        }
        capture >> frame;
    }
    
    if(frame.empty() )
        std::cout << "Video has succ finished." << std::endl;
    else
        std::cout << "btn pressed value: " << (int)btn << std::endl << "You have been escaped video." << std::endl;

    return 0;
}

void ConditionalResize(cv::Mat &inputFrame, const int ResolutionLimit, int &ResizingIncrement)
{
    ResizingIncrement = 0;
    while(inputFrame.rows > ResolutionLimit && inputFrame.cols > ResolutionLimit)
    {
        cv::resize(inputFrame, inputFrame, cv::Size(inputFrame.cols/2, inputFrame.rows/2) );
        ++ResizingIncrement;
    }
}

void RedColorSegmentation(const cv::Mat &inputFrame, cv::Mat &outputHueFrame)
{
    cv::Mat new_image = inputFrame.clone(), hsv_image;
	cv::cvtColor(new_image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat redMask1, redMask2, redMask;

	cv::inRange(hsv_image, cv::Scalar(0, 25, 98), cv::Scalar(11, 39, 144), redMask1);
	cv::inRange(hsv_image, cv::Scalar(156, 30, 79), cv::Scalar(179, 93, 151), redMask2);
	redMask = redMask1 + redMask2;
    
    cv::inRange(hsv_image, cv::Scalar(0, 25, 60), cv::Scalar(35, 40, 75), redMask1);
    cv::inRange(hsv_image, cv::Scalar(169, 28, 78), cv::Scalar(173,90, 152), redMask2);
    redMask += redMask1 + redMask2;

    cv::inRange(hsv_image, cv::Scalar(157, 28, 58), cv::Scalar(177, 78 , 81 ), redMask1);
    cv::inRange(hsv_image, cv::Scalar(142, 67, 44), cv::Scalar(165, 102, 104), redMask2);
    redMask += redMask1 + redMask2;
    
    cv::inRange(hsv_image, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), redMask1);
	cv::inRange(hsv_image, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), redMask2);
    redMask += redMask1 + redMask2;
/*
    cv::inRange(hsv_image, cv::Scalar(110, 30, 75), cv::Scalar(160, 75, 120), redMask1);
    redMask += redMask1;
*/
    GaussianBlur( redMask, redMask, cv::Size(9,9), 1, 1 );
    //cv::imshow("red mask", redMask);
    outputHueFrame = redMask.clone();
}

void RedColorCropping(const cv::Mat &RedHueFrame, std::vector<cv::Rect> &outputCrops)
{
    outputCrops.clear();
    
    std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(RedHueFrame, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    std::vector<std::vector<cv::Point> > contours_poly(contours.size() );
	for(int i = 0; i < contours.size(); ++i)
		cv::approxPolyDP((cv::Mat)contours[i], contours_poly[i], 3, true);

    for(int i = 0; i < contours_poly.size(); ++i)
		if (cv::contourArea(contours_poly[i]) >= 500 && contours_poly[i].size() >= 8 && contours_poly[i].size() <= 50 )
            outputCrops.push_back(cv::boundingRect(contours_poly[i]) );
    
    for(int i = 0; i < outputCrops.size(); ++i)
    {
        for(int rectIncrVal = outputCrops[i].width/4; rectIncrVal >= 1; --rectIncrVal)
        {
            if( (outputCrops[i].x - rectIncrVal) >= 0 && (outputCrops[i].y - rectIncrVal) >= 0 && 
                (outputCrops[i].x + outputCrops[i].width  + 2*rectIncrVal) <= RedHueFrame.size().width && 
                (outputCrops[i].y + outputCrops[i].height + 2*rectIncrVal) <= RedHueFrame.size().height   )
            {
                outputCrops[i].x -= rectIncrVal;
                outputCrops[i].y -= rectIncrVal;
                outputCrops[i].width  += 2*rectIncrVal;
                outputCrops[i].height += 2*rectIncrVal;	//increase size of cropped rectangle from the frame
                break;
            }
        }
    }
}

void SpeedLimitDetCropping(const cv::Mat &inputFrame, const std::vector<cv::Rect> &inputCrops, 
    cv::CascadeClassifier &HAAR, std::vector<cv::Rect> &outputCrops)
{
    outputCrops.clear();
    for(int i=0; i<inputCrops.size(); ++i)
	{
		std::vector<cv::Rect> stopFound;		
        HAAR.detectMultiScale( inputFrame(inputCrops[i]), stopFound, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(5, 5));
		
		if( !stopFound.empty() )
		{
			stopFound[0].x +=  inputCrops[i].x;
			stopFound[0].y +=  inputCrops[i].y;
        
			outputCrops.push_back( stopFound[0] );
		}	
	}
}

void PrepareForOCR(const cv::Mat &inputFrame, std::vector<cv::Rect> &inputCrops, 
    std::vector<cv::Mat> &outputMatCrops, const int resizing)
{
    if(!inputCrops.size() )
        return;

    outputMatCrops.clear();
    cv::Mat BlackHueFrame = inputFrame.clone();
    {
        double alpha = 3.0;
        int beta = 0 ;
        for( int y = 0; y < BlackHueFrame.rows; y++ ) 
            for( int x = 0; x < BlackHueFrame.cols; x++ ) 
                for( int c = 0; c < BlackHueFrame.channels(); c++ ) 
                {
                    BlackHueFrame.at<cv::Vec3b>(y,x)[c] = cv::saturate_cast<uchar>( alpha*BlackHueFrame.at<cv::Vec3b>(y,x)[c] + beta );
                }

        /*
        cv::imshow("brighter", BlackHueFrame);

        cv::Mat hsvImage;
        cvtColor(BlackHueFrame, hsvImage, cv::COLOR_BGR2HSV);
        cv::Mat mask;
        inRange(hsvImage, cv::Scalar(0, 0, 0, 0), cv::Scalar(180, 255, 30, 0), mask);
        */
    }
    //cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1.5, 1));
    for(int i=0; i<inputCrops.size(); ++i)
    {
        for(int i=0; i<resizing; ++i)
        {
            inputCrops[i].height *= 2;
            inputCrops[i].width *= 2;
            inputCrops[i].x *= 2;
            inputCrops[i].y *= 2;
        }
        cv::Mat tmpCrop = BlackHueFrame(inputCrops[i]).clone();
        std::stringstream ss;
        ss << "crop" << i;
        cv::imshow(ss.str(), tmpCrop);
        
        //cv::floodFill(tmpCrop, cv::Point(0,0), cv::Scalar(255));
        //cv::erode(tmpCrop, tmpCrop, element);
        
        outputMatCrops.push_back(tmpCrop);
    }
}

unsigned int incr = 0;
void DetectionDrawing(cv::Mat &finalFrame, const std::vector<cv::Rect> &DrawingCrops, cv::Scalar color, std::string text)
{
    if(DrawingCrops.empty() )
        return;

    cv::Mat img1 = finalFrame.clone();
        
    for(int i=0; i<DrawingCrops.size(); ++i)
        cv::rectangle(finalFrame, DrawingCrops[i], color, -1);
    
    cv::addWeighted(img1, 0.8, finalFrame, 0.2, 0, finalFrame);
    for(int i = 0 ; i <DrawingCrops.size(); ++i) 
    {
        cv::rectangle(finalFrame, DrawingCrops[i], color, 3);
        cv::putText(finalFrame, text, cv::Point(DrawingCrops[i].x+1, DrawingCrops[i].y+8), cv::FONT_HERSHEY_DUPLEX, 0.3, color, 1);
    }
    /*
    {
        std::stringstream ss;
        ss << SaveDetectedFramesTo << incr << ".jpg";
        cv::imwrite(ss.str(), finalFrame);
        ++incr;
    }
    */
}