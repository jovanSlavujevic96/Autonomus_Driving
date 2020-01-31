#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

#include <iostream>

#include <opencv2/opencv.hpp>

namespace jovan{

void resize(cv::Mat &image);
cv::Mat createMask(const cv::Mat &inputImage);
cv::Mat makeROI(const cv::Mat &inputImage, const cv::Mat &mask);
cv::Mat redColorSegmentation(const cv::Mat &image);
std::vector<cv::Rect> getRedHueContours(const cv::Mat &frame);
void doPreclassification(std::vector<cv::Rect> &redContours);
std::vector<cv::Rect> doClassification(std::vector<cv::Rect> &Contours, const cv::Mat &inputImage, cv::CascadeClassifier &cascade);
void drawDetected(cv::Mat &image, const std::vector<cv::Rect> &contours, const cv::Scalar &color, const std::string object);

};

int main(void)
{
    cv::VideoCapture cap("/home/rtrk/Videos/testVideos/LimitTest2.mp4");
    if(!cap.isOpened() ) return -1;

    const int play = 10, pause = 0;
    int state = play;
    int i=0;
    cv::Mat mask;
    cv::CascadeClassifier cascade;
    {
        std::string Path;
        bool info;
        if(! (info = cascade.load(Path = "/home/rtrk/myWS/src/bachelor/cascade/speed_limit.xml")) )
        {
            std::cout << "cascade load: " << std::boolalpha << info << std::endl;
            std::cout << "there's no such a classifier in path:\n" << Path << std::endl;
            return -1;
        }
    }
    while(cap.isOpened() )
    {
        cv::Mat frame;
        cap >> frame;
        jovan::resize(frame);

        if(mask.empty() )
        {
            mask = jovan::createMask(frame);
            //cv::imshow("mask", mask);
        }
        auto ROI = jovan::makeROI(frame, mask);
        auto hue = jovan::redColorSegmentation(ROI);
  
        cv::imshow("hue", hue);
        //cv::imshow("ROI", ROI);

        auto redConts = jovan::getRedHueContours(hue);
        jovan::doPreclassification(redConts);
        auto speedSigns = jovan::doClassification(redConts, ROI, cascade);  //(ROI, cascade); //(redConts, ROI, cascade);
        
        for(int i=0; i<redConts.size(); ++i)
        {
            std::stringstream ss;
            ss << "crop" << i;
            cv::imshow(ss.str(), frame(redConts[i]) );
        }

        jovan::drawDetected(frame, speedSigns, cv::Scalar(0, 0, 255), "Speed Limit");
        cv::imshow("frame", frame); 

        int btn = cv::waitKey(state);
        if(btn == 27 || btn == 'q') break;
        else if(btn == 32 || btn == 'p')
        {
            if(state == play) state = pause;
            else state = play;
        }
    }

    return 0;
}

void jovan::resize(cv::Mat &image)
{
    cv::resize(image, image, cv::Size(
        std::round(image.size().width*0.6), 
        std::round(image.size().height*0.6)    
    ));
}

cv::Mat jovan::createMask(const cv::Mat &inputImage)
{
    cv::Mat mask = cv::Mat::zeros(inputImage.size(), CV_8UC1);
    cv::Mat mask2 = mask.clone(), mask3 = mask.clone();

    //for specific video
    int x[4]={0}, y[4]={0};
    x[0] = 0, y[0] = 430;   //y=500
    x[1] = inputImage.cols, y[1] = y[0];
    x[2] = x[1], y[2] = inputImage.rows;
    x[3] = 0, y[3] = y[2];
  
    std::vector<cv::Point> pts(4);
    for(int i=0; i<pts.size(); ++i)
        pts[i] = cv::Point(x[i],y[i] );

    // Create a binary polygon mask
    cv::fillConvexPoly(mask, pts, cv::Scalar(255, 0, 0) );

    x[0] = std::round(inputImage.cols*0.38), y[0] = 430;
    x[1] = std::round(inputImage.cols*0.57), y[1] = y[0];
    x[2] = x[1], y[2] = 0;
    x[3] = x[0], y[3] = y[2];
  
    for(int i=0; i<pts.size(); ++i)
        pts[i] = cv::Point(x[i],y[i] );

    cv::fillConvexPoly(mask2, pts, cv::Scalar(255, 0, 0));

    x[0] = 0, y[0] = 0;
    x[1] = inputImage.cols, y[1] = y[0];
    x[2] = x[1], y[2] = 185;
    x[3] = x[0], y[3] = y[2];
  
    for(int i=0; i<pts.size(); ++i)
        pts[i] = cv::Point(x[i],y[i] );

    cv::fillConvexPoly(mask3, pts, cv::Scalar(255, 0, 0));
    
    cv::Mat finalMask = mask+mask2+mask3;
    finalMask = ~finalMask;

    return finalMask; 
}

cv::Mat jovan::makeROI(const cv::Mat &inputImage, const cv::Mat &mask)
{
    cv::Mat res;
    inputImage.copyTo(res, mask);
    return res;
}

cv::Mat jovan::redColorSegmentation(const cv::Mat &image)
{
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    
    cv::Mat lowerRedHueRange, upperRedHueRange;
    
    cv::inRange(hsv, 
        cv::Scalar(0,100,100), 
        cv::Scalar(10,255,255), 
        lowerRedHueRange
    );
    cv::inRange(hsv,
        cv::Scalar(160,100,100),
        cv::Scalar(179,255,255),
        upperRedHueRange
    );

    cv::Mat redMask;
    cv::addWeighted(lowerRedHueRange, 1.0f, upperRedHueRange, 1.0f, 0.0f, redMask);
    lowerRedHueRange = cv::Mat();
    
    cv::inRange(hsv, 
        cv::Scalar(0, 50, 50), 
        cv::Scalar(10, 255, 255), 
        lowerRedHueRange
    );
    cv::addWeighted(redMask, 1.0f, lowerRedHueRange, 1.0f, 0.0f, redMask);
    lowerRedHueRange = cv::Mat();

	cv::inRange(hsv, 
        cv::Scalar(160, 50, 50), 
        cv::Scalar(180, 255, 255), 
        lowerRedHueRange
    );
    cv::addWeighted(redMask, 1.0f, lowerRedHueRange, 1.0f, 0.0f, redMask);
    lowerRedHueRange = cv::Mat();

    cv::inRange(hsv, 
        cv::Scalar(140, 100, 30), 
        cv::Scalar(160, 160, 60), 
        lowerRedHueRange
    );
    cv::addWeighted(redMask, 1.0f, lowerRedHueRange, 1.0f, 0.0f, redMask);
    lowerRedHueRange = cv::Mat();
    
    cv::GaussianBlur(redMask, redMask, cv::Size(9,9), 2,2);
    return redMask;
}

std::vector<cv::Rect> jovan::getRedHueContours(const cv::Mat &frame)
{
    std::vector<cv::Rect> tmpContours;
    
    cv::Mat blured;
	cv::GaussianBlur(frame, blured, cv::Size(9,9), 1,1);

    std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(blured, 
        contours, 
        hierarchy, 
        cv::RETR_TREE, 
        cv::CHAIN_APPROX_SIMPLE
    );
    
    std::vector<std::vector<cv::Point> > contours_poly(contours.size() );
	for(int i = 0; i < contours.size(); ++i)
		cv::approxPolyDP((cv::Mat)contours[i], contours_poly[i], 3, true);

    for(int i = 0; i < contours_poly.size(); ++i)
		if (cv::contourArea(contours_poly[i]) >= 250 && contours_poly[i].size() <= 50 )
            tmpContours.push_back(cv::boundingRect(contours_poly[i]) );
    
    return tmpContours; 
}

void jovan::doPreclassification(std::vector<cv::Rect> &redContours)
{           
    for(int j=0; j<redContours.size(); ++j)
    {
        if( (redContours[j].size().height < 30 || redContours[j].size().height > 80) || (redContours[j].size().width < 30 || redContours[j].size().width > 80) )
        {
            redContours.erase(redContours.begin() + j); 
        }
    }
}

//unsigned int globalIncr=0;
std::vector<cv::Rect> jovan::doClassification(std::vector<cv::Rect> &Contours, const cv::Mat &inputImage, cv::CascadeClassifier &cascade)
{

    std::vector<cv::Rect> ContoursToResize = Contours;
    for(int i=0; i<ContoursToResize.size(); ++i)
    {
        for(int IncrVal = std::round(ContoursToResize[i].width/3); IncrVal >= 1; --IncrVal)
        {
            if( (ContoursToResize[i].x - IncrVal) >= 0 && (ContoursToResize[i].y - IncrVal) >= 0 && 
                (ContoursToResize[i].x + ContoursToResize[i].width  + 2*IncrVal) <= inputImage.size().width && 
                (ContoursToResize[i].y + ContoursToResize[i].height + 2*IncrVal) <= inputImage.size().height   )
            {
                ContoursToResize[i].x -= IncrVal;
                ContoursToResize[i].y -= IncrVal;
                ContoursToResize[i].width  += 2*IncrVal;
                ContoursToResize[i].height += 2*IncrVal;	//increase size of cropped rectangle from the frame
                break;
            }
        }
    }
    std::vector<cv::Rect> rects;
    for(int i=0; i<ContoursToResize.size(); ++i)
	{
		std::vector<cv::Rect> speedLfound;		
        cascade.detectMultiScale( inputImage(ContoursToResize[i]), speedLfound, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(25, 25));
		
		if( !speedLfound.empty() )
		{   
            speedLfound[0].x +=  ContoursToResize[i].x;
            speedLfound[0].y +=  ContoursToResize[i].y;
    
            rects.push_back( speedLfound[0] );
        }
        else
        {
            Contours.erase(Contours.begin() + i); 
            //remove from original image size vector image which is not detected by classifier as speed limit sign
        }
        
	}
    return rects;
}

void jovan::drawDetected(cv::Mat &image, const std::vector<cv::Rect> &contours, const cv::Scalar &color, const std::string object)
{
    #define img image
    cv::Mat img1 = img.clone();
	
	for(unsigned int i=0; i<contours.size(); ++i)
        cv::rectangle(img, contours[i], color, -1);
	
	cv::addWeighted(img1, 0.8, img, 0.2, 0, img);
	
	for(unsigned int i = 0 ; i <contours.size(); ++i) 
    {
        cv::rectangle(img, contours[i], color, 3);
        cv::putText(img, object, cv::Point(contours[i].x+1, contours[i].y+8), cv::FONT_HERSHEY_DUPLEX, 0.3, color, 1);
    }
}