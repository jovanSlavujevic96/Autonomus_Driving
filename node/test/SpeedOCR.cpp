#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

#include <iostream>
#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>

namespace jovan{

void loadCascade(cv::CascadeClassifier *cascade, const int size, const std::string *path);
void resize(cv::Mat &img);
cv::Mat createMask(const cv::Mat &img);
cv::Mat makeROI(const cv::Mat &img, const cv::Mat &mask);
cv::Mat redColorSegmentation(const cv::Mat &image);
std::vector<cv::Rect> getRedHueContours(const cv::Mat &img);
void doPreclassification(std::vector<cv::Rect> &contours);
std::vector<cv::Rect> doClassification(std::vector<cv::Rect> &contours, const cv::Mat &img, cv::CascadeClassifier *cascade, const int size);
std::vector<cv::Rect> detectDigits(const std::vector<cv::Rect> &contours, const cv::Mat &img, cv::CascadeClassifier *cascade, const int size);
void saveDetected(const std::vector<cv::Rect> &contours, const cv::Mat &img);
void drawDetected(cv::Mat &img, const std::vector<cv::Rect> &contours, const cv::Scalar &color, const std::string object);

};

#define SpeedLimitCascade1 "/home/rtrk/myWS/src/bachelor/cascade/speed_limit/classifier/cascade.xml"
//#define SpeedLimitCascade2 "/home/rtrk/myWS/src/bachelor/cascade/speed_limit/classifier/cascade2.xml"
#define Speed80Cascade "/home/rtrk/myWS/src/bachelor/cascade/speed_limit/classifier/cascade_80.xml"

int main(void)
{
    cv::VideoCapture cap("/home/rtrk/Videos/testVideos/LimitTest2.mp4");
    if(!cap.isOpened() ) 
        return -1;
    cv::CascadeClassifier cascade_speedLimit; //[2];
    {
        //const std::string Paths[2] = {SpeedLimitCascade1, SpeedLimitCascade2};
        //jovan::loadCascade(cascade_speedLimit, 2, Paths);
        const std::string Paths = SpeedLimitCascade1;
        jovan::loadCascade(&cascade_speedLimit, 1, &Paths);
    }
    
    cv::CascadeClassifier cascade_80speedLimit;
    {
        std::string Paths = Speed80Cascade;
        jovan::loadCascade(&cascade_80speedLimit, 1, &Paths);
    }
    
    const int play = 10, pause = 0;
    int state = play;
    int i=0;
    cv::Mat mask;
    /*
    std::unique_ptr<tesseract::TessBaseAPI> ocr = std::make_unique<tesseract::TessBaseAPI>();
    ocr->Init(NULL, "eng", tesseract::OEM_LSTM_ONLY);
    ocr->SetPageSegMode(tesseract::PSM_AUTO);
    */
    while(cap.isOpened() )
    {
        cv::Mat frame;
        cap >> frame;
        if(frame.empty()) break;

        jovan::resize(frame);

        if(mask.empty() )
        {
            mask = jovan::createMask(frame);
            //cv::imshow("mask", mask);
        }
        auto ROI = jovan::makeROI(frame, mask);
        auto hue = jovan::redColorSegmentation(ROI);
  
        //cv::imshow("hue", hue);
        cv::imshow("ROI", ROI);

        auto Conts = jovan::getRedHueContours(hue);
        //jovan::doPreclassification(redConts);
        Conts = jovan::doClassification(Conts, ROI, &cascade_speedLimit, 1);  
        Conts = jovan::detectDigits(Conts, frame, &cascade_80speedLimit, 1);
        
        //jovan::saveDetected(speedSigns, frame);
        /*
        for(int i=0; i<redConts.size(); ++i)
        {
            std::stringstream ss;
            ss << "crop" << i;
            
            auto image = frame(redConts[i]).clone();
            cv::imshow(ss.str(), image );

            
            //ocr->SetImage(image.data, image.size().width, image.size().height, image.channels(), image.step1() );
            //std::cout << "text: " << ocr->GetUTF8Text() << std::endl;
        }
        */

        jovan::drawDetected(frame, Conts, cv::Scalar(0, 0, 255), "Speed Limit");
        cv::imshow("frame", frame); 

        auto btn = cv::waitKey(state);
        if(btn == 27 || btn == 'q') break;
        else if(btn == 32 || btn == 'p')
        {
            if(state == play) state = pause;
            else state = play;
        }
    }

    cap.release();
    cv::destroyAllWindows();
    //ocr->End();
    return 0;
}

void jovan::loadCascade(cv::CascadeClassifier *cascade, const int size, const std::string *path)
{
    for(int i=0; i<size; ++i)
    {
        if(!cascade[i].load(path[i]))
        {
            std::cout << "Error while loading cascade from path: " << path[i] << std::endl;
            std::exit(-1);
        }
    }
}

void jovan::resize(cv::Mat &img)
{
    cv::resize(img, img, cv::Size(
        std::round(img.size().width*0.6), 
        std::round(img.size().height*0.6)    
    ));
}

cv::Mat jovan::createMask(const cv::Mat &img)
{
    cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::Mat mask2 = mask.clone(), mask3 = mask.clone();

    //for specific video
    int x[4]={0}, y[4]={0};
    x[0] = 0, y[0] = 430;   //y=500
    x[1] = img.cols, y[1] = y[0];
    x[2] = x[1], y[2] = img.rows;
    x[3] = 0, y[3] = y[2];
  
    std::vector<cv::Point> pts(4);
    for(int i=0; i<pts.size(); ++i)
        pts[i] = cv::Point(x[i],y[i] );

    // Create a binary polygon mask
    cv::fillConvexPoly(mask, pts, cv::Scalar(255, 0, 0) );

    x[0] = std::round(img.cols*0.38), y[0] = 430;
    x[1] = std::round(img.cols*0.57), y[1] = y[0];
    x[2] = x[1], y[2] = 0;
    x[3] = x[0], y[3] = y[2];
  
    for(int i=0; i<pts.size(); ++i)
        pts[i] = cv::Point(x[i],y[i] );

    cv::fillConvexPoly(mask2, pts, cv::Scalar(255, 0, 0));

    x[0] = 0, y[0] = 0;
    x[1] = img.cols, y[1] = y[0];
    x[2] = x[1], y[2] = 185;
    x[3] = x[0], y[3] = y[2];
  
    for(int i=0; i<pts.size(); ++i)
        pts[i] = cv::Point(x[i],y[i] );

    cv::fillConvexPoly(mask3, pts, cv::Scalar(255, 0, 0));
    
    cv::Mat finalMask = mask+mask2+mask3;
    finalMask = ~finalMask;

    return finalMask; 
}

cv::Mat jovan::makeROI(const cv::Mat &img, const cv::Mat &mask)
{
    cv::Mat res;
    img.copyTo(res, mask);
    return res;
}

cv::Mat jovan::redColorSegmentation(const cv::Mat &img)
{
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    
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

std::vector<cv::Rect> jovan::getRedHueContours(const cv::Mat &img)
{
    std::vector<cv::Rect> tmpContours;
    
    cv::Mat blured;
	cv::GaussianBlur(img, blured, cv::Size(9,9), 1,1);

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

void jovan::doPreclassification(std::vector<cv::Rect> &contours)
{           
    for(int j=0; j<contours.size(); ++j)
    {
        if( (contours[j].size().height < 30 || contours[j].size().height > 80) || (contours[j].size().width < 30 || contours[j].size().width > 80) )
        {
            contours.erase(contours.begin() + j); 
        }
    }
}

std::vector<cv::Rect> jovan::doClassification(std::vector<cv::Rect> &contours, const cv::Mat &img, cv::CascadeClassifier *cascade, const int size)
{
    /*
    for(auto contour : contours)
    {
        auto limit = std::round(contour.width/3);
        for(int incrVal = limit; incrVal >= 1; --incrVal)
        {
            if((contour.x-incrVal)>=0 && (contour.y-incrVal)>=0 && 
                (contour.x+contour.width+2*incrVal)<=img.cols && 
                (contour.y+contour.height+2*incrVal)<=img.rows )
            {
                contour.x -= incrVal;
                contour.y -= incrVal;
                contour.width += 2*incrVal;
                contour.height += 2*incrVal;
                break;
            }

        }
    }
    */
    for(int i=0; i<contours.size(); ++i)
    {
        auto *contour = &contours[i];
        auto limit = std::round(contour->width/3);
        for(int incrVal = limit; incrVal >= 1; --incrVal)
        {
            if((contour->x-incrVal)>=0 && (contour->y-incrVal)>=0 && 
                (contour->x+contour->width+2*incrVal)<=img.cols && 
                (contour->y+contour->height+2*incrVal)<=img.rows )
            {
                contour->x -= incrVal;
                contour->y -= incrVal;
                contour->width += 2*incrVal;
                contour->height += 2*incrVal;
                break;
            }
        }
    }

    std::vector<cv::Rect> rects;
    for(auto contour : contours)
	{
		std::vector<cv::Rect> speedLfound[size];
        int iterrator=0;
        for(int i=0; i<size; i++)
        {		
            cascade[i].detectMultiScale( img(contour), speedLfound[i], 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(25, 25));
            if(speedLfound[i].empty() )
                break;
            ++iterrator;
        }
		if( iterrator == size /*!speedLfound[1].empty()*/ )
		{   
            speedLfound[0][0].x +=  contour.x;
            speedLfound[0][0].y +=  contour.y;
            rects.push_back( speedLfound[0][0] );
        }
	}
    return rects;
}

std::vector<cv::Rect> jovan::detectDigits(const std::vector<cv::Rect> &contours, const cv::Mat &img, cv::CascadeClassifier *cascade, const int size)
{
    std::vector<cv::Rect> rects;
    for(auto contour : contours)
    {
        std::vector<cv::Rect> digitFound;
        bool info;
        for(int j=0; j<size; j++)
        {		
            cascade[j].detectMultiScale( img(contour), digitFound, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(25, 25));
            if(! (info = digitFound.empty()) )
                break;
        }
		if( !info )
		{   
            digitFound[0].x +=  contour.x;
            digitFound[0].y +=  contour.y;
            rects.push_back( digitFound[0] );
        }
    }
    return rects;
}


unsigned int global_incr = 0;
void jovan::saveDetected(const std::vector<cv::Rect> &contours, const cv::Mat &img)
{
    for(auto rect : contours)
    {
        std::stringstream ss;
        ss << "/home/rtrk/Desktop/Detected/" << "crop" << global_incr << ".jpg";
        global_incr++;
        auto saved = img(rect).clone();
        cv::imwrite(ss.str(), saved);
        std::cout << ss.str() << " -> SAVED\n";
    }
}


void jovan::drawDetected(cv::Mat &img, const std::vector<cv::Rect> &contours, const cv::Scalar &color, const std::string object)
{
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
