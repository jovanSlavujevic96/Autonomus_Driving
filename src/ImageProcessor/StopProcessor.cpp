#include <bachelor/ImageProcessor/StopProcessor.hpp>
#include <cv_bridge/cv_bridge.h> 

#define StopClassifierPath "/home/rtrk/myWS/src/bachelor/cascade/stop_sign.xml"

void StopProcessor::loadCascade(cv::CascadeClassifier *cascade, const int size, const std::string *path)
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

int StopProcessor::resize(cv::Mat &image, const int limit)
{
    int numOfResizing = 0;
    while(image.rows > limit && image.cols > limit)
    {
        cv::resize(image, image, cv::Size(std::round(image.cols/2), std::round(image.rows/2)) );
        ++numOfResizing;
    }
    return numOfResizing;
}

void StopProcessor::crop(cv::Mat &image)
{
    image = image(cv::Rect(0, 0, image.cols, std::round(image.rows/2))).clone();
}

void StopProcessor::redColorSegmentation(const cv::Mat &sample, cv::Mat &result)
{
    cv::Mat hsv;
	cv::cvtColor(sample, hsv, cv::COLOR_BGR2HSV);
    cv::Mat lowerRedHueRange, upperRedHueRange;
	cv::inRange(hsv, cv::Scalar(0, 25, 98), cv::Scalar(11, 39, 144), lowerRedHueRange);
	cv::inRange(hsv, cv::Scalar(156, 30, 79), cv::Scalar(179, 93, 151), upperRedHueRange);
    cv::addWeighted(lowerRedHueRange, 1.0f, upperRedHueRange, 1.0f, 0.0f, result);    
    cv::inRange(hsv, cv::Scalar(0, 25, 60), cv::Scalar(35, 40, 75), lowerRedHueRange);
    cv::addWeighted(result, 1.0f, lowerRedHueRange, 1.0f, 0.0f, result);
    cv::inRange(hsv, cv::Scalar(169, 28, 78), cv::Scalar(173,90, 152), lowerRedHueRange);
    cv::addWeighted(result, 1.0f, lowerRedHueRange, 1.0f, 0.0f, result);
    cv::inRange(hsv, cv::Scalar(157, 28, 58), cv::Scalar(177, 78 , 81 ), lowerRedHueRange);
    cv::addWeighted(result, 1.0f, lowerRedHueRange, 1.0f, 0.0f, result);
    cv::inRange(hsv, cv::Scalar(142, 67, 44), cv::Scalar(165, 102, 104), lowerRedHueRange);
    cv::addWeighted(result, 1.0f, lowerRedHueRange, 1.0f, 0.0f, result);
    cv::inRange(hsv, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), lowerRedHueRange);
    cv::addWeighted(result, 1.0f, lowerRedHueRange, 1.0f, 0.0f, result);
	cv::inRange(hsv, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), lowerRedHueRange);
    cv::addWeighted(result, 1.0f, lowerRedHueRange, 1.0f, 0.0f, result);
    cv::inRange(hsv, cv::Scalar(110, 30, 75), cv::Scalar(160, 75, 120), lowerRedHueRange);
    cv::addWeighted(result, 1.0f, lowerRedHueRange, 1.0f, 0.0f, result);
}

std::vector<cv::Rect> StopProcessor::getRedContours(const cv::Mat1b &hueImage) const
{    
    cv::Mat blured;
	cv::GaussianBlur(hueImage, blured, cv::Size(9,9), 2,2);

    std::vector<cv::Rect> redRects;   
    std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(blured, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> > contours_poly(contours.size() );
	for(int i = 0; i < contours.size(); ++i)
    {
		cv::approxPolyDP((cv::Mat)contours[i], contours_poly[i], 3, true);
    }
    for(int i = 0; i < contours_poly.size(); ++i)
	{
    	if (cv::contourArea(contours_poly[i]) >= 500 && contours_poly[i].size() >= 8 && contours_poly[i].size() <= 50 )
        {
            redRects.push_back(cv::boundingRect(contours_poly[i]) );
        }
    }
    return redRects; 
}

std::vector<cv::Rect> StopProcessor::getDetectedStopContours(const cv::Mat &image, std::vector<cv::Rect> &contours)
{
    for(int i=0; i<contours.size(); ++i)
    {
        auto *contour = &contours[i];
        auto limit = std::round(contour->width/4);
        for(int rectIncrVal = limit; rectIncrVal >= 1; --rectIncrVal)
        {
            if( (contour->x - rectIncrVal) >= 0 && (contour->y - rectIncrVal) >= 0 && 
                (contour->x + contour->width  + 2*rectIncrVal) <= image.cols && 
                (contour->y + contour->height + 2*rectIncrVal) <= image.rows   )
            {
                contour->x -= rectIncrVal;
                contour->y -= rectIncrVal;
                contour->width  += 2*rectIncrVal;
                contour->height += 2*rectIncrVal;	//increase size of cropped rectangle from the frame
                break;
            }
        }
    }
    std::vector<cv::Rect> stopRect;
    for(int i=0; i<contours.size(); ++i)
	{
		std::vector<cv::Rect> stopFound;
        m_StopClassifier.detectMultiScale( image(contours[i]), stopFound, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(25, 25));
		if( !stopFound.empty() )
		{
			stopFound[0].x +=  contours[i].x;
			stopFound[0].y +=  contours[i].y;
        
			stopRect.push_back( stopFound[0]);
		}	
	}
    return stopRect;
}

std::vector<cv::Mat> StopProcessor::getTextImagesForOCR(const int numOfResizing, std::vector<cv::Rect> &contours)
{   
    std::vector<cv::Mat> tmpVec;
    cv::Mat tmpFrame = m_Frame.clone();
    StopProcessor::redColorSegmentation(tmpFrame, tmpFrame);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1.5, 1));
    for(int i=0; i<contours.size(); ++i)
    {
        for(int j=0; j<numOfResizing; ++j)
        {
            contours[i].x *= 2;
            contours[i].y *= 2;
            contours[i].width *= 2;
            contours[i].height *= 2;
        }
        cv::Mat tmpCrop = tmpFrame(contours[i]).clone();
        
        cv::floodFill(tmpCrop, cv::Point(0,0), cv::Scalar(255));
        cv::erode(tmpCrop, tmpCrop, element);
        
        tmpVec.push_back(tmpCrop);
    }
    return tmpVec;
}

std::vector<bool> StopProcessor::getDetectionFromOCR(const std::vector<cv::Mat> &images)
{
    std::vector<bool> detection;
    for(auto image : images)
    {
        std::string word;
        m_OCR->run(image, word, NULL, NULL, NULL, cv::text::OCR_LEVEL_WORD );
        
        bool STOP[4] = {false};
        int incr = 0;
        for(auto letter : word)
        {
            if(letter == 's' || letter == 'S' || letter == '$')
            {
                STOP[0] = true;
                ++incr;  
            }
            if(letter == 't' || letter == 'T')
            {
                STOP[1] = true;
                ++incr; 
            }
            if(letter == 'o' || letter == 'O' || letter == '0')
            {
                STOP[2] = true;
                ++incr;
            }
            if(letter == 'p' || letter == 'P')
            {
                STOP[3] = true;
                ++incr;
            }
            if(incr >= 2) 
            {
                break;
            }  
        }
        if(incr >= 2)
        {
            detection.push_back(true);
        }
        else
        {
            detection.push_back(false);   
        }
    }
    return detection;
}

void StopProcessor::drawLocations(cv::Mat &image, const std::vector<bool> &detetcion, const std::vector<cv::Rect> &contours,
    const cv::Scalar colorEdge = cv::Scalar(0, 0, 255), const cv::Scalar colorText = cv::Scalar(255, 0, 255), const std::string text = "STOP")
{
    if(contours.empty() )
    {
        m_StopDetected = false;
		return;
    }
    for(int i=0; i<detetcion.size(); ++i)
    {
        if(detetcion[i])
        {
            break;
        }
        if(i == (detetcion.size()-1) && !detetcion[i] )
        {
            m_StopDetected = false;
            return;
        }
    }
    cv::Mat helpImage = image.clone();
	for(unsigned int i=0; i<contours.size(); ++i)
    {
        if(detetcion[i] )
	    {
            cv::rectangle(image, contours[i], colorEdge, -1);
        }
    }
	cv::addWeighted(helpImage, 0.8, image, 0.2, 0, image);
	for(unsigned int i = 0 ; i <contours.size(); ++i) 
    {
        if(detetcion[i] )
        {
            cv::rectangle(image, contours[i], colorEdge, 3);
            cv::putText(image, text, cv::Point(contours[i].x+1, (contours[i].width+contours[i].y+18)), 
                cv::FONT_HERSHEY_DUPLEX, 0.7f, colorText, 1);
        }
    }
    m_StopDetected = true;
}

StopProcessor::StopProcessor() : m_StopDetected{false}
{
    const std::string Path = StopClassifierPath;
    StopProcessor::loadCascade(&m_StopClassifier, 1, &Path);
    m_OCR = cv::text::OCRTesseract::create(NULL, "eng", "STOP", 1, 6);
}

void StopProcessor::setFrame(sensor_msgs::Image &rawFrame)
{
    m_Frame = cv_bridge::toCvCopy(rawFrame, "bgr8")->image.clone();

    auto helpImage = m_Frame.clone();
    auto numOfResizing = StopProcessor::resize(helpImage, 600); //resize image for faster performance!
    StopProcessor::crop(helpImage); //use only upper half of the image(frame) for faster performance!
    cv::Mat1b redHueImage;  //binary mask
    StopProcessor::redColorSegmentation(helpImage, redHueImage);  //set red hue binary mask
    auto contours = StopProcessor::getRedContours(redHueImage);
    contours = StopProcessor::getDetectedStopContours(helpImage, contours);
    auto images = StopProcessor::getTextImagesForOCR(numOfResizing, contours);
    auto detection = StopProcessor::getDetectionFromOCR(images);
    StopProcessor::drawLocations(m_Frame, detection, contours);
    //cv::resize(m_Frame, m_Frame, cv::Size(m_Frame.cols*0.75, m_Frame.rows*0.75));
}

sensor_msgs::Image StopProcessor::getProcessedFrame(void) const
{
    cv_bridge::CvImagePtr cv_ptr(std::make_unique<cv_bridge::CvImage> () );
    cv_ptr->encoding = "bgr8";
	cv_ptr->image = m_Frame;

    sensor_msgs::Image image1;
	cv_ptr->toImageMsg(image1);
    return image1;
}

bool StopProcessor::getDetection(void) const
{
    return m_StopDetected;
}

std::string StopProcessor::getResult(void) const
{
    return "Stop Sign";
}

std::vector<int> StopProcessor::getCoordinates(void) const
{

}