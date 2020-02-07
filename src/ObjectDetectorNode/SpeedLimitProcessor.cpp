#include <bachelor/ObjectDetectorNode/SpeedLimitProcessor.hpp>
#include <cv_bridge/cv_bridge.h> 

#define SpeedCascadesPath "/home/rtrk/myWS/src/bachelor/cascade/speed_limit/classifier/"
#define SpeedLimitClassifier SpeedCascadesPath "cascade.xml"
#define SpeedLimit80Classifier SpeedCascadesPath "cascade_80.xml"

void SpeedLimitProcessor::loadCascade(cv::CascadeClassifier *cascade, const int size, const std::string *path)
{
    for(int i=0; i<size; ++i)
    {
        if(!cascade[i].load(path[i]))
        {
            std::cout << "Error while loading cascade from path: " << path[i] << std::endl;
            std::exit(-1);
        }
        else
        {
            std::cout << "Successfully loaded classifier from path: " << path[i] << std::endl;
        }
    }
}

void SpeedLimitProcessor::resize(cv::Mat &image, const float resizeFactor)
{
    cv::resize(image, image, cv::Size(std::round(image.cols*resizeFactor), std::round(image.rows*resizeFactor)) );
}

void SpeedLimitProcessor::createMask(const cv::Mat &image)
{
    cv::Mat mask1 = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::Mat mask2 = mask1.clone(), mask3 = mask1.clone();

    //for specific video
    int x[4]={0}, y[4]={0};
    x[0] = 0, y[0] = 430;  
    x[1] = image.cols, y[1] = y[0];
    x[2] = x[1], y[2] = image.rows;
    x[3] = 0, y[3] = y[2];
  
    std::vector<cv::Point> pts(4);
    for(int i=0; i<pts.size(); ++i)
    {
        pts[i] = cv::Point(x[i],y[i] );
    }
    // Create a binary polygon mask
    cv::fillConvexPoly(mask1, pts, cv::Scalar(255, 0, 0) );

    x[0] = std::round(image.cols*0.38), y[0] = 430;
    x[1] = std::round(image.cols*0.57), y[1] = y[0];
    x[2] = x[1], y[2] = 0;
    x[3] = x[0], y[3] = y[2];
  
    for(int i=0; i<pts.size(); ++i)
    {
        pts[i] = cv::Point(x[i],y[i] );
    }
    cv::fillConvexPoly(mask2, pts, cv::Scalar(255, 0, 0));

    x[0] = 0, y[0] = 0;
    x[1] = image.cols, y[1] = y[0];
    x[2] = x[1], y[2] = 185;
    x[3] = x[0], y[3] = y[2];
  
    for(int i=0; i<pts.size(); ++i)
    {
        pts[i] = cv::Point(x[i],y[i] );
    }
    cv::fillConvexPoly(mask3, pts, cv::Scalar(255, 0, 0));
    
    m_ImageMask = mask1+mask2+mask3;
    m_ImageMask = ~m_ImageMask;
}

cv::Mat SpeedLimitProcessor::makeROI(const cv::Mat &image) const
{
    cv::Mat res;
    image.copyTo(res, m_ImageMask);
    return res;
}

void SpeedLimitProcessor::redColorSegmentation(const cv::Mat &sample, cv::Mat1b &result)
{
    cv::Mat hsv;
    cv::cvtColor(sample, hsv, cv::COLOR_BGR2HSV);
    cv::Mat lowerRedHueRange, upperRedHueRange;
    cv::inRange(hsv, cv::Scalar(0,100,100), cv::Scalar(10,255,255), lowerRedHueRange );
    cv::inRange(hsv, cv::Scalar(160,100,100), cv::Scalar(179,255,255), upperRedHueRange);
    cv::addWeighted(lowerRedHueRange, 1.0f, upperRedHueRange, 1.0f, 0.0f, result);
    cv::inRange(hsv, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), lowerRedHueRange);
    cv::addWeighted(result, 1.0f, lowerRedHueRange, 1.0f, 0.0f, result);
	cv::inRange(hsv, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), lowerRedHueRange);
    cv::addWeighted(result, 1.0f, lowerRedHueRange, 1.0f, 0.0f, result);
    cv::inRange(hsv, cv::Scalar(140, 100, 30),cv::Scalar(160, 160, 60), lowerRedHueRange);
    cv::addWeighted(result, 1.0f, lowerRedHueRange, 1.0f, 0.0f, result);
}

std::vector<cv::Rect> SpeedLimitProcessor::getRedContours(const cv::Mat1b &hueImage) const
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
    	if (cv::contourArea(contours_poly[i]) >= 250 && contours_poly[i].size() <= 50 )
        {
            redRects.push_back(cv::boundingRect(contours_poly[i]) );
        }
    }
    return redRects; 
}

void SpeedLimitProcessor::eraseFromContour(std::vector<cv::Rect> &contours)
{           
    for(int j=0; j<contours.size(); ++j)
    {
        if( (contours[j].size().height < 30 || contours[j].size().height > 80) || (contours[j].size().width < 30 || contours[j].size().width > 80) )
        {
            contours.erase(contours.begin() + j); 
        }
    }
}

std::vector<cv::Rect> SpeedLimitProcessor::getDetectedSpeedLimitContours(const cv::Mat &image, std::vector<cv::Rect> &contours)
{
    for(int i=0; i<contours.size(); ++i)
    {
        auto *contour = &contours[i];
        auto limit = std::round(contour->width/3);
        for(int incrVal = limit; incrVal >= 1; --incrVal)
        {
            if( (contour->x-incrVal)>=0 && (contour->y-incrVal)>=0 && 
                (contour->x+contour->width+2*incrVal)<=image.cols && 
                (contour->y+contour->height+2*incrVal)<=image.rows )
            {
                contour->x -= incrVal;
                contour->y -= incrVal;
                contour->width += 2*incrVal;
                contour->height += 2*incrVal;
                break;
            }
        }
    }
    std::vector<cv::Rect> speedRects;
    for(int i=0; i<contours.size(); ++i)
	{
		std::vector<cv::Rect> speedLfound;
        m_SpeedClassifier.detectMultiScale( image(contours[i]), speedLfound, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(25, 25));
		if( !speedLfound.empty() )
		{   
            speedLfound[0].x +=  contours[i].x;
            speedLfound[0].y +=  contours[i].y;
            speedRects.push_back( speedLfound[0] );
        }
	}
    return speedRects;
}

std::vector<cv::Rect> SpeedLimitProcessor::getSpeedLimitValues(const cv::Mat &image, const std::vector<cv::Rect> &contours)
{
    const int possibleLimitVals[5] = {80,40,50,100,130};
    std::vector<cv::Rect> limits;
    bool assign = true;
    for(int i=0; i<contours.size(); ++i)
    {
        bool info;
        int whichCascade = 0;
        for(int j=0; j<1; j++)
        {	
            whichCascade = j;
            std::vector<cv::Rect> digitFound;	
            m_LimitRecognizeClassifier[j].detectMultiScale( image(contours[i]), digitFound, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(25, 25));
            if(! (info = digitFound.empty()) )
            {
                break;
            }
        }
		if( !info )
		{   
            limits.push_back(contours[i]);
            if(assign)
            {
                assign = false;
                m_SpeedValue = possibleLimitVals[whichCascade];
            }
        }
    }
    return limits;
}

void SpeedLimitProcessor::drawLocations(cv::Mat &image, std::vector<cv::Rect> &contours, const float resizeFactor,
    const cv::Scalar colorEdge = cv::Scalar(255,0,0), const cv::Scalar colorText = cv::Scalar(255,255,0), const std::string text = "SPEED LIMIT ")
{
    if(contours.empty() )
    {
        m_SpeedLimitDetected = false;
		return;
    }
    for(int i=0; i<contours.size(); ++i)
    {
        auto *contour = &contours[i];
        contour->x = std::round(contour->x/resizeFactor);
        contour->y = std::round(contour->y/resizeFactor);
        contour->width = std::round(contour->width/resizeFactor);
        contour->height = std::round(contour->height/resizeFactor);
    }
    cv::Mat helpImage = image.clone();
	for(unsigned int i=0; i<contours.size(); ++i)
    {
        cv::rectangle(image, contours[i], colorEdge, -1);
    }
	cv::addWeighted(helpImage, 0.8f, image, 0.2f, 0, image);
	for(unsigned int i = 0 ; i <contours.size(); ++i) 
    {
        cv::rectangle(image, contours[i], colorEdge, 3);
        cv::putText(image, (text+std::to_string(m_SpeedValue) ), cv::Point(contours[i].x+1, (contours[i].y+contours[i].height+18)), 
            cv::FONT_HERSHEY_DUPLEX, 0.7f, colorText, 2);
    }
    m_SpeedLimitDetected = true;
}

SpeedLimitProcessor::SpeedLimitProcessor() : m_SpeedLimitDetected{false}
{
    const std::string Path = SpeedLimitClassifier;
    SpeedLimitProcessor::loadCascade(&m_SpeedClassifier, 1, &Path);
    const std::string Paths[1] = {SpeedLimit80Classifier};
    SpeedLimitProcessor::loadCascade(m_LimitRecognizeClassifier, 1, Paths);
}

void SpeedLimitProcessor::setFrame(sensor_msgs::Image &rawFrame)
{
    m_Frame = cv_bridge::toCvCopy(rawFrame, "bgr8")->image.clone();
    auto helpImage = m_Frame.clone();
    SpeedLimitProcessor::resize(helpImage, 0.6f);
    
    if(m_ImageMask.empty() )
    {
        SpeedLimitProcessor::createMask(helpImage);
    }

    auto imageROI = SpeedLimitProcessor::makeROI(helpImage);
    cv::Mat1b redHueImage;  //binary mask
    SpeedLimitProcessor::redColorSegmentation(imageROI, redHueImage);
    auto contours =  SpeedLimitProcessor::getRedContours(redHueImage);
    //SpeedLimitProcessor::eraseFromContour(contours);
    contours = SpeedLimitProcessor::getDetectedSpeedLimitContours(helpImage, contours);
    contours = SpeedLimitProcessor::getSpeedLimitValues(helpImage, contours);
    SpeedLimitProcessor::drawLocations(m_Frame, contours, 0.6f);
}

sensor_msgs::Image SpeedLimitProcessor::getProcessedFrame(void) const
{
    cv_bridge::CvImagePtr cv_ptr(std::make_unique<cv_bridge::CvImage> () );
    cv_ptr->encoding = "bgr8";
	cv_ptr->image = m_Frame;

    sensor_msgs::Image image1;
	cv_ptr->toImageMsg(image1);
    return image1;
}

bool SpeedLimitProcessor::getDetection(void) const
{
    return m_SpeedLimitDetected;
}

std::string SpeedLimitProcessor::getResult(void) const
{
    return std::to_string(m_SpeedValue);
}

std::string SpeedLimitProcessor::getProcessorName(void) const
{
    return "Speed Limit Processor";
}