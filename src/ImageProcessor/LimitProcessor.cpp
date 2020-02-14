#include <bachelor/ImageProcessor/LimitProcessor.hpp>
#include <cv_bridge/cv_bridge.h> 

#define SpeedCascadesPath "/home/rtrk/myWS/src/bachelor/cascade/"
#define SpeedLimitClassifier SpeedCascadesPath "speed_limit_signs/classifier/cascade2.xml"
//#define SpeedLimit80Classifier SpeedCascadesPath "speed_limit_signs/classifier/cascade_80.xml"
#define SpeedLimit80Classifier SpeedCascadesPath "speed_limit_80/classifier/cascade.xml"
#define SpeedLimit40Classifier SpeedCascadesPath "speed_limit_40/classifier/cascade.xml"

static cv::Scalar HSVtoRGB(int H, double S, double V) 
{
	double C = S * V;
	double X = C * (1 - abs(fmod(H / 60.0, 2) - 1));
	double m = V - C;
	double Rs, Gs, Bs;

	if(H >= 0 && H < 60) {
		Rs = C;
		Gs = X;
		Bs = 0;	
	}
	else if(H >= 60 && H < 120) {	
		Rs = X;
		Gs = C;
		Bs = 0;	
	}
	else if(H >= 120 && H < 180) {
		Rs = 0;
		Gs = C;
		Bs = X;	
	}
	else if(H >= 180 && H < 240) {
		Rs = 0;
		Gs = X;
		Bs = C;	
	}
	return cv::Scalar((Bs + m)*255, (Gs + m)*255, (Rs + m)*255);
}

void LimitProcessor::loadCascade(cv::CascadeClassifier *cascade, const int size, const std::string *path)
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

void LimitProcessor::resize(cv::Mat &image, const float resizeFactor)
{
    cv::resize(image, image, cv::Size(std::round(image.cols*resizeFactor), std::round(image.rows*resizeFactor)) );
}

void LimitProcessor::createMask(const cv::Mat &image)
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

    x[0] = std::round(image.cols*0.44), y[0] = 430;
    x[1] = std::round(image.cols*0.5), y[1] = y[0];
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

cv::Mat LimitProcessor::makeROI(const cv::Mat &image) const
{
    cv::Mat res;
    image.copyTo(res, m_ImageMask);
    return res;
}

void LimitProcessor::redColorSegmentation(const cv::Mat &sample, cv::Mat1b &result)
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

std::vector<cv::Rect> LimitProcessor::getRedContours(const cv::Mat1b &hueImage) const
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

void LimitProcessor::preprocessContours(const cv::Mat &image, std::vector<cv::Rect> &contours)
{
    for(int i=0; i<contours.size(); ++i)
    {
        auto *contour = &contours[i];
        float factor = 0.33f;
        int dimension;
        if(contour->width < contour->height)
        {
            dimension = contour->width;
        }
        else
        {
            dimension = contour->height;
        }
        int limit = std::round(dimension*factor);

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
}

std::vector<cv::Rect> LimitProcessor::getDetectedSpeedLimitContours(const cv::Mat &image, const std::vector<cv::Rect> &contours)
{
    std::vector<cv::Rect> speedRects;
    
    for(int i=0; i<contours.size(); ++i)
	{
		std::vector<cv::Rect> speedLfound;
        m_SpeedClassifier.detectMultiScale( image(contours[i]), speedLfound, 1.1f, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(25, 25));
		if( !speedLfound.empty() )
		{   
            
            speedLfound[0].x += contours[i].x;
            speedLfound[0].y += contours[i].y;
            speedRects.push_back(speedLfound[0]);
            
            //speedRects.push_back(contours[i]);
        }
	}
    return speedRects;
}

void LimitProcessor::resize(std::vector<cv::Rect> &contours, const float resizeFactor)
{
    for(int i=0; i<contours.size(); ++i)
    {
        contours[i].x = std::round(contours[i].x/resizeFactor);
        contours[i].y = std::round(contours[i].y/resizeFactor);
        contours[i].width = std::round(contours[i].height/resizeFactor);
        contours[i].height = std::round(contours[i].width/resizeFactor);
    }
}

int glob =0 ;
std::vector<cv::Mat> LimitProcessor::getTextImagesForOCR(const cv::Mat &image, std::vector<cv::Rect> &contours)
{
    std::vector<cv::Mat> images;
    cv::Mat1b redHue;
    LimitProcessor::redColorSegmentation(image, redHue);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1.5f, 1)); 
    for(int i=0; i<contours.size(); ++i)
    {
        auto cropic = image(contours[i]).clone();
        cv::Mat tmpCrop = redHue(contours[i]).clone();
        
        cv::floodFill(tmpCrop, cv::Point(0,0), cv::Scalar(255));
        cv::erode(tmpCrop, tmpCrop, element);
        
        std::stringstream ss;
        ss << "crop" << glob;


        cv::Mat res;
        auto tmpCrop2 = ~tmpCrop;
        cropic.copyTo(res, tmpCrop2);

        tmpCrop = tmpCrop2 & tmpCrop;

        cv::imshow(ss.str(), res);
        glob++;
        images.push_back(tmpCrop);
    }
    return images;
}

/*
std::vector<cv::Rect> LimitProcessor::getSpeedLimitValues(const cv::Mat &image, const std::vector<cv::Rect> &contours)
{
    std::vector<cv::Rect> limits;
    bool assign = true;
    for(int i=0; i<contours.size(); ++i)
    {
        bool info;
        int whichCascade = 0;
        std::vector<cv::Rect> digitFound;
        for(int j=0; j<m_NumOfClassifiers; j++)
        {	
            whichCascade = j;	
            m_LimitRecognizeClassifier[j].detectMultiScale( image(contours[i]), digitFound, 1.1f, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(25, 25));
            if(! (info = digitFound.empty()) )
            {
                break;
            }
        }
		if( !info )
		{   
            digitFound[0].x += contours[i].x;
            digitFound[0].y += contours[i].y;
            limits.push_back(digitFound[0]);
            if(assign)
            {
                assign = false;
                m_LimitValue = m_PossibleLimitValues[whichCascade];
            }
        }
    }
    return limits;
}
*/

void LimitProcessor::drawLocations(cv::Mat &image, const std::vector<cv::Rect> &contours,
    const cv::Scalar colorText = cv::Scalar(255,255,0), const cv::Scalar colorEdge = cv::Scalar(255,0,0), const std::string text = "SPEED LIMIT ")
{
    if(contours.empty() )
    {
        m_SpeedLimitDetected = false;
		return;
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
        cv::putText(image, (text/*+std::to_string(m_LimitValue)*/ ), cv::Point(contours[i].x+1, (contours[i].y+contours[i].height+18)), 
            cv::FONT_HERSHEY_DUPLEX, 0.7f, colorText, 2);
    }
    m_SpeedLimitDetected = true;
}

LimitProcessor::LimitProcessor() : m_SpeedLimitDetected{false}//, 
    //m_NumOfClassifiers{ sizeof(m_LimitRecognizeClassifier) / sizeof(*m_LimitRecognizeClassifier) },
    //m_PossibleLimitValues{40,80,50,100,130}
{
    const std::string Path = SpeedLimitClassifier;
    LimitProcessor::loadCascade(&m_SpeedClassifier, 1, &Path);
    //const std::string Paths[2] = {SpeedLimit40Classifier, SpeedLimit80Classifier};
    //LimitProcessor::loadCascade(m_LimitRecognizeClassifier, m_NumOfClassifiers, Paths);

    /*
    int h=0;
    for(int i=0; i<m_NumOfClassifiers; ++i)
    {
        m_ColorMap[m_PossibleLimitValues[i]] = HSVtoRGB(h, 1.0f, 1.0f);
        h += 30;
    }
    */
}

#define Lilac cv::Scalar(255,0,255)
void LimitProcessor::setFrame(const sensor_msgs::Image &Frame)
{
    m_Frame = cv_bridge::toCvCopy(Frame, "bgr8")->image.clone();
    auto helpImage = m_Frame.clone();
    const float resizeFactor = 0.6f;
    LimitProcessor::resize(helpImage, resizeFactor);
    
    if(m_ImageMask.empty() )
    {
        LimitProcessor::createMask(helpImage);
    }
    auto imageROI = LimitProcessor::makeROI(helpImage);
    //cv::imshow("ROI", imageROI);
    cv::Mat1b redHueImage;  //binary mask
    LimitProcessor::redColorSegmentation(imageROI, redHueImage);
    auto contours =  LimitProcessor::getRedContours(redHueImage);
    LimitProcessor::preprocessContours(helpImage, contours);
    contours = LimitProcessor::getDetectedSpeedLimitContours(helpImage, contours);
    LimitProcessor::resize(contours, resizeFactor);
    if(contours.size() )
    {
       getTextImagesForOCR(m_Frame, contours);
    }
    //contours = LimitProcessor::getSpeedLimitValues(helpImage, contours);
    
    LimitProcessor::drawLocations(m_Frame, contours); //replaced Lilac with m_ColorMap[m_LimitValue]
}

sensor_msgs::Image LimitProcessor::getProcessedFrame(void) const
{
    cv_bridge::CvImagePtr cv_ptr(std::make_unique<cv_bridge::CvImage> () );
    cv_ptr->encoding = "bgr8";
	cv_ptr->image = m_Frame;

    sensor_msgs::Image image1;
	cv_ptr->toImageMsg(image1);
    return image1;
}

bool LimitProcessor::getDetection(void) const
{
    return m_SpeedLimitDetected;
}

std::string LimitProcessor::getResult(void) const
{
    if(m_LimitValue)
    {
        return std::to_string(m_LimitValue);
    }
    else
    {
        return "NaN";
    }
}

std::vector<std::vector<int>> LimitProcessor::getCoordinates(void) const
{

}