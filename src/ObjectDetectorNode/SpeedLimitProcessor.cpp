#include <bachelor/ObjectDetectorNode/SpeedLimitProcessor.hpp>
#include <cv_bridge/cv_bridge.h> 

#define SpeedLimitClassifierPath "/home/rtrk/myROSworkspace/src/bachelor/classifiers/speedLimit1.xml"

void SpeedLimitProcessor::resize(const unsigned int limit)
{
    m_NumOfResizing = 0;
    while(m_HelpProcFrame.rows > limit && m_HelpProcFrame.cols > limit)
    {
        cv::resize(m_HelpProcFrame, m_HelpProcFrame, cv::Size(m_HelpProcFrame.cols/2, m_HelpProcFrame.rows/2) );
        ++m_NumOfResizing;
    }
}

void SpeedLimitProcessor::crop(void)
{
    m_HelpProcFrame = m_HelpProcFrame(cv::Rect(0, 0, m_HelpProcFrame.cols, m_HelpProcFrame.rows/2));
}

void SpeedLimitProcessor::setRedHueFrame(const cv::Mat &sample, cv::Mat &result)
{
    cv::Mat new_image = sample.clone(), hsv_image;
	cv::cvtColor(new_image, hsv_image, cv::COLOR_BGR2HSV);

    cv::Mat redMask1, redMask2;
	cv::inRange(hsv_image, cv::Scalar(0, 25, 98), cv::Scalar(11, 39, 144), redMask1);
	cv::inRange(hsv_image, cv::Scalar(156, 30, 79), cv::Scalar(179, 93, 151), redMask2);
	cv::Mat redMask = redMask1 + redMask2;
    
    cv::inRange(hsv_image, cv::Scalar(0, 25, 60), cv::Scalar(35, 40, 75), redMask1);
    cv::inRange(hsv_image, cv::Scalar(169, 28, 78), cv::Scalar(173,90, 152), redMask2);
    redMask += redMask1 + redMask2;

    cv::inRange(hsv_image, cv::Scalar(157, 28, 58), cv::Scalar(177, 78 , 81 ), redMask1);
    cv::inRange(hsv_image, cv::Scalar(142, 67, 44), cv::Scalar(165, 102, 104), redMask2);
    redMask += redMask1 + redMask2;
    
    cv::inRange(hsv_image, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), redMask1);
	cv::inRange(hsv_image, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), redMask2);
    redMask += redMask1 + redMask2;

    cv::inRange(hsv_image, cv::Scalar(110, 30, 75), cv::Scalar(160, 75, 120), redMask1);
    redMask += redMask1;

    GaussianBlur( redMask, redMask, cv::Size(9,9), 2, 2 );
    result = redMask.clone();
}

std::vector<cv::Rect> SpeedLimitProcessor::getRedHueContours(void) const
{
    std::vector<cv::Rect> tmpContours;

    std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(m_RedHueFrame, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    std::vector<std::vector<cv::Point> > contours_poly(contours.size() );
	for(int i = 0; i < contours.size(); ++i)
		cv::approxPolyDP((cv::Mat)contours[i], contours_poly[i], 3, true);

    for(int i = 0; i < contours_poly.size(); ++i)
		if (cv::contourArea(contours_poly[i]) >= 500 && contours_poly[i].size() >= 8 )
        {    
            cv::Rect tmp = cv::boundingRect(contours_poly[i]);
            tmp.y += (tmp.height/8);
            tmp.height -= 2*tmp.height/8;
            tmp.x += (tmp.width/20);
            tmp.width -= 2*tmp.width/20;

            tmpContours.push_back(tmp);
        }

    for(int i = 0; i < tmpContours.size(); ++i)
    {
        cv::Rect A = tmpContours[i];
        for(int j = 0; j < tmpContours.size(); ++j)
        {
            cv::Rect B = tmpContours[j];
            if(A.x > B.x && A.width < B.width && (A.x + A.width) < (B.x + B.width) &&
               A.y > B.y && A.height < B.height && (A.y + A.height) < (B.y + B.height) )
                tmpContours.erase(tmpContours.begin() + j);
        }
    }
    return tmpContours; 
}

std::vector<cv::Rect> SpeedLimitProcessor::getSpeedLimitContours(void) 
{
    auto redHueResizedContours = SpeedLimitProcessor::getRedHueContours();
    std::vector<cv::Rect> tmp;

    for(int i=0; i<redHueResizedContours.size(); ++i)
        for(int rectIncrVal = redHueResizedContours[i].width/4; rectIncrVal >= 1; --rectIncrVal)
            if( (redHueResizedContours[i].x - rectIncrVal) >= 0 && (redHueResizedContours[i].y - rectIncrVal) >= 0 && 
                (redHueResizedContours[i].x + redHueResizedContours[i].width  + 2*rectIncrVal) <= m_HelpProcFrame.size().width && 
                (redHueResizedContours[i].y + redHueResizedContours[i].height + 2*rectIncrVal) <= m_HelpProcFrame.size().height   )
            {
                redHueResizedContours[i].x -= rectIncrVal;
                redHueResizedContours[i].y -= rectIncrVal;
                redHueResizedContours[i].width  += 2*rectIncrVal;
                redHueResizedContours[i].height += 2*rectIncrVal;	//increase size of cropped rectangle from the frame
                break;
            }

    for(int i=0; i<redHueResizedContours.size(); ++i)
	{
		std::vector<cv::Rect> speedLFound;		
        m_SpeedClassifier.detectMultiScale( m_HelpProcFrame(redHueResizedContours[i]), speedLFound, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(25, 25));
		
		if( !speedLFound.empty() )
		{
			speedLFound[0].x +=  redHueResizedContours[i].x;
			speedLFound[0].y +=  redHueResizedContours[i].y;
        
			tmp.push_back( speedLFound[0] );
		}	
	}
    return tmp;
}

void SpeedLimitProcessor::setContoursByOCRcheckAndStrings(void) 
{
    auto samples = SpeedLimitProcessor::getSpeedLimitContours();

    for(unsigned int i=0; i<samples.size(); ++i)
    {
        std::string output = m_OCR->run( m_HelpProcFrame(samples[i]), 1, 0);

        int incr=0; 
        const int length = strlen(output.c_str() );
        for(unsigned int j=0; j<length; ++j)
        {
            if(output.c_str()[j] >= 48 && output.c_str()[j] <= 57)
                ++incr;
            else
                break;
        }
        if(length > 1 && incr == length )
        {
            m_Strings.push_back(output);
            for(int k=0; k<m_NumOfResizing; ++k)
            {
                samples[i].x *= 2;
                samples[i].y *= 2;
                samples[i].width *= 2;
                samples[i].height *= 2;
            }
            m_SpeedLimitContours.push_back(samples[i]);
        }
    }
}

void SpeedLimitProcessor::drawLocations(cv::Mat &img, const cv::Scalar color = cv::Scalar(0,255,255) )
{
    if(!m_SpeedLimitContours.size() || !m_Strings.size())
    {
        m_SpeedLimitDetected = false;
        m_LimitValue = 0;
        return;
    }
    cv::Mat img1 = img.clone();
        
    for(unsigned int i=0; i<m_SpeedLimitContours.size(); ++i)
        cv::rectangle(img, m_SpeedLimitContours[i], color, -1);
    
    cv::addWeighted(img1, 0.8, img, 0.2, 0, img);
    for(unsigned int i = 0 ; i <m_SpeedLimitContours.size(); ++i) 
    {
        cv::rectangle(img, m_SpeedLimitContours[i], color, 3);
        cv::putText(img, m_Strings[i], cv::Point(m_SpeedLimitContours[i].x+1, m_SpeedLimitContours[i].y+8), cv::FONT_HERSHEY_DUPLEX, 0.3, color, 1);
    }
    m_SpeedLimitDetected = true;
    m_LimitValue = std::stoi(m_Strings[0] );
}

SpeedLimitProcessor::SpeedLimitProcessor() :
    m_OCR{cv::text::OCRTesseract::create(NULL, "eng", "0123456789", 1, 6) },
    m_SpeedLimitDetected{false}, m_NumOfResizing{0}, m_LimitValue{0}
{
    if( !m_SpeedClassifier.load(SpeedLimitClassifierPath) )
    {
        std::cerr << '\t' << "Error loading classifier: " << "\n\t" << SpeedLimitClassifierPath << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

void SpeedLimitProcessor::setFrame(sensor_msgs::Image &rawFrame)
{
    m_Strings.clear();
    m_SpeedLimitContours.clear();

    m_InputFrame = cv_bridge::toCvCopy(rawFrame, "bgr8")->image.clone();
    m_HelpProcFrame = m_InputFrame.clone();

    SpeedLimitProcessor::resize(600);
    //SpeedLimitProcessor::crop(); //use only upper half of the image(frame)!
    
    SpeedLimitProcessor::setRedHueFrame(m_HelpProcFrame, m_RedHueFrame);
    SpeedLimitProcessor::setContoursByOCRcheckAndStrings();
    SpeedLimitProcessor::drawLocations(m_InputFrame);

    //cv::resize(m_InputFrame, m_InputFrame, cv::Size(m_InputFrame.cols*0.75, m_InputFrame.rows*0.75) );
    
}

sensor_msgs::Image SpeedLimitProcessor::getProcessedFrame(void) const
{
    cv_bridge::CvImagePtr cv_ptr(std::make_unique<cv_bridge::CvImage> () );
    cv_ptr->encoding = "bgr8";
	cv_ptr->image = m_InputFrame;
	
    sensor_msgs::Image img1;
	cv_ptr->toImageMsg(img1);

    return img1;
}

bool SpeedLimitProcessor::getDetection(void) const
{
    return m_SpeedLimitDetected;
}

int SpeedLimitProcessor::getValue(void) const
{
    return m_LimitValue;
}

std::string SpeedLimitProcessor::getProcessingName(void) const
{
    return "SpeedLimit";
}