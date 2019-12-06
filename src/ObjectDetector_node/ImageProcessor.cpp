#include <bachelor/ObjectDetector_node/ImageProcessor.hpp>

#include <cv_bridge/cv_bridge.h> ////x

#define StopClassifierPath "/home/rtrk/myROSworkspace/src/bachelor/classifiers/stop_sign.xml"

void ImageProcessor::resize(const unsigned int limit)
{
    m_NumOfResizing = 0;
    while(m_ProcFrame.rows > limit && m_ProcFrame.cols > limit)
    {
        cv::resize(m_ProcFrame, m_ProcFrame, cv::Size(m_ProcFrame.cols/2, m_ProcFrame.rows/2) );
        ++m_NumOfResizing;
    }
}

void ImageProcessor::setRedHueFrame(const cv::Mat &sample, cv::Mat &result)
{
    cv::Mat new_image = sample.clone();
    cv::Mat hsv_image;
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
    redMask += redMask1;// + redMask2;

    result = redMask.clone();
}

std::vector<cv::Rect> ImageProcessor::getRedHueContours(void) const
{
    std::vector<cv::Rect> tmpContours;
    
    cv::Mat blured;
	cv::GaussianBlur(m_RedHueFrame, blured, cv::Size(9,9), 1,1);

    std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(blured, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    std::vector<std::vector<cv::Point> > contours_poly(contours.size() );
	for(int i = 0; i < contours.size(); ++i)
		cv::approxPolyDP((cv::Mat)contours[i], contours_poly[i], 3, true);

    for(int i = 0; i < contours_poly.size(); ++i)
		if (cv::contourArea(contours_poly[i]) >= 500 && contours_poly[i].size() >= 8 && contours_poly[i].size() <= 50 )
            tmpContours.push_back(cv::boundingRect(contours_poly[i]) );
    
    return tmpContours; 
}

void ImageProcessor::setStopSignContours(void)
{
    auto redHueResizedContours = ImageProcessor::getRedHueContours();
    
    for(int i=0; i<redHueResizedContours.size(); ++i)
        for(int rectIncrVal = redHueResizedContours[i].width/4; rectIncrVal >= 1; --rectIncrVal)
            if( (redHueResizedContours[i].x - rectIncrVal) >= 0 && (redHueResizedContours[i].y - rectIncrVal) >= 0 && 
                (redHueResizedContours[i].x + redHueResizedContours[i].width  + 2*rectIncrVal) <= m_ProcFrame.size().width && 
                (redHueResizedContours[i].y + redHueResizedContours[i].height + 2*rectIncrVal) <= m_ProcFrame.size().height   )
            {
                redHueResizedContours[i].x -= rectIncrVal;
                redHueResizedContours[i].y -= rectIncrVal;
                redHueResizedContours[i].width  += 2*rectIncrVal;
                redHueResizedContours[i].height += 2*rectIncrVal;	//increase size of cropped rectangle from the frame
                break;
            }

    for(int i=0; i<redHueResizedContours.size(); ++i)
	{
		std::vector<cv::Rect> stopFound;		
        m_StopClassifier.detectMultiScale( m_ProcFrame(redHueResizedContours[i]), stopFound, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(25, 25));
		
		if( !stopFound.empty() )
		{
			stopFound[0].x +=  redHueResizedContours[i].x;
			stopFound[0].y +=  redHueResizedContours[i].y;
        
			m_StopSignContours.push_back( stopFound[0] );
		}	
	}
}

std::vector<cv::Mat> ImageProcessor::getPreprocessedImagesForOCR(void)
{   
    std::vector<cv::Mat> tmpVec;
    cv::Mat tmpFrame = m_OrigSizeFrame.clone();
    ImageProcessor::setRedHueFrame(tmpFrame, tmpFrame);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1.5, 1));
    for(int i=0; i<m_StopSignContours.size(); ++i)
    {
        for(int j=0; j<m_NumOfResizing; ++j)
        {
            m_StopSignContours[i].x *= 2;
            m_StopSignContours[i].y *= 2;
            m_StopSignContours[i].width *= 2;
            m_StopSignContours[i].height *= 2;
        }
        cv::Mat tmpCrop = tmpFrame(m_StopSignContours[i]).clone();
        
        cv::floodFill(tmpCrop, cv::Point(0,0), cv::Scalar(255));
        cv::erode(tmpCrop, tmpCrop, element);
        
        tmpVec.push_back(tmpCrop);
    }
    
    return tmpVec;
}

void ImageProcessor::setOCRdetection(void)
{
    auto ImagesForOCR = ImageProcessor::getPreprocessedImagesForOCR();

    for(int i=0; i<ImagesForOCR.size(); ++i)
    {
        std::string output;
        m_OCR->run(ImagesForOCR[i], output, NULL, NULL, NULL, cv::text::OCR_LEVEL_WORD );
        
        bool STOP[4] = {false};
        for(int j=0; j<strlen(output.c_str()); ++j)
        {
            if(output.c_str()[j] == 's' || output.c_str()[j] == 'S' || output.c_str()[j] == '$')
                STOP[0] = true;  
            if(output.c_str()[j] == 't' || output.c_str()[j] == 'T')
                STOP[1] = true; 
            if(output.c_str()[j] == 'o' || output.c_str()[j] == 'O' || output.c_str()[j] == '0')
                STOP[2] = true;
            if(output.c_str()[j] == 'p' || output.c_str()[j] == 'P')
                STOP[3] = true;  
        }
        unsigned int incr = 0;
        for(int j=0; j<4; ++j)      
            if(STOP[j])
                ++incr;
        if(incr >= 2)
            m_OCRdetection.push_back(true);
        else
            m_OCRdetection.push_back(false);
        
    }
}

void ImageProcessor::drawLocations(cv::Mat &img, const cv::Scalar color, const std::string text)
{
    for(int i=0; i<m_OCRdetection.size(); ++i)
    {
        if(m_OCRdetection[i])
            break;
        if(i == (m_OCRdetection.size()-1) && !m_OCRdetection[i] )
            return;
    }

    const int stopCompare = strcmp(text.c_str(), "STOP sign");
    if(m_StopSignContours.empty() )
    {
        if(!stopCompare)
            m_StopDetected = false;
		return;
    }
    cv::Mat img1 = img.clone();
	
	for(unsigned int i=0; i<m_StopSignContours.size(); ++i)
        if(m_OCRdetection[i] )
	        cv::rectangle(img, m_StopSignContours[i], color, -1);
	
	cv::addWeighted(img1, 0.8, img, 0.2, 0, img);
	
	for(unsigned int i = 0 ; i <m_StopSignContours.size(); ++i) 
        if(m_OCRdetection[i] )
        {
            cv::rectangle(img, m_StopSignContours[i], color, 3);
            cv::putText(img, text, cv::Point(m_StopSignContours[i].x+1, m_StopSignContours[i].y+8), cv::FONT_HERSHEY_DUPLEX, 0.3, color, 1);
        }
	
    std::cout << '\t' << text << " detected." << std::endl;
    if(!stopCompare)
        m_StopDetected = true;
}

ImageProcessor::ImageProcessor() : m_NumOfResizing{0}, m_StopDetected{false} 
    , m_OCR{cv::text::OCRTesseract::create(NULL, "eng", "STOP", 1, 6)}
{
    if( !m_StopClassifier.load(StopClassifierPath) )
    {
        std::cerr << '\t' << "Error loading classifier: " << "\n\t" << StopClassifierPath << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

ImageProcessor::~ImageProcessor()
{
    system("clear");
}

void ImageProcessor::setFrame(sensor_msgs::Image &rawFrame)
{
    m_OrigSizeFrame = cv_bridge::toCvCopy(rawFrame, "bgr8")->image;

    m_StopSignContours.clear();
    m_OCRdetection.clear();
    m_ProcFrame = m_OrigSizeFrame.clone();
    ImageProcessor::resize(600); //umanji
    //iskropaj!!!
    ImageProcessor::setRedHueFrame(m_ProcFrame, m_RedHueFrame);
    ImageProcessor::setStopSignContours();
    ImageProcessor::setOCRdetection();
    ImageProcessor::drawLocations(m_OrigSizeFrame, cv::Scalar(0, 0, 255), "STOP sign");
    cv::resize(m_OrigSizeFrame, m_OrigSizeFrame, cv::Size(m_OrigSizeFrame.cols*0.75, m_OrigSizeFrame.rows*0.75));
}

sensor_msgs::Image ImageProcessor::getProcessedFrame(void) const
{
    cv_bridge::CvImagePtr cv_ptr(std::make_unique<cv_bridge::CvImage> () );
    cv_ptr->encoding = "bgr8";
	cv_ptr->image = m_OrigSizeFrame;
	
    sensor_msgs::Image img1;
	cv_ptr->toImageMsg(img1);

    return img1;
}

bool ImageProcessor::getDetection(void) const
{
    if(m_StopDetected)
        return true;
    return false; 
}