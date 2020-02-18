#include <bachelor/ImageProcessor/LimitProcessor.hpp>
#include <cv_bridge/cv_bridge.h> 

#define SpeedCascadesPath "/home/rtrk/myWS/src/bachelor/cascade/"
#define SpeedLimitClassifier SpeedCascadesPath "speed_limit_signs/classifier/cascade2.xml"
//#define SpeedLimit80Classifier SpeedCascadesPath "speed_limit_signs/classifier/cascade_80.xml"

static cv::Mat SaveThenLoad(const cv::Mat &image)
{
    cv::imwrite("/home/rtrk/Pictures/test/tmp.png", image);
    return (cv::imread("/home/rtrk/Pictures/test/tmp.png"));
}

static cv::Scalar HSVtoRGB(int H, double S, double V) 
{
	double C = S * V;
	double X = C * (1 - std::abs(std::fmod(H / 60.0, 2) - 1));
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
    std::vector<cv::Rect> redRects;

    cv::Mat blured;
	cv::GaussianBlur(hueImage, blured, cv::Size(9,9), 2,2);
    std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(blured, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> contours_poly(contours.size() );
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
        contours[i].width = std::round(contours[i].width/resizeFactor);
        contours[i].height = std::round(contours[i].height/resizeFactor);
    }
}

cv::Mat LimitProcessor::approximateCircle(cv::Mat binaryMask, int dilation_elem = 0)
{
    cv::cvtColor(binaryMask, binaryMask, cv::COLOR_BGR2GRAY);
    int dilation_type = 0;
    switch(dilation_elem)
    {
        case 0:
            dilation_type = cv::MORPH_RECT; 
        break;

        case 1:
            dilation_type = cv::MORPH_CROSS; 
        break;

        case 2:
            dilation_type = cv::MORPH_ELLIPSE;
        break;
    }

    int size = 1;
    cv::Mat element = getStructuringElement(dilation_type, cv::Size(2 * size + 1, 2 * size + 1), cv::Point(size, size));
    cv::morphologyEx(binaryMask, binaryMask, cv::MORPH_OPEN, element);
    std::vector<cv::Point2f> points;
    for (int x = 0; x < binaryMask.cols; x++)
    {
        for (int y = 0; y < binaryMask.rows; y++)
        {
            if (binaryMask.at<uchar>(y, x) > 0)
            {
                points.push_back(cv::Point2f(x, y));
            }
        }
    }
    float xn = 0, xsum = 0;
    float yn = 0, ysum = 0;
    float n = points.size();

    for (int i = 0; i < n; i++)
    {
        xsum = xsum + points[i].x;
        ysum = ysum + points[i].y;
    }
    xn = xsum / n;
    yn = ysum / n;

    float ui = 0;
    float vi = 0;
    float suu = 0, suuu = 0;
    float svv = 0, svvv = 0;
    float suv = 0;
    float suvv = 0, svuu = 0;

    for (int i = 0; i < n; i++)
    {
        ui = points[i].x - xn;
        vi = points[i].y - yn;

        suu = suu + (ui * ui);
        suuu = suuu + (ui * ui * ui);

        svv = svv + (vi * vi);
        svvv = svvv + (vi * vi * vi);

        suv = suv + (ui * vi);

        suvv = suvv + (ui * vi * vi);
        svuu = svuu + (vi * ui * ui);
    }

    cv::Mat A = (cv::Mat_<float>(2, 2) << suu, suv, suv, svv);
    cv::Mat B = (cv::Mat_<float>(2, 1) << 0.5*(suuu + suvv),0.5*(svvv + svuu));
    cv::Mat abc;
    cv::solve(A, B, abc);

    float u = abc.at<float>(0);
    float v = abc.at<float>(1);

    float x = u + xn;
    float y = v + yn;

    float alpha = u * u + v * v + ((suu + svv) / n);
    float r = sqrt(alpha);

    cv::cvtColor(binaryMask, binaryMask, cv::COLOR_GRAY2BGR);
    if(x>0 && y>0 && r>0)
    {
        cv::circle(binaryMask, cv::Point(x, y), r, cv::Scalar(255, 0, 0), 1, 8, 0);
    }
    return binaryMask;
}

std::vector<cv::Mat> LimitProcessor::getTextImagesForOCR(const cv::Mat &image, std::vector<cv::Rect> &contours)
{
    std::vector<cv::Mat> images(contours.size() );
    cv::Mat1b redHue;
    LimitProcessor::redColorSegmentation(image, redHue);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1.5f, 1)); 
    for(int i=0; i<contours.size(); ++i)
    {
        auto binaryMaskCrop = SaveThenLoad(redHue(contours[i]) );
        auto originalCrop = SaveThenLoad(image(contours[i]) );
        auto approxCircle = LimitProcessor::approximateCircle(binaryMaskCrop,2);

        cv::Mat1b mask;
        cv::inRange(approxCircle, cv::Scalar(255,0,0), cv::Scalar(255,0,0), mask );
        auto CONTOURS = LimitProcessor::getRedContours(mask);
        if(CONTOURS.empty() )
        {
            images.push_back(cv::Mat());
            continue;
        }
        cv::floodFill(mask, cv::Point(0,0), cv::Scalar(255));
        cv::erode(mask, mask, element);
        mask = ~mask;

        auto alpha = SaveThenLoad(mask);
        cv::Mat foreground =  SaveThenLoad(image(contours[i]));
        cv::Mat background = cv::Mat(foreground.rows, foreground.cols, CV_8UC3, cv::Scalar(255,255,255) ); //white background
        
        foreground.convertTo(foreground, CV_32FC3);
        background.convertTo(background, CV_32FC3);
        alpha.convertTo(alpha, CV_32FC3, 1.0f/255);
        
        cv::Mat ouImage = cv::Mat::zeros(foreground.size(), foreground.type());
        cv::multiply(alpha, foreground, foreground); 
        cv::multiply(cv::Scalar::all(1.0)-alpha, background, background); 
        cv::add(foreground, background, ouImage);

        cv::Mat FINAL = SaveThenLoad(ouImage);
        FINAL = FINAL(CONTOURS[0]);
        const float factor = 0.2f;
        auto P1 = cv::Point(std::round(FINAL.cols*factor), std::round(FINAL.rows*factor));
        auto P2 = cv::Point(std::round(FINAL.cols*(1-factor)), std::round(FINAL.rows*(1-factor)) );
        FINAL = FINAL(cv::Rect(P1, P2));  

        images[i] = FINAL.clone();
    }
    return images;
}

std::vector<bool> LimitProcessor::getRecognizedLimits(const std::vector<cv::Mat> &images)
{
    std::vector<bool> detection(images.size() );
    int i=0;
    for(auto image : images)
    {
        if(image.empty() )
        {
            detection[i] = false;
            continue;
        }
        std::string word;
        m_OCR->run(image, word, NULL, NULL, NULL, cv::text::OCR_LEVEL_WORD);
        std::cout << "word: " << word << std::endl;

        bool _80[2] = {false};
        int incr=0;
        for(auto letter : word)
        {
            if( (letter == '8' || letter == 'B' || letter == 'U') && !_80[0])
            {
                _80[0] = true;
                incr++;
            }
            else if( (letter == '0' || letter == 'O' || letter == 'o') && !_80[1])
            {
                _80[1] = true;
                incr++;
            }
            if(incr == 2)
            {
                break;
            }
        }
        if(incr == 2)
        {
            detection[i] = true;
        }
        else
        {
            detection[i] = false;
        }
        ++i;
    }
    return detection;
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
    m_OCR = cv::text::OCRTesseract::create(NULL, "eng", "0123456789", 1, 6);
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

    if(contours.size() )
    {
        LimitProcessor::resize(contours, resizeFactor);
        auto images = LimitProcessor::getTextImagesForOCR(m_Frame, contours);
        auto detection = LimitProcessor::getRecognizedLimits(images);
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