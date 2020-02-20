#include <bachelor/ImageProcessor/LimitProcessor.hpp>
#include <cv_bridge/cv_bridge.h> 

#define SpeedLimitClassifier "/home/rtrk/myWS/src/bachelor/cascade/speed_limit_sign.xml"
#define SpeedLimitClassifier2 "/home/rtrk/myWS/src/bachelor/cascade/speed_limit_sign2.xml"

#define element1 (cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1.5f, 1)) )
#define element2 (cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 + 1, 2 + 1), cv::Point(1, 1)) )

#define resizeOCRfactor 0.2f
#define resizeFrameFactor 0.6f

#define blue cv::Scalar(255,0,0)

namespace jovan //help functions
{

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
};

cv::Mat LimitProcessor::saveThenLoad(const cv::Mat& image)
{
    cv::imwrite("tmp.png", image);
    return (cv::imread("tmp.png"));
}

void LimitProcessor::loadCascade(cv::CascadeClassifier* cascade, const int size, const std::string* path)
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

void LimitProcessor::resize(cv::Mat& imageToResize, const float resizeFactor)
{
    cv::resize(imageToResize, imageToResize, cv::Size(std::round(imageToResize.cols*resizeFactor), std::round(imageToResize.rows*resizeFactor)) );
}

void LimitProcessor::resize(std::vector<cv::Rect>& contoursToResize, const float resizeFactor)
{
    for(int i=0; i<contoursToResize.size(); ++i)
    {
        contoursToResize[i].x = std::round(contoursToResize[i].x*resizeFactor);
        contoursToResize[i].y = std::round(contoursToResize[i].y*resizeFactor);
        contoursToResize[i].width = std::round(contoursToResize[i].width*resizeFactor);
        contoursToResize[i].height = std::round(contoursToResize[i].height*resizeFactor);
    }
}

void LimitProcessor::createROImask(const cv::Mat& frame)
{
    cv::Mat mask1 = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Mat mask2 = mask1.clone(), mask3 = mask1.clone();

    //for specific video
    int x[4]={0}, y[4]={0};
    x[0] = 0, y[0] = 430;  
    x[1] = frame.cols, y[1] = y[0];
    x[2] = x[1], y[2] = frame.rows;
    x[3] = 0, y[3] = y[2];
  
    std::vector<cv::Point> pts(4);
    for(int i=0; i<pts.size(); ++i)
    {
        pts[i] = cv::Point(x[i],y[i] );
    }
    // Create a binary polygon mask
    cv::fillConvexPoly(mask1, pts, cv::Scalar(255, 0, 0) );

    x[0] = std::round(frame.cols*0.44), y[0] = 430;
    x[1] = std::round(frame.cols*0.5), y[1] = y[0];
    x[2] = x[1], y[2] = 0;
    x[3] = x[0], y[3] = y[2];
  
    for(int i=0; i<pts.size(); ++i)
    {
        pts[i] = cv::Point(x[i],y[i] );
    }
    cv::fillConvexPoly(mask2, pts, cv::Scalar(255, 0, 0));

    x[0] = 0, y[0] = 0;
    x[1] = frame.cols, y[1] = y[0];
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

cv::Mat LimitProcessor::getROIframe(const cv::Mat& frame) const
{
    cv::Mat res;
    frame.copyTo(res, m_ImageMask);
    return res;
}

void LimitProcessor::redColorSegmentationMask(const cv::Mat& sample, cv::Mat1b& result)
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

std::vector<cv::Rect> LimitProcessor::getRedContours(const cv::Mat1b& hueImage) const
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

void LimitProcessor::increaseRectsForClassification(const cv::Mat& frame, std::vector<cv::Rect>& contours)
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
                (contour->x+contour->width+2*incrVal)<=frame.cols && 
                (contour->y+contour->height+2*incrVal)<=frame.rows )
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

std::vector<cv::Rect> LimitProcessor::getDetectedLimitContours(const cv::Mat& frame, const std::vector<cv::Rect>& contours)
{
    std::vector<cv::Rect> speedRects;
    
    for(int i=0; i<contours.size(); ++i)
	{
		std::vector<cv::Rect> speedLfound;
        for(int j=0; j<NumOfClassifiers; ++j)
        {
            m_SpeedClassifier[j].detectMultiScale( frame(contours[i]), speedLfound, 1.1f, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(25, 25));
            if( !speedLfound.empty() )
		    {     
                speedLfound[0].x += contours[i].x;
                speedLfound[0].y += contours[i].y;
                speedRects.push_back(speedLfound[0]);
                break;
            }
        }
        
	}
    return speedRects;
}

cv::Mat LimitProcessor::approximateCircle(cv::Mat& binaryMask)
{
    cv::cvtColor(binaryMask, binaryMask, cv::COLOR_BGR2GRAY);
    cv::morphologyEx(binaryMask, binaryMask, cv::MORPH_OPEN, element2);
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
        cv::circle(binaryMask, cv::Point(x, y), r, blue, 1, 8, 0);
    }
    return binaryMask;
}

std::vector<cv::Mat> LimitProcessor::getTextImagesForOCR(const cv::Mat& image, const cv::Mat1b& hueImage, std::vector<cv::Rect>& contours)
{
    std::vector<cv::Mat> images(contours.size() );

    cv::Mat binaryMaskCrop, approxCircle, FINAL;
    const float factor = 0.2f;
    cv::Mat1b mask;
    std::vector<cv::Rect> CONTOURS;

    for(int i=0; i<contours.size(); ++i)
    {
        binaryMaskCrop = LimitProcessor::saveThenLoad(hueImage(contours[i]) );
        approxCircle = LimitProcessor::approximateCircle(binaryMaskCrop);
        cv::inRange(approxCircle, blue, blue, mask );
        CONTOURS = LimitProcessor::getRedContours(mask);
        if(CONTOURS.empty() )
        {
            continue;
        }
        FINAL = LimitProcessor::saveThenLoad(image(contours[i]));
        FINAL = FINAL(CONTOURS[0]);
        FINAL = FINAL(cv::Rect(cv::Point(std::round(FINAL.cols*resizeOCRfactor), std::round(FINAL.rows*resizeOCRfactor)), 
            cv::Point(std::round(FINAL.cols*(1-resizeOCRfactor)), std::round(FINAL.rows*(1-resizeOCRfactor)) ) ));  
        images[i] = FINAL.clone();
    }
    return images;
}

void LimitProcessor::changeString(std::string& string)
{
    for(int j=0; j<string.size(); ++j)
    {
        if(string[j] == 'B')
        {
            string[j] = '8';
        }
        else if(string[j] == 'O' || string[j] == 'o' || string[j] == 'u' || string[j] == 'U')
        {
            string[j] = '0';
        }
        else if(string[j] == 'S' || string[j] == 's')
        {
            string[j] = '5';
        }
        else if(string[j] == 'A')
        {
            string[j] = '4';
        }
    }
    for(int i=0; i<string.size(); ++i)
    {
        if(string[i] == ')' && string[i-1] == '(')
        {
            string[i-1] = '0';
            string[i] = 0;
        }
    }
}

void LimitProcessor::getValueFromOCRstring(std::string& string, int& value, std::string& word)
{
    value = 0;
    word.clear();

    for(int j=0; j<string.size(); ++j)
    {
        if(string[j] >= '0' && string[j] <= '9')
        {
            word += string[j];
        }
        if(word.size() == 3)
        {
            break;
        }
    }

    if(!word.size() ||  word[0] == '0')
    {
        return; 
    }
    else if(word.size()==1 && word[0] != '0')
    {
        value = std::stoi(word);
        value *= 10;
    }
    else
    {
        value = std::stoi(word);
        if( (value%10) )
        {
            value = 0;
            return;
        }
    }
    if(value < 40 || value > 130)
    {
        value = 0;
    }
}

std::vector<int> LimitProcessor::getOCR(std::vector<cv::Mat>& images, std::vector<cv::Rect>& contours)
{
    std::vector<int> digits;
    std::string readen, word;
    int value;
    for(int i=0; i<images.size(); ++i)
    {
        if(images[i].empty() )
        {
            contours.erase(contours.begin()+i);
            continue;
        }
        m_OCR->run(images[i], readen, NULL, NULL, NULL, cv::text::OCR_LEVEL_WORD);
        if(readen.size() > 5)
        {
            contours.erase(contours.begin()+i);
            continue;
        }
        readen.erase(readen.begin() + (readen.size()-1));   //remove new line
        LimitProcessor::changeString(readen);
        LimitProcessor::getValueFromOCRstring(readen, value, word);
        if(!value)
        {
            contours.erase(contours.begin()+i);
            continue; 
        }
        digits.push_back(value);
    }

    return digits;
}

int LimitProcessor::getMode(const std::vector<int>& value)
{ 
    int index = 0;
    int highest = 0;
    for (unsigned int a = 0; a < value.size(); a++)
    {
        int count = 1;
        int Position = value.at(a);
        for (unsigned int b = a + 1; b < value.size(); b++)
        {
            if (value.at(b) == Position)
            {
                count++;
            }
        }
        if (count >= index)
        {
            index = count;
            highest = Position;
        }
    }
    return highest;
}

void LimitProcessor::setCoordinates(const std::vector<cv::Rect>& contours)
{
    for(int i=0; i<contours.size(); ++i)
    {
        //std::cout << contours[i].x << ' ' << contours[i].y << ' ' << contours[i].width << ' ' << contours[i].height << std::endl;
        m_Coordinates.push_back({contours[i].x, contours[i].y, contours[i].width, contours[i].height });
        //std::cout << m_Coordinates[i][0] << ' ' << m_Coordinates[i][1] << ' ' << m_Coordinates[i][2] << ' ' << m_Coordinates[i][3] << std::endl;
    }
    //std::cout << std::endl;

    const int loop = (4-m_Coordinates.size());
    for(int i=0; i<loop; ++i)
    {
        m_Coordinates.push_back({0,0,0,0});
    }
    
}

void LimitProcessor::drawLocations(cv::Mat& image, const std::vector<cv::Rect>& contours,
    const cv::Scalar& colorText = cv::Scalar(255,255,0), const cv::Scalar& colorEdge = cv::Scalar(255,0,0), const std::string& text = "SPEED LIMIT ")
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
        cv::putText(image, (text + std::to_string(m_LimitValue) ), cv::Point(contours[i].x+1, (contours[i].y+contours[i].height+18)), 
            cv::FONT_HERSHEY_DUPLEX, 0.7f, colorText, 2);
    }
    m_SpeedLimitDetected = true;
}

LimitProcessor::LimitProcessor() : 
    m_SpeedLimitDetected{false}
{
    const std::string Path[NumOfClassifiers] = {SpeedLimitClassifier, SpeedLimitClassifier2};
    LimitProcessor::loadCascade(m_SpeedClassifier, NumOfClassifiers, Path);
    
    this->m_OCR = cv::text::OCRTesseract::create(NULL, "eng", "0123456789", 1, 6); //PSM_CIRCLE_WORD
}

LimitProcessor::~LimitProcessor() 
{
    system("rm \"tmp.png\"");   //remove help png file
}

#define Lilac cv::Scalar(255,0,255)

void LimitProcessor::setFrame(const sensor_msgs::Image& frame)
{
    m_Coordinates.clear();
    cv::Mat1b redHueImage;  //binary mask
    std::vector<cv::Rect> contours;
    {
        m_Frame = cv_bridge::toCvCopy(frame, "bgr8")->image.clone();
        auto helpImage = m_Frame.clone();
        LimitProcessor::resize(helpImage, resizeFrameFactor);
        
        if(m_ImageMask.empty() )
        {
            LimitProcessor::createROImask(helpImage);
        }
        auto imageROI = LimitProcessor::getROIframe(helpImage);
        
        LimitProcessor::redColorSegmentationMask(imageROI, redHueImage);
        contours =  LimitProcessor::getRedContours(redHueImage);
        LimitProcessor::increaseRectsForClassification(helpImage, contours);
        contours = LimitProcessor::getDetectedLimitContours(helpImage, contours);
    }
    if(contours.size() )
    {
        LimitProcessor::resize(redHueImage, (1/resizeFrameFactor) );   //increase size of image
        LimitProcessor::resize(contours, (1/resizeFrameFactor) );    //increase size of rectangles

        auto images = LimitProcessor::getTextImagesForOCR(m_Frame, redHueImage, contours);
        std::vector<int> numbers = LimitProcessor::getOCR(images, contours);
        if(numbers.size() )
        {
            m_LimitValue = LimitProcessor::getMode(numbers);
        }
    }
    LimitProcessor::setCoordinates(contours);
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
    return m_Coordinates;
}