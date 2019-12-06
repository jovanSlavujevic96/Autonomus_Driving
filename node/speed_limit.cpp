#include <opencv2/opencv.hpp>
#include <opencv2/text/ocr.hpp>

#include <unistd.h>

#define PicLibPath "/home/rtrk/Pictures/"
void setRedHueVal(const cv::Mat &input_image, cv::Mat &output_image);
std::vector<cv::Rect> getRedHueContours(const cv::Mat &sample);
std::vector<cv::Rect> OCR_check(const std::vector<cv::Rect> &samples, cv::Mat &image, cv::Ptr<cv::text::OCRTesseract> &ocr_ptr, std::vector<std::string> &strings);

int main(int argc, char** argv)
{
    if(argc != 2)
        return -1;
    cv::Mat src, src_gray;
    {
        std::stringstream ss;
        ss << PicLibPath << argv[1];
        src = cv::imread(ss.str() );    
    }
    if(src.empty() )
        return -1; 

    while(src.cols > 1000 && src.rows > 1000 )
        cv::resize(src, src, cv::Size(src.cols/2, src.rows/2));
    setRedHueVal( src, src_gray);
    GaussianBlur( src_gray, src_gray, cv::Size(9,9), 2, 2 );
    cv::Ptr<cv::text::OCRTesseract> ocr = cv::text::OCRTesseract::create(NULL, "eng", "STOP", 1, 6);
    std::vector<std::string> strings;
    auto contour_vecs = getRedHueContours(src_gray), ocr_vecs = OCR_check(contour_vecs, src, ocr, strings);
    
    {
        cv::Mat img1 = src.clone();
        
        for(unsigned int i=0; i<ocr_vecs.size(); ++i)
            cv::rectangle(src, ocr_vecs[i], cv::Scalar(0,255,255), -1);
        
        cv::addWeighted(img1, 0.8, src, 0.2, 0, src);
        for(unsigned int i = 0 ; i <ocr_vecs.size(); ++i) 
        {
            cv::rectangle(src, ocr_vecs[i], cv::Scalar(0,255,255), 3);
            cv::putText(src, strings[i], cv::Point(ocr_vecs[i].x+1, ocr_vecs[i].y+8), cv::FONT_HERSHEY_DUPLEX, 0.3, cv::Scalar(0,0,0), 1);
        }
    }
    cv::imshow("red hue", src_gray);
    cv::imshow("out", src);
    cv::waitKey();

    return 0;
}

void setRedHueVal(const cv::Mat &input_image, cv::Mat &output_image)
{
    if(!input_image.data)
        return;
    
    cv::Mat new_image = input_image.clone(), hsv_image;
	cv::cvtColor(new_image, hsv_image, cv::COLOR_BGR2HSV);

    const cv::Scalar xyz[18] = {cv::Scalar(0,25,98),     cv::Scalar(11,39,144),   cv::Scalar(156,30,79),  cv::Scalar(179,93,151), cv::Scalar(0,25,60), 
                                cv::Scalar(35,40,75),    cv::Scalar(169,28,78),   cv::Scalar(173,90,152), cv::Scalar(157,28,58),  cv::Scalar(177,78,81), 
                                cv::Scalar(142,67,44),   cv::Scalar(165,102,104), cv::Scalar(0,50,50),    cv::Scalar(10,255,255),  cv::Scalar(160,50,50), 
                                cv::Scalar(180,255,255), cv::Scalar(110,30,75),   cv::Scalar(160,75,120) };

    cv::Mat redMask;
    for(int i=0; i<16; i+=4)
    {
        cv::Mat redMask1, redMask2;
        cv::inRange(hsv_image, xyz[i]  , xyz[i+1] , redMask1);
        cv::inRange(hsv_image, xyz[i+2], xyz[i+3] , redMask2);
        if(!i)
            redMask = redMask1+redMask2;
        else
            redMask += redMask1+redMask2;
        if(i==12)
        {
            cv::Mat redMask_;
            cv::inRange(hsv_image, xyz[16], xyz[17] , redMask_);
            redMask += redMask_;
        }
    }
    output_image = redMask.clone();
}

std::vector<cv::Rect> getRedHueContours(const cv::Mat &sample)
{
    std::vector<cv::Rect> tmpContours;

    std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(sample, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    std::vector<std::vector<cv::Point> > contours_poly(contours.size() );
	for(int i = 0; i < contours.size(); ++i)
		cv::approxPolyDP((cv::Mat)contours[i], contours_poly[i], 3, true);

    for(int i = 0; i < contours_poly.size(); ++i)
		if (cv::contourArea(contours_poly[i]) >= 500 && contours_poly[i].size() >= 8 && contours_poly[i].size() <= 50 )
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

std::vector<cv::Rect> OCR_check(const std::vector<cv::Rect> &samples, cv::Mat &image, cv::Ptr<cv::text::OCRTesseract> &ocr_ptr, std::vector<std::string> &strings)
{
    strings.clear();
    std::vector<cv::Rect> tmp;
    if(samples.empty() )
        return tmp;

    for(unsigned int i=0; i<samples.size(); ++i)
    {
        std::string output = ocr_ptr->run( image(samples[i]), 1, 0);
        
        {
            std::stringstream ss;
            ss << output << ' ' << i;
            cv::imshow(ss.str(), image(samples[i]) );
        } 
               
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
            strings.push_back(output);
            tmp.push_back(samples[i]);
            /*
            std::cout << "izlaz: ";
            std::cout << output << ' ';
            for(int a=0; a<strlen(output.c_str() ); ++a)
                printf("%d ", output.c_str()[a] );
            std::cout << std::endl << strlen(output.c_str() ) << std::endl;
            */
        }
    }
}
    
    