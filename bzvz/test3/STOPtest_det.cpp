#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>
#include <vector>

#define VideoLibPath "/home/rtrk/Videos/"
#define SaveFramesPath "/home/rtrk/Pictures/DetectedObjects/"
#define StopClassifierPath "/home/rtrk/myROSworkspace/src/bachelor/classifiers/stop_classifier/cascade.xml"

#define REDcolor cv::Scalar(0, 0, 255)

void setRedHueFrame(const cv::Mat &inputArg, cv::Mat &outputArg)
{
    cv::Mat new_image = inputArg.clone();
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

    outputArg = redMask.clone();
}

std::vector<cv::Rect> redHueContours(cv::Mat &origFrame, cv::Mat &hueFrameArg)
{
    setRedHueFrame(origFrame, hueFrameArg);
    std::vector<cv::Rect> TMP;

    cv::Mat blured;
	cv::GaussianBlur(hueFrameArg, blured, cv::Size(9,9), 1,1);

    std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(blured, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    std::vector<std::vector<cv::Point> > contours_poly(contours.size() );
	for(int i = 0; i < contours.size(); ++i)
		cv::approxPolyDP((cv::Mat)contours[i], contours_poly[i], 3, true);

    for(int i = 0; i < contours_poly.size(); ++i)
		if (cv::contourArea(contours_poly[i]) >= 350 && contours_poly[i].size() >= 8 && contours_poly[i].size() <= 50 )
            TMP.push_back(cv::boundingRect(contours_poly[i]) );

    for(int i=0; i<TMP.size(); ++i)
        for(int rectIncrVal = TMP[i].width/4; rectIncrVal >= 1; --rectIncrVal)
            if( (TMP[i].x - rectIncrVal) >= 0 && (TMP[i].y - rectIncrVal) >= 0 && 
                (TMP[i].x + TMP[i].width  + 2*rectIncrVal) <= origFrame.size().width && 
                (TMP[i].y + TMP[i].height + 2*rectIncrVal) <= origFrame.size().height   )
            {
                TMP[i].x -= rectIncrVal;
                TMP[i].y -= rectIncrVal;
                TMP[i].width  += 2*rectIncrVal;
                TMP[i].height += 2*rectIncrVal;	//increase size of cropped rectangle from the frame
                break;
            }

    return TMP;
}

std::vector<cv::Rect> stopContours(cv::CascadeClassifier &haarArg, cv::Mat &frameArg, std::vector<cv::Rect> &redHueContoursArg)
{
    std::vector<cv::Rect> TMP;

    for(int i=0; i<redHueContoursArg.size(); ++i)
    {
        std::vector<cv::Rect> helpTMP;
        haarArg.detectMultiScale(frameArg(redHueContoursArg[i]), helpTMP, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(25, 25));
        if( !helpTMP.empty() )
		{
			helpTMP[0].x +=  redHueContoursArg[i].x;
			helpTMP[0].y +=  redHueContoursArg[i].y;

            //bcs of resizing
            helpTMP[0].x *= 2;
			helpTMP[0].y *= 2;
            helpTMP[0].height *= 2;
			helpTMP[0].width *= 2;
      
			TMP.push_back( helpTMP[0] );
        }
    }
    return TMP;
}

unsigned int globalIncr = 0;
void draw(std::vector<cv::Rect> &contoursArg, cv::Mat &frameArg, cv::Scalar colorArg = REDcolor, std::string textArg = "STOP")
{
    if(!contoursArg.size() )
        return;

    cv::Mat img1 = frameArg.clone();
	
	for(unsigned int i=0; i<contoursArg.size(); ++i)
	{
        cv::rectangle(frameArg, contoursArg[i], colorArg, -1);
    }

	cv::addWeighted(img1, 0.8, frameArg, 0.2, 0, frameArg);
	
	for(unsigned int i = 0 ; i <contoursArg.size(); ++i) 
    {
        cv::rectangle(frameArg, contoursArg[i], colorArg, 3);
        cv::putText(frameArg, textArg, cv::Point(contoursArg[i].x+1, contoursArg[i].y+8), cv::FONT_HERSHEY_DUPLEX, 0.3, colorArg, 1);
    }
    
    std::stringstream ss;
    ss << SaveFramesPath << globalIncr <<".jpg";
    globalIncr++;
    cv::imwrite(ss.str(), frameArg);
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        return -1;
    }
    cv::VideoCapture cap;
    {
        std::stringstream ss;
        ss << VideoLibPath << argv[1]; 
        cap = cv::VideoCapture(ss.str() );
        if(!cap.isOpened() )
        {
            return -1;
        }
        std::cout << "succ imported video\n";
    }

    cv::CascadeClassifier haar;
    if( !haar.load(StopClassifierPath) )
    {
        return -1;
    }
    std::cout << "succ loaded haar cascade classifier\n";

    cv::Mat orig_frame, frame;
    cap >> orig_frame;
    #define play 30
    #define pause 0
    int state = play;

    while( !orig_frame.empty() )
    {
        frame = orig_frame.clone();
        cv::resize(frame, frame, cv::Size(frame.cols*0.5, frame.rows*0.5) );
        cv::Mat hueFrame;
        auto redHueContoursVec = redHueContours(frame, hueFrame);
        auto stopContoursVec = stopContours(haar, frame, redHueContoursVec);
        draw(stopContoursVec,orig_frame);

        cv::imshow(argv[1], orig_frame);
        int btn = cv::waitKey(state);
        if(btn == 27)
            break;
        else if(btn == 32)
        {
            if(state)
                state = pause;
            else
                state = play;
        }
        cap >> orig_frame; 
    }
    std::cout << "succ exit program\n";
    return 0;
}