#include <bachelor/ObjectDetectorNode/RoadLaneProcessor.hpp>
#include <cv_bridge/cv_bridge.h> 

cv::Mat RoadLaneProcessor::deNoise(const cv::Mat &inputImage) const
{
    cv::Mat deNoised;
    cv::GaussianBlur(inputImage, deNoised, cv::Size(3, 3), 0, 0);
    return deNoised;    
}

cv::Mat RoadLaneProcessor::edges(const cv::Mat &inputImage) const
{
    cv::Mat gray, blur, canny;
    cv::cvtColor(inputImage, gray, CV_RGB2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5,5),0);
    cv::Canny(blur, canny, 50, 150);
    return canny;
}

void RoadLaneProcessor::createMask(const cv::Mat &inputImage)
{
    cv::Mat mask = cv::Mat::zeros(inputImage.size(), inputImage.type());

    //for specific video
    const int x0 = 310, y0 = 555;
    const int x1 = 845, y1 = y0;
    const int x2 = 595, y2 = 445;
    const int x3 = 485, y3 = y2;
  
    cv::Point pts[4] = 
    {
        cv::Point(x0,y0), //cv::Point(210, 720),
        cv::Point(x1,y1), //cv::Point(550, 450),
        cv::Point(x2,y2), //cv::Point(717, 450),
        cv::Point(x3,y3) //cv::Point(1280, 720)
    };
    // Create a binary polygon mask
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 0, 0));

    m_FrameMask = mask.clone(); //return
}

cv::Mat RoadLaneProcessor::getROI(const cv::Mat &inputImage) const
{
    cv::Mat output;
    // Multiply the edges image and the mask to get the output
    cv::bitwise_and(inputImage, m_FrameMask, output);
    return output;
}

std::vector<cv::Vec4i> RoadLaneProcessor::houghLines(const cv::Mat &inputImage) const
{
    std::vector<cv::Vec4i> lines;
    // rho and theta are selected by trial and error
    cv::HoughLinesP(inputImage, lines, 2, M_PI/180, 20, 10, 5);
    return lines;
}

std::vector<std::vector<cv::Vec4i>> RoadLaneProcessor::lineSeparation(const std::vector<cv::Vec4i> &lines)
{
    std::vector<std::vector<cv::Vec4i> > output(2);
    
    cv::Point ini, fini;
    
    const double slope_thresh = 0.35;//0.3;
    std::vector<double> slopes;
  
    std::vector<cv::Vec4i> selected_lines;
    std::vector<cv::Vec4i> right_lines, left_lines;

    // Calculate the slope of all the detected lines
    for (auto i : lines) 
    {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);

        // Basic algebra: slope = (y1 - y0)/(x1 - x0)
        double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y))/(static_cast<double>(fini.x) - static_cast<double>(ini.x) );

        // If the slope is too horizontal, discard the line
        // If not, save them  and their respective slope
        if (std::abs(slope) >= slope_thresh) 
        {
            slopes.push_back(slope);
            selected_lines.push_back(i);
        }
    }

    // Split the lines into right and left lines
    double ImgCenter = static_cast<double>((m_InputFrame.cols / 2));
    
    for(int j=0; j<selected_lines.size(); ++j)
    {
        ini = cv::Point(selected_lines[j][0], selected_lines[j][1]);
        fini = cv::Point(selected_lines[j][2], selected_lines[j][3]);

        // Condition to classify line as left side or right side
        if (slopes[j] > 0 && fini.x > ImgCenter && ini.x > ImgCenter) 
        {
            right_lines.push_back(selected_lines[j]);
            m_RightFlag = true;
        } 
        else if (slopes[j] < 0 && fini.x < ImgCenter && ini.x < ImgCenter) 
        {
            left_lines.push_back(selected_lines[j]);
            m_LeftFlag = true;
        }
    }

    output[0] = right_lines;
    output[1] = left_lines;

    return output;
}

std::vector<cv::Point> RoadLaneProcessor::regression(const std::vector<std::vector<cv::Vec4i>> &lines)
{
    std::vector<cv::Point> output(4);
    cv::Point ini, fini, ini2, fini2;
    cv::Vec4d right_line, left_line;
    std::vector<cv::Point> right_pts, left_pts;
    double rightM, leftM;
    cv::Point rightB, leftB;

    // If right lines are being detected, fit a line using all the init and final points of the lines
    if (m_RightFlag) 
    {
        for (auto i : lines[0]) 
        {
            ini = cv::Point(i[0], i[1]);
            fini = cv::Point(i[2], i[3]);

            right_pts.push_back(ini);
            right_pts.push_back(fini);
        }

        if (right_pts.size() > 0) 
        {
            // The right line is formed here
            cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
            rightM = right_line[1] / right_line[0];
            rightB = cv::Point(right_line[2], right_line[3]);
        }
    }

    // If left lines are being detected, fit a line using all the init and final points of the lines
    if (m_LeftFlag == true) 
    {
        for (auto j : lines[1]) 
        {
            ini2 = cv::Point(j[0], j[1]);
            fini2 = cv::Point(j[2], j[3]);

            left_pts.push_back(ini2);
            left_pts.push_back(fini2);
        }

        if (left_pts.size() > 0) 
        {
            // The left line is formed here
            cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
            leftM = left_line[1] / left_line[0];
            leftB = cv::Point(left_line[2], left_line[3]);
        }
    }

    // One the slope and offset points have been obtained, apply the line equation to obtain the line points
    int ini_y = m_InputFrame.rows;
    
    //for specific video
    int fin_y = 445;

    double right_ini_x = ((ini_y - rightB.y) / rightM) + rightB.x;
    double right_fin_x = ((fin_y - rightB.y) / rightM) + rightB.x;

    double left_ini_x = ((ini_y - leftB.y) / leftM) + leftB.x;
    double left_fin_x = ((fin_y - leftB.y) / leftM) + leftB.x;

    output[0] = cv::Point(right_ini_x, ini_y);
    output[1] = cv::Point(right_fin_x, fin_y);
    output[2] = cv::Point(left_ini_x, ini_y);
    output[3] = cv::Point(left_fin_x, fin_y);

    const int dotX = std::round( (output[1].x+output[3].x)/2);
    const int dotY = output[1].y;
    m_MeasuredDot = cv::Point(dotX, dotY);

    return output;
}

void RoadLaneProcessor::plotLane(const std::vector<cv::Point> &lanePts)
{
    cv::Mat tmp = m_InputFrame.clone();
    // Create the transparent polygon for a better visualization of the lane
    if(m_RightFlag && m_LeftFlag)
    {
        std::vector<cv::Point> poly_points;
        poly_points.push_back(lanePts[2]);
        poly_points.push_back(lanePts[0]);
        poly_points.push_back(lanePts[1]);
        poly_points.push_back(lanePts[3]);
        cv::fillConvexPoly(tmp, poly_points, cv::Scalar(0, 0, 255), CV_AA, 0);
        cv::addWeighted(tmp, 0.3, m_InputFrame, 1.0 - 0.3, 0, m_InputFrame);
    }
    // Plot both lines of the lane boundary
    if(m_RightFlag)
        cv::line(m_InputFrame, lanePts[0], lanePts[1], cv::Scalar(0, 255, 255), 5, CV_AA);
    if(m_LeftFlag)
        cv::line(m_InputFrame, lanePts[2], lanePts[3], cv::Scalar(0, 255, 255), 5, CV_AA);


    // Plot the turn message
    cv::putText(m_InputFrame, RoadLaneProcessor::getResult(), cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);
    cv::circle(m_InputFrame, m_RefDot, 2, cv::Scalar(0,0,255), 3 );
    if(m_LeftFlag || m_RightFlag)
        cv::circle(m_InputFrame, m_MeasuredDot, 2, cv::Scalar(0,255,255), 3);
  
    m_RightFlag = false;
    m_LeftFlag = false;
    m_MeasuredDot = cv::Point();
}

RoadLaneProcessor::RoadLaneProcessor() : m_LeftFlag{false}, m_RightFlag{false}
{   
    //for specific  video
    m_RefDot = cv::Point(542, 445); 
}

void RoadLaneProcessor::setFrame(sensor_msgs::Image &rawFrame)
{
    m_InputFrame = cv_bridge::toCvCopy(rawFrame, "bgr8")->image.clone();
    {
        #define frame m_InputFrame
        cv::resize(frame,frame,cv::Size(std::round(frame.size().width*0.6), std::round(frame.size().height*0.6) ) );
    }
    cv::Mat forProcess = RoadLaneProcessor::deNoise(m_InputFrame);
    forProcess = RoadLaneProcessor::edges(forProcess);
    if(m_FrameMask.empty() )
        RoadLaneProcessor::createMask(forProcess);
    forProcess = RoadLaneProcessor::getROI(forProcess);
    auto lines = RoadLaneProcessor::houghLines(forProcess);
    auto separatedLines = RoadLaneProcessor::lineSeparation(lines);
    auto regression = RoadLaneProcessor::regression(separatedLines);
    RoadLaneProcessor::plotLane(regression);
}

sensor_msgs::Image RoadLaneProcessor::getProcessedFrame(void) const
{
    cv_bridge::CvImagePtr cv_ptr(std::make_unique<cv_bridge::CvImage> () );
    cv_ptr->encoding = "bgr8";
	cv_ptr->image = m_InputFrame;
	
    sensor_msgs::Image img1;
	cv_ptr->toImageMsg(img1);

    return img1;
}

bool RoadLaneProcessor::getDetection(void) const
{
    if(m_RightFlag || m_LeftFlag)
        return true;
    else
        return false;
}

std::string RoadLaneProcessor::getResult(void) const    
{
    const int threshold = 10;
    if( m_RefDot.x - m_MeasuredDot.x >= threshold )    //its going right
        return "Turn Left";
    else if( m_RefDot.x - m_MeasuredDot.x <= (0-threshold) ) //its going left
        return "Turn Right";
    else if( std::abs(m_RefDot.x - m_MeasuredDot.x) < threshold )  
        return "Go Straight";
}

std::string RoadLaneProcessor::getProcessorName(void) const
{
    return "Road Lane";
}