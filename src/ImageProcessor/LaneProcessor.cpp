#include <bachelor/ImageProcessor/LaneProcessor.hpp>
#include <cv_bridge/cv_bridge.h> 

#define threshold 5

void LaneProcessor::resize(cv::Mat& image, const float resizeFactor)
{
    cv::resize(image, image, cv::Size( std::round(image.size().width*resizeFactor), std::round(image.size().height*resizeFactor) ) ); 
}

cv::Mat LaneProcessor::deNoise(const cv::Mat& image) const
{
    cv::Mat deNoised;
    cv::GaussianBlur(image, deNoised, cv::Size(3, 3), 0, 0);
    return deNoised;    
}

cv::Mat LaneProcessor::edges(const cv::Mat& image) const
{
    cv::Mat gray, blur, canny;
    cv::cvtColor(image, gray, CV_RGB2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5,5),0);
    cv::Canny(blur, canny, 50, 150);
    return canny;
}

void LaneProcessor::createMask(const cv::Mat& image)
{
    cv::Mat mask = cv::Mat::zeros(image.size(), image.type());

    //for specific video
    const int x0 = 290, y0 = 555;
    const int x1 = 895, y1 = y0;
    const int x2 = 615, y2 = 445;
    const int x3 = 465, y3 = y2;
  
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

cv::Mat LaneProcessor::getROI(const cv::Mat& image) const
{
    cv::Mat output;
    // Multiply the edges image and the mask to get the output
    cv::bitwise_and(image, m_FrameMask, output);
    return output;
}

std::vector<cv::Vec4i> LaneProcessor::houghLines(const cv::Mat& image) const
{
    std::vector<cv::Vec4i> lines;
    // rho and theta are selected by trial and error
    cv::HoughLinesP(image, lines, 2, M_PI/180, 20, 10, 5);
    return lines;
}

std::vector<std::vector<cv::Vec4i>> LaneProcessor::lineSeparation(const cv::Mat& image, const std::vector<cv::Vec4i>& lines)
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
    double ImgCenter = static_cast<double>((image.cols / 2));
    
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

std::vector<cv::Point> LaneProcessor::regression(const cv::Mat& image, const std::vector<std::vector<cv::Vec4i>>& lines)
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
            cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01f, 0.01f);
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
            cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01f, 0.01f);
            leftM = left_line[1] / left_line[0];
            leftB = cv::Point(left_line[2], left_line[3]);
        }
    }

    // One the slope and offset points have been obtained, apply the line equation to obtain the line points
    int ini_y = image.rows;
    
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

    if(m_LeftFlag && m_RightFlag)
    {
        const int dotX = std::round( (output[1].x+output[3].x)/2);
        const int dotY = output[1].y;
        m_MeasuredDot = cv::Point(dotX, dotY);
    }
    return output;
}

void LaneProcessor::setCoordinates(std::vector<cv::Point>& lanePts, const float resizeFactor)
{
    //resize lines
    for(int i=0; i<lanePts.size(); ++i)
    {
        lanePts[i].x = std::round(lanePts[i].x/resizeFactor);
        lanePts[i].y = std::round(lanePts[i].y/resizeFactor);
    }
    //first push right line
    if(m_RightFlag) 
    {
        m_Coordinates.push_back({lanePts[0].x, lanePts[0].y, lanePts[1].x, lanePts[1].y });
    }
    else
    {
        m_Coordinates.push_back({0,0,0,0});
    }
    //second push left line
    if(m_LeftFlag)  
    {
        m_Coordinates.push_back({lanePts[2].x, lanePts[2].y, lanePts[3].x, lanePts[3].y});
    }
    else
    {
        m_Coordinates.push_back({0,0,0,0});
    }
    //then push measured and ref dot like lines
    if(m_LeftFlag && m_RightFlag) 
    {   
        m_Coordinates.push_back({(int)std::round(m_MeasuredDot.x/resizeFactor), (int)std::round((m_MeasuredDot.y+30)/resizeFactor), 
            (int)std::round(m_MeasuredDot.x/resizeFactor), (int)std::round((m_MeasuredDot.y-30)/resizeFactor)});

        m_Coordinates.push_back({(int)std::round(m_RefDot.x/resizeFactor), (int)std::round((m_RefDot.y+15)/resizeFactor), 
            (int)std::round(m_RefDot.x/resizeFactor), (int)std::round((m_RefDot.y-15)/resizeFactor) });
    }
    else
    {
        m_Coordinates.push_back({0,0,0,0});
        m_Coordinates.push_back({0,0,0,0});
    }
}

void LaneProcessor::plotLane(cv::Mat& image, const std::vector<cv::Point>& lanePts, const float resizeFactor)
{
    if(m_RightFlag)
    {
        cv::line(image, lanePts[0], lanePts[1], cv::Scalar(0, 255, 255), 5, CV_AA);
    }
    if(m_LeftFlag)
    {
        cv::line(image, lanePts[2], lanePts[3], cv::Scalar(0, 255, 255), 5, CV_AA);
    }
    if(m_LeftFlag && m_RightFlag)
    {
        cv::line(image, cv::Point( std::round(m_MeasuredDot.x/resizeFactor), std::round((m_MeasuredDot.y+30)/resizeFactor) ), 
            cv::Point( std::round(m_MeasuredDot.x/resizeFactor), std::round((m_MeasuredDot.y-30)/resizeFactor) ), cv::Scalar(0, 255, 255), 3, CV_AA); //measuredDot (Line)
        cv::line(image, cv::Point( std::round(m_RefDot.x/resizeFactor), std::round((m_RefDot.y+15)/resizeFactor) ), 
            cv::Point( std::round(m_RefDot.x/resizeFactor), std::round((m_RefDot.y-15)/resizeFactor) ), cv::Scalar(0, 0, 255), 3, CV_AA); //referent Dot (line)

        cv::Mat tmp = image.clone();
        std::vector<cv::Point> poly_points(4);
        poly_points[0]=(lanePts[2]);
        poly_points[1]=(lanePts[0]);
        poly_points[2]=(lanePts[1]);
        poly_points[3]=(lanePts[3]);
        cv::fillConvexPoly(tmp, poly_points, cv::Scalar(0, 0, 255), CV_AA, 0);
        cv::addWeighted(tmp, 0.3f, image, 1.0f - 0.3f, 0, image);
    }

    cv::putText(image, LaneProcessor::getResult(), cv::Point(50, 90), 
        cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cvScalar(0, 255, 0), 1, CV_AA);  //print advised direction of driving 
}

LaneProcessor::LaneProcessor() : 
    m_LeftFlag{false}, m_RightFlag{false}
    //m_Coordinates{std::vector<std::vector<int>>(4)}
{   
    //for specific  video
    m_RefDot = cv::Point(539, 445); 
}

void LaneProcessor::setFrame(const sensor_msgs::Image& frame)
{
    m_RightFlag = false;
    m_LeftFlag = false;
    m_MeasuredDot = cv::Point(); //reset all important variables
    m_Coordinates.clear();
    
    m_Frame = cv_bridge::toCvCopy(frame, "bgr8")->image.clone();
    auto helpImage = m_Frame.clone();
    const float percentage = 0.6f;
    LaneProcessor::resize(helpImage, percentage);
    cv::Mat forProcess = LaneProcessor::deNoise(helpImage);
    forProcess = LaneProcessor::edges(forProcess);
    if(m_FrameMask.empty() )
    {
        LaneProcessor::createMask(forProcess);
    }
    forProcess = LaneProcessor::getROI(forProcess);
    auto lines = LaneProcessor::houghLines(forProcess);
    auto separatedLines = LaneProcessor::lineSeparation(helpImage, lines);
    auto regression = LaneProcessor::regression(helpImage, separatedLines);
    LaneProcessor::setCoordinates(regression, percentage);
    LaneProcessor::plotLane(m_Frame, regression, percentage);
}

sensor_msgs::Image LaneProcessor::getProcessedFrame(void) const
{
    cv_bridge::CvImagePtr cv_ptr(std::make_unique<cv_bridge::CvImage> () );
    cv_ptr->encoding = "bgr8";
	cv_ptr->image = m_Frame;
	
    sensor_msgs::Image img1;
	cv_ptr->toImageMsg(img1);

    return img1;
}

std::string LaneProcessor::getResult(void) const    
{
    if(m_MeasuredDot == cv::Point() )
    {
        return "Changing the Lane";
    }
    if( m_RefDot.x - m_MeasuredDot.x >= threshold )    //its going right
    {
        return "Turn Left";
    }
    else if( m_RefDot.x - m_MeasuredDot.x <= (0-threshold) ) //its going left
    {
        return "Turn Right";
    }
    else if( std::abs(m_RefDot.x - m_MeasuredDot.x) < threshold )  
    {
        return "Go Straight";
    }
}

Topic LaneProcessor::getWatchdogTopic(void) const
{
    return ImHere_LaneDet;
}

Topic LaneProcessor::getCoordinateTopic(void) const
{
    return Coord_LaneDet;
}

Topic LaneProcessor::getECUTopic(void) const
{
    return ECU_LaneDet;
}

std::vector<std::vector<int>> LaneProcessor::getCoordinates(void) const
{
    return m_Coordinates;
}