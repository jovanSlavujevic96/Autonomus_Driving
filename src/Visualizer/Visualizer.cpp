#include <bachelor/Visualizer/Visualizer.hpp>
#include <cv_bridge/cv_bridge.h>
#include <bachelor/DataSender/DataSender.hpp>
#include <numeric>

#define NumOfDetectors (sizeof(Visualizer::m_DisplayIt)/sizeof(*Visualizer::m_DisplayIt))
#define Red cv::Scalar(0,0,255)
#define Yellow cv::Scalar(0, 255, 255)
#define Lilac cv::Scalar(255,0,255)

bool Visualizer::pauseCheck(void)
{
    auto btn = cv::waitKey(1);
    if(btn == 'p' || btn == 32) //<space> asci val -> 32
    {
        m_Ignore = false;
        m_Pause = !m_Pause;
        return true;
    }
    else if(btn == 'q' || btn == 27) //<esc> asci val -> 27
    {
        return false;
    }
    m_Ignore = true;
    return true;
}

bool Visualizer::drawDetected(void)
{
    if(m_LanePointsVec.size() != 4 || m_Frame.empty() )
    {
        return false;
    }
    
    bool rightLine=false, leftLine=false;
    //if there are lines draw them
    if( (m_LanePointsVec[0][0] != cv::Point(0,0)) && (m_LanePointsVec[0][1] != cv::Point(0,0)) )    //if both dots are zeros, there are no line
    {
        rightLine = true;
        cv::line(m_Frame, m_LanePointsVec[0][0], m_LanePointsVec[0][1], Yellow, 5, CV_AA);
    }
    if( (m_LanePointsVec[1][0] != cv::Point(0,0)) && (m_LanePointsVec[1][1] != cv::Point(0,0)) )
    {
        leftLine = true;
        cv::line(m_Frame, m_LanePointsVec[1][0], m_LanePointsVec[1][1], Yellow, 5, CV_AA);
    }
    if(rightLine && leftLine)
    {
        cv::line(m_Frame, m_LanePointsVec[2][0], m_LanePointsVec[2][1], Yellow, 3, CV_AA); //measuredDot (Line)
        cv::line(m_Frame, m_LanePointsVec[3][0], m_LanePointsVec[3][1], Red, 3, CV_AA); //referent Dot (line)

        cv::Mat tmp = m_Frame.clone();
        std::vector<cv::Point> poly_points;
        poly_points.push_back(m_LanePointsVec[0][0]);
        poly_points.push_back(m_LanePointsVec[0][1]);
        poly_points.push_back(m_LanePointsVec[1][1]);
        poly_points.push_back(m_LanePointsVec[1][0]);
        cv::fillConvexPoly(tmp, poly_points, Red, CV_AA, 0);
        cv::addWeighted(tmp, 0.3f, m_Frame, 1.0f - 0.3f, 0, m_Frame);
    }
    return true;
}

bool Visualizer::drawDetected(const std::vector<cv::Rect> &contours, const cv::Scalar &contourColor, const std::string sign, const cv::Scalar &textColor)
{
    if(contours.size() != 4 || m_Frame.empty() ) 
    {
        return false;
    }

    int size=0;
    for(int i=0; i<contours.size(); ++i)
    {
        if(contours[i] == cv::Rect(0,0,0,0) )
        {
            if(i==0)
            {
                return true;    //there are no rects and that's ok
            }
            size = i;
            break;
        }    
    }

    auto helpImage = m_Frame.clone();
	for(int i=0; i<size; ++i)
    {
        cv::rectangle(m_Frame, contours[i], contourColor, -1);
    }
	cv::addWeighted(helpImage, 0.8f, m_Frame, 0.2f, 0, m_Frame);
	for(int i = 0 ; i <size; ++i) 
    {
        cv::rectangle(m_Frame, contours[i], contourColor, 3);
        cv::putText(m_Frame, sign, cv::Point(contours[i].x+1, (contours[i].width+contours[i].y+18)), 
            cv::FONT_HERSHEY_DUPLEX, 0.7f, textColor, 1);
    }
    return true;
}

void Visualizer::resetVariables(void)
{
    m_DisplayIt[0] = !m_DrawLanes;
    m_DisplayIt[1] = !m_DrawStopRects;
    m_DisplayIt[2] = !m_DrawLimitRects;
    m_LanePointsVec.clear();
    m_StopRectsVec.clear();
}

Visualizer::Visualizer(bool DrawLanes, bool DrawStopRects, bool DrawLimitRects) : 
    m_PauseSender{std::make_unique<DataSender<std_msgs::Bool>>(PauseOrPlay)},
    m_ToWatchdog{std::make_unique<DataSender<std_msgs::Bool>>(ImHere_Visual)},
    m_DrawLanes{DrawLanes},
    m_DrawStopRects{DrawStopRects},
    m_DrawLimitRects{DrawLimitRects}
{
    this->m_Ignore = false;
    this->m_Pause = false;
    this->m_FrameRecevied = false;
    this->m_CoordsReceived = true;
    
    this->m_DisplayIt[0] = !m_DrawLanes;
    this->m_DisplayIt[1] = !m_DrawStopRects;
    this->m_DisplayIt[2] = !m_DrawLimitRects;
}

bool Visualizer::doStuff(void) 
{
    {   
        //msg instance dissapears out of scope
        std_msgs::Bool msg;
        msg.data = true;
        m_ToWatchdog->Publish(msg);
    }
    if(!m_Ignore)	//if (<space> or 'p') pressed -> m_Ignore = false -> send pause or start
    {
        std_msgs::Bool msg;
        msg.data = m_Pause;
        m_PauseSender->Publish(msg); //send pause or start to publisher (VideoPlayerNode)
    }
    if(std::accumulate(m_DisplayIt, m_DisplayIt + NumOfDetectors, 0) == NumOfDetectors && !m_CoordsReceived)
    {
        try
        {
            cv::imshow("Visualization: ", m_Frame);
        }
        catch(cv::Exception &error)
        {
            return false;
        }
        m_CoordsReceived = true;
    }
    if(!Visualizer::pauseCheck() )	//if exit button (<esc> or 'q') is pressed exit the program
	{
        return false;
    }
    Visualizer::resetVariables();
    return true;
}

void Visualizer::update(const sensor_msgs::Image &_msg, Topics _subjTopic) 
{
    if(_subjTopic == RawFrame && m_CoordsReceived)    
    {
        m_Frame = cv_bridge::toCvCopy(_msg, "bgr8")->image.clone();
        m_FrameRecevied = true;
        m_CoordsReceived = false;
    }
}

void Visualizer::update(const bachelor::Coordinates &_msg, Topics _subjTopic) 
{
    switch(_subjTopic)
    {
        case Coord_LaneDet:
        {
            if(m_DrawLanes && m_FrameRecevied)
            {
                for(int i=0; i<4; ++i)
                {
                    std::vector<cv::Point> tmp;
                    tmp.push_back(cv::Point(_msg.X1[i], _msg.Y1[i]));
                    tmp.push_back(cv::Point(_msg.X2_Width[i], _msg.Y2_Height[i]));
                    m_LanePointsVec.push_back(tmp);
                }
                m_DisplayIt[0] = Visualizer::drawDetected();
            }
            break;
        }
        case Coord_StopDet:
        {
            if(m_DrawStopRects && m_FrameRecevied)
            {
                for(int i=0; i<4; ++i)
                {
                    cv::Rect tmp = cv::Rect(_msg.X1[i], _msg.Y1[i], _msg.X2_Width[i], _msg.Y2_Height[i] );
                    m_StopRectsVec.push_back(tmp);
                }
                m_DisplayIt[1] = Visualizer::drawDetected(m_StopRectsVec, Red, "STOP", Lilac);
            }
            break;
        }
    }
}
