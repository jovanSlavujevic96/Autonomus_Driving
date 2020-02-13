#include <bachelor/DrawAndDisplay/DrawAndDisplay.hpp>
#include <cv_bridge/cv_bridge.h>
#include <bachelor/DataSender/DataSender.hpp>

/*
#include <mutex>
#include <condition_variable>

static std::mutex m;
static std::condition_variable condVar;
*/

bool DrawAndDisplay::pauseCheck(void)
{
    auto btn = cv::waitKey(1);
    if(btn == 'p' || btn == 32) //<space> asci val -> 32
    {
        m_Ignore = false;
        if(m_Pause)
        {
            m_Pause = false;
        }
        else
        {
            m_Pause = true;
        }
        return true;
    }
    else if(btn == 'q' || btn == 27) //<esc> asci val -> 27
    {
        return false;
    }
    m_Ignore = true;
    return true;
}

void DrawAndDisplay::drawDetectedLanes(void)
{
    //std::unique_lock<std::mutex> lk(m);
	//condVar.wait(lk, [this] { return m_LanesVecSizeIs4 ;} );
    if(m_LanesVec.size() != 4)
    {
        return;
    }
    
    bool rightLine=false, leftLine=false;
    //if there are lines draw them
    if( (m_LanesVec[0][0].x != 0 && m_LanesVec[0][0].y != 0) )
    {
        rightLine = true;
        cv::line(m_Frame, m_LanesVec[0][0], m_LanesVec[0][1], cv::Scalar(0, 255, 255), 5, CV_AA);
    }
    if( (m_LanesVec[1][0].x != 0 && m_LanesVec[1][0].y != 0) )
    {
        leftLine = true;
        cv::line(m_Frame, m_LanesVec[1][0], m_LanesVec[1][1], cv::Scalar(0, 255, 255), 5, CV_AA);
    }
    if(rightLine && leftLine)
    {
        cv::line(m_Frame, m_LanesVec[2][0], m_LanesVec[2][1], cv::Scalar(0, 255, 255), 3, CV_AA); //measuredDot (Line)
        cv::line(m_Frame, m_LanesVec[3][0], m_LanesVec[3][1], cv::Scalar(0, 0, 255), 3, CV_AA); //referent Dot (line)

        cv::Mat tmp = m_Frame.clone();
        std::vector<cv::Point> poly_points;
        poly_points.push_back(m_LanesVec[0][0]);
        poly_points.push_back(m_LanesVec[0][1]);
        poly_points.push_back(m_LanesVec[1][1]);
        poly_points.push_back(m_LanesVec[1][0]);
        cv::fillConvexPoly(tmp, poly_points, cv::Scalar(0, 0, 255), CV_AA, 0);
        cv::addWeighted(tmp, 0.3f, m_Frame, 1.0f - 0.3f, 0, m_Frame);
    }
}

DrawAndDisplay::DrawAndDisplay(bool DrawLanes) : 
    m_PauseSender{std::make_unique<DataSender<std_msgs::Bool>>(fromDISPtoCAM)},
    m_ToWatchdog{std::make_unique<DataSender<std_msgs::Bool>>(fromDISPtoWDOG)},
    m_DrawLanes{DrawLanes}
{
    this->m_FrameRecStarted = false;
    this->m_Ignore = false;
    this->m_Pause = false;
    //this->m_LanesVecSizeIs4 = false;
}

bool DrawAndDisplay::doStuff(void) 
{
    m_LanesVec.clear();
    {   
        //msg instance dissapears out of scope
        std_msgs::Bool msg;
        msg.data = true;
        m_ToWatchdog->Publish(msg);
    }
    if(!DrawAndDisplay::pauseCheck() )	//if exit button (<esc> or 'q') is pressed exit the program
	{
        return false;
    }
    if(!m_Ignore)	//if (<space> or 'p') pressed -> m_Ignore = false -> send pause or start
    {
        std_msgs::Bool msg;
        msg.data = m_Pause;
        m_PauseSender->Publish(msg); //send pause or start to publisher (VideoPlayerNode)
    }
    return true;
}

void DrawAndDisplay::update(sensor_msgs::Image &_data, Topics _subjTopic) 
{
    if(_subjTopic == fromCAMtoOBJDET)
    {
        m_FrameRecStarted = true;
        m_Frame = cv_bridge::toCvCopy(_data, "bgr8")->image.clone();
    }
}

void DrawAndDisplay::update(bachelor::Coordinates &_data, Topics _subjTopic) 
{
    if(_subjTopic == fromOBJDETtoDRAW && m_DrawLanes)
    {
        for(int i=0; i<4; ++i)
        {
            cv::Point points[2] = {cv::Point(_data.X1[i], _data.Y1[i]), cv::Point(_data.X2_Width[i], _data.Y2_Height[i]) };
            m_LanesVec.push_back(points);
        }
        std::cout << m_LanesVec[0][0] << std::endl;
        DrawAndDisplay::drawDetectedLanes();
    }
    try
    {
        cv::imshow("detection", m_Frame);
    }
    catch(cv::Exception &error) //there are no more frames // video ended
    {
        return;// false;
    }
}
