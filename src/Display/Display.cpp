#include <bachelor/Display/Display.hpp>
#include <bachelor/DataSender/DataSender.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

Display::Display() : 
    m_PauseBtnEmiter{std::make_unique<DataSender<std_msgs::Bool>>(fromDISPtoCAM) },
    m_WatchdogEmiter{std::make_unique<DataSender<std_msgs::Bool>>(fromDISPtoWDOG) }
{
    this->m_Ignore = true;
    this->m_Pause = false;
}

void Display::checkIfPressed(void)
{
    char btn = cv::waitKey(1);
    if(btn == 32) // <space> 
    {
        std::cout << "<space> button pressed. "; 
        m_Ignore = false;
        if(m_Pause)
        {
            std::cout << "Play video." << std::endl;
            m_Pause = false;
        }
        else
        {
            std::cout << "Pause video." << std::endl;
            m_Pause = true;
        }
        return;
        
    }
    else if(btn == 27) // <esc>
    {
        std::cout << "<esc> button pressed. Node will exit." << std::endl;
        std::exit(EXIT_SUCCESS);
    }
    m_Ignore = true;
}

void Display::update(sensor_msgs::Image &_data, Topics _subjTopic)
{
    if(_subjTopic == fromOBJDETtoDISP)
    {
        cv::Mat picMat = cv_bridge::toCvCopy(_data, "bgr8")->image;
		cv::imshow("video stream", picMat );
    }
}

bool Display::doStuff(void)
{
    {
        std_msgs::Bool msg;
        msg.data = true;
        m_WatchdogEmiter->Publish(msg);
    }
    Display::checkIfPressed();	//check is <space> or <esc> pressed 
	if(!m_Ignore)	//if <space> pressed -> m_Ignore = false -> send pause or start
    {
        std_msgs::Bool msg;
        msg.data = m_Pause;
        m_PauseBtnEmiter->Publish(msg); //send pause or start to publisher (VideoPlayerNode)
    }
    return true;
}