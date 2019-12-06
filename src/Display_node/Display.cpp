#include <bachelor/Dipslay_node/Display.hpp>
#include <bachelor/DataProtocol/BoolDataEmiter.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

Display::Display() : m_PauseBtnEmiter{std::make_unique<BoolDataEmiter>(TopicName(fromDISPtoVIDEOP) )}
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

void Display::update(sensor_msgs::Image &_frame, Topics _subjTopic)
{
    if(_subjTopic == fromOBJDETtoDISP)
    {
        cv::Mat picMat = cv_bridge::toCvCopy(_frame, "bgr8")->image;
		cv::imshow("video stream", picMat );
    }
}

void Display::Keyboard(void)
{
    Display::checkIfPressed();	//check is <space> or <esc> pressed 
	if(!m_Ignore)	//if <space> pressed send pause or start
			m_PauseBtnEmiter->Publish(m_Pause); //send pause or start to publisher (VideoPlayerNode)
}