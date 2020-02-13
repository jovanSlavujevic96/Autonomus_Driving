#ifndef BACHELOR_DRAWANDDISPLAY_DRAWANDDISPLAY_HPP_
#define BACHELOR_DRAWANDDISPLAY_DRAWANDDISPLAY_HPP_

#include <bachelor/Coordinates.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>

#include <bachelor/IObserver.hpp>
#include <bachelor/DataSender/IDataSender.hpp>

#include <memory>
#include <opencv2/opencv.hpp>

template class IObserver<sensor_msgs::Image>;
template class IObserver<bachelor::Coordinates>;
class DrawAndDisplay : public IObserver<sensor_msgs::Image>, public IObserver<bachelor::Coordinates>
{
    std::unique_ptr<IDataSender<std_msgs::Bool>> m_PauseSender, m_ToWatchdog;
    std::vector<cv::Point*> m_LanesVec;
    cv::Mat m_Frame;
    bool m_Pause, m_Ignore;
    bool m_FrameRecStarted;  //bool m_LanesVecSizeIs4;
    bool m_DrawLanes;  

    bool pauseCheck(void);
    void drawDetectedLanes(void);
public:
    DrawAndDisplay(bool DrawLanes);
    virtual ~DrawAndDisplay() = default;

    bool doStuff(void) override;
    void update(sensor_msgs::Image &_data, Topics _subjTopic) override;
    void update(bachelor::Coordinates &_data, Topics _subjTopic) override;
};

#endif //BACHELOR_DRAWANDDISPLAY_DRAWANDDISPLAY_HPP_