#ifndef BACHELOR_VISUALIZER_VISUALIZER_HPP_
#define BACHELOR_VISUALIZER_VISUALIZER_HPP_

#include <bachelor/Coordinates.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>

#include <bachelor/IObserver.hpp>
#include <bachelor/DataSender/IDataSender.hpp>

#include <memory>
#include <opencv2/opencv.hpp>

template class IObserver<sensor_msgs::Image>;
template class IObserver<bachelor::Coordinates>;
class Visualizer : public IObserver<sensor_msgs::Image>, public IObserver<bachelor::Coordinates>
{
    std::unique_ptr<IDataSender<std_msgs::Bool>> m_PauseSender, m_ToWatchdog;
    std::vector<std::vector<cv::Point>> m_LanePointsVec;
    std::vector<cv::Rect> m_StopRectsVec;
    cv::Mat m_Frame;
    bool m_Pause, m_Ignore;
    const bool m_DrawLanes, m_DrawStopRects, m_DrawLimitRects;
    bool m_DisplayIt[3], m_FrameRecevied, m_CoordsReceived;  

    bool pauseCheck(void);
    bool drawDetected(void);    //for Lanes
    bool drawDetected(const std::vector<cv::Rect> &contours, const cv::Scalar &contourColor, const std::string sign, const cv::Scalar &textColor);  //for STOP Or Limit sign
    void resetVariables(void);
public:
    Visualizer(bool DrawLanes, bool DrawStopRects, bool DrawLimitRects);
    virtual ~Visualizer() = default;

    bool doStuff(void) override;
    void update(const sensor_msgs::Image &_msg, Topics _subjTopic) override;
    void update(const bachelor::Coordinates &_msg, Topics _subjTopic) override;
};

#endif //BACHELOR_VISUALIZER_VISUALIZER_HPP_