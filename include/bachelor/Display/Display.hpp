#ifndef BACHELOR_DISPLAY_DISPLAY_HPP_
#define BACHELOR_DISPLAY_DISPLAY_HPP_

#include <bachelor/Coordinates.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>

#include <bachelor/IObserver.hpp>
#include <bachelor/DataSender/IDataSender.hpp>
#include <bachelor/Visualizer/IVisualizer.hpp>

#include <memory>
#include <opencv2/opencv.hpp>

template class IObserver<sensor_msgs::Image>;
template class IObserver<bachelor::Coordinates>;
class Display : public IObserver<sensor_msgs::Image>, public IObserver<bachelor::Coordinates>
{
    std::unique_ptr<IDataSender<std_msgs::Bool>> m_PauseSender, m_ToWatchdog;
    bool m_Pause, m_Ignore;

    std::vector<IVisualizer*> m_Visualizers;

    cv::Mat m_Frame;
    bool m_FrameRecevied;  

    bool pauseCheck(void);
public:
    Display();
    virtual ~Display() = default;

    void addVisualizer(IVisualizer* _visualizer);

    bool doStuff(void) override;
    void update(const sensor_msgs::Image &_msg, Topics _subjTopic) override;
    void update(const bachelor::Coordinates &_msg, Topics _subjTopic) override;
};

#endif //BACHELOR_DISPLAY_DISPLAY_HPP_