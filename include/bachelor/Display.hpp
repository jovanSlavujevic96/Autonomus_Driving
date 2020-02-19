#ifndef BACHELOR_DISPLAY_HPP_
#define BACHELOR_DISPLAY_HPP_

#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>

#include <bachelor/IObserver.hpp>
#include <bachelor/DataSender/IDataSender.hpp>
#include <bachelor/Visualizer/IVisualizer.hpp>

#include <memory>
#include <opencv2/opencv.hpp>

class Display : public IObserver<sensor_msgs::Image>
{
    std::unique_ptr<IDataSender<std_msgs::Bool>> m_PauseSender, m_ToWatchdog;
    std::vector<IVisualizer*> m_Visualizers;
    cv::Mat m_Frame;
    bool m_Pause, m_FrameRecevied;

    bool pauseCheck(void);
public:
    Display();
    virtual ~Display() = default;

    void addVisualizer(IVisualizer* visualizer);

    bool doStuff(void) override;
    void update(const sensor_msgs::Image& msg, Topics subjTopic) override;
};

#endif //BACHELOR_DISPLAY_HPP_