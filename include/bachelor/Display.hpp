#ifndef BACHELOR_DISPLAY_HPP_
#define BACHELOR_DISPLAY_HPP_

#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <bachelor/Coordinates.h>

#include <bachelor/IObserver.hpp>
#include <bachelor/DataSender/IDataSender.hpp>
#include <bachelor/Visualizer/IVisualizer.hpp>

#include <memory>
#include <opencv2/opencv.hpp>

class Display : 
    public IObserver<sensor_msgs::Image>,
    public IObserver<bachelor::Coordinates>
{
    std::unique_ptr<IDataSender<std_msgs::Bool>> m_PauseSender, m_ToWatchdog;

    std::map<Topics, IVisualizer*> m_Visualizers;
    std::map<Topics, bool> m_Recievement;
    std::map<Topics, std::vector<cv::Point>> m_Points;
    std::vector<Topics> m_TopicsInUse;
    
    cv::Mat m_Frame;
    bool m_Pause, m_FrameRecevied;

    bool calculateRecievement(void);
    void resetVariables(void);
    bool pauseCheck(void);
public:
    Display();
    virtual ~Display() = default;

    void addVisualizer(IVisualizer* visualizer, Topics subjTopic);
    
    bool doStuff(void) override;
    void update(const sensor_msgs::Image& msg, Topics subjTopic) override;
    void update(const bachelor::Coordinates& msg, Topics subjTopic) override;
};

#endif //BACHELOR_DISPLAY_HPP_