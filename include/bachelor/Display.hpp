#ifndef BACHELOR_DISPLAY_HPP_
#define BACHELOR_DISPLAY_HPP_

#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <bachelor/Coordinates.h>
#include <bachelor/Log.h>

#include <bachelor/IObserver.hpp>
#include <bachelor/DataSender/IDataSender.hpp>
#include <bachelor/Visualizer/IVisualizer.hpp>

#include <memory>
#include <opencv2/opencv.hpp>

class Display : 
    public IObserver<sensor_msgs::Image>,
    public IObserver<bachelor::Coordinates>,
    public IObserver<bachelor::Log>
{
    std::unique_ptr<IDataSender<std_msgs::Bool>> m_PauseSender, m_ImHere;

    std::map<Topic, IVisualizer*> m_Visualizers;
    std::map<Topic, bool> m_Recievement;
    std::map<Topic, std::vector<std::vector<cv::Point>>> m_Points;
    
    cv::Mat m_Frame;
    bool m_Pause, m_FrameRecevied;
    bachelor::Log m_ECULog;

    bool calculateRecievement(void);
    void resetVariables(void);
    bool pauseCheck(void);
    void assignStringAndPoints(std::vector<std::string>& text, std::vector<std::vector<cv::Point>>& pts, const Topic topic);
public:
    Display();
    virtual ~Display() = default;

    void addVisualizer(IVisualizer* visualizer, const Topic subjTopic);
    
    bool doStuff(void) override;
    void update(const sensor_msgs::Image& msg, const Topic subjTopic) override;
    void update(const bachelor::Coordinates& msg, const Topic subjTopic) override;
    void update(const bachelor::Log& msg, const Topic subjTopic) override;
};

#endif //BACHELOR_DISPLAY_HPP_