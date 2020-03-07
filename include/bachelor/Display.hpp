#ifndef BACHELOR_DISPLAY_HPP_
#define BACHELOR_DISPLAY_HPP_

#include <bachelor/IObserver.hpp>
#include <bachelor/DataProtocol/ISender.hpp>
#include <bachelor/Visualizer/IVisualizer.hpp>

#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

class Display : 
    public IObserver
{
    std::unique_ptr<ISender> m_PauseSender, m_ImHere;

    std::map<Topic, IVisualizer*> m_Visualizers;
    std::map<Topic, bool> m_Recievement;
    std::map<Topic, std::vector<std::vector<cv::Point>>> m_Points;

    std::vector<std::string> m_Log;
    
    cv::Mat m_Frame;
    bool m_Pause;

    bool calculateRecievement(void);
    void resetVariables(void);
    bool pauseCheck(void);
    void assignStringAndPoints(std::vector<std::string>*& text, std::vector<std::vector<cv::Point>>*& pts, const Topic topic);
    bool drawAndDisplay(void);
public:
    Display();
    virtual ~Display() = default;

    void addVisualizer(IVisualizer* visualizer, const Topic subjTopic);
    bool doStuff(void) override;
    void update(const IPlatformRcv* receiver) override;
};

#endif //BACHELOR_DISPLAY_HPP_