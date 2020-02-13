#ifndef BACHELOR_DISPLAY_DISPLAY_HPP_
#define BACHELOR_DISPLAY_DISPLAY_HPP_

#include <memory>

#include <bachelor/IObserver.hpp>
#include <bachelor/DataSender/IDataSender.hpp>

#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>

template class IObserver<sensor_msgs::Image>;
class Display : public IObserver<sensor_msgs::Image>
{
    std::unique_ptr<IDataSender<std_msgs::Bool> > m_PauseBtnEmiter, m_WatchdogEmiter;
    bool m_Pause, m_Ignore;

    void checkIfPressed(void);
public:
    Display();
    virtual ~Display() = default;

    void update(sensor_msgs::Image &_data, Topics _subjTopic) override;
    bool doStuff(void) override;
};

#endif //BACHELOR_DISPLAY_DISPLAY_HPP_