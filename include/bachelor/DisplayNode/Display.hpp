#ifndef BACHELOR_DISPLAY_NODE_DISPLAY_HPP_
#define BACHELOR_DISPLAY_NODE_DISPLAY_HPP_

#include <bachelor/Observer/IImageObserver.hpp>
#include <bachelor/DataProtocol/IBoolDataEmiter.hpp>

#include <memory>

class Display : public IImageObserver
{
    std::unique_ptr<IBoolDataEmiter> m_PauseBtnEmiter;
    bool m_Pause, m_Ignore;
    void checkIfPressed(void);
public:
    Display();
    virtual ~Display() = default;

    void Keyboard(void);
    virtual void update(sensor_msgs::Image &_frame, Topics _subjTopic) override;
};

#endif //BACHELOR_DISPLAY_NODE_DISPLAY_HPP_