#ifndef BACHELOR_DATAPROTOCOL_FRAMEDATAEMITER_HPP_
#define BACHELOR_DATAPROTOCOL_FRAMEDATAEMITER_HPP_

#include "IFrameDataEmiter.hpp"

#include <memory>
#include <string>

//sends frame to specific topic
class FrameDataEmiter : public IFrameDataEmiter
{
    //PIMPL //hidden ROS
	class ImplFrameEmiter;
	std::unique_ptr<ImplFrameEmiter> m_PimplFrameEmiter;
public:
    FrameDataEmiter(const std::string _topicName);
    virtual ~FrameDataEmiter();

    virtual void Publish(const sensor_msgs::Image &_frame) override;
};

#endif //BACHELOR_DATAPROTOCOL_FRAMEDATAEMITER_HPP_