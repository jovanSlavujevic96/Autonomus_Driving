#ifndef BACHELOR_DATAPROTOCOL_FRAMEDATARECEIVER_HPP
#define BACHELOR_DATAPROTOCOL_FRAMEDATARECEIVER_HPP

#include "IFrameDataReceiver.hpp"
#include <memory>
#include <string>

//subject
class FrameDataReceiver : public IFrameDataReceiver
{
private:
	//PIMPL //hidden ROS cv_bridge
	class ImplFrameSub;
	std::unique_ptr<ImplFrameSub> m_PimplFrameSub;
	
public:
	FrameDataReceiver(const std::string _topicName); 
	virtual ~FrameDataReceiver();
	
	virtual void registerObserver(IImageObserver *observer, Topics _subjTopic) override;
	virtual void removeObserver(IImageObserver *observer) override;
};

#endif //BACHELOR_DATAPROTOCOL_FRAMEDATARECEIVER_HPP
