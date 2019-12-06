#ifndef BACHELOR_DATAPROTOCOL_BOOLDATARECEIVER_HPP_
#define BACHELOR_DATAPROTOCOL_BOOLDATARECEIVER_HPP_

#include "IBoolDataReceiver.hpp"
#include <string>
#include <memory>

//receives bool type of message from specific topic

class BoolDataReceiver : public IBoolDataReceiver
{    
   	//PIMPL //hidden ROS
    class ImplDataReceiver;
    std::unique_ptr<ImplDataReceiver> m_PimplDataReceiver;
public:
    BoolDataReceiver(const std::string _topicName);
    virtual ~BoolDataReceiver();   

    virtual void registerObserver(IBoolObserver *observer, Topics _subjTopic) override;
	virtual void removeObserver(IBoolObserver *observer) override;
};

#endif //BACHELOR_DATAPROTOCOL_BOOLDATARECEIVER_HPP_

