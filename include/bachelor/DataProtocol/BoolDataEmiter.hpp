#ifndef BACHELOR_DATAPROTOCOL_BOOLDATAEMITER_HPP_
#define BACHELOR_DATAPROTOCOL_BOOLDATAEMITER_HPP_

#include "IBoolDataEmiter.hpp"

#include <memory>
#include <string>

//sends bool type of message to specific topic
class BoolDataEmiter : public IBoolDataEmiter
{
	//PIMPL //hidden ROS
	class ImplDataEmiter;
	std::unique_ptr<ImplDataEmiter> m_PimplDataEmiter;

public:
	BoolDataEmiter(const std::string _topicName);
	virtual ~BoolDataEmiter();

	virtual void Publish(const bool data) override;
};

#endif //BACHELOR_DATAPROTOCOL_BOOLDATAEMITER_HPP_