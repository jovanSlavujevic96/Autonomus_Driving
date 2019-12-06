#ifndef BACHELOR_OBSERVER_IBOOLOBSERVER_HPP_
#define BACHELOR_OBSERVER_IBOOLOBSERVER_HPP_

#include <bachelor/Topics.h>

//BoolObserver interface
class IBoolObserver
{
public:
	explicit IBoolObserver() = default;
	virtual ~IBoolObserver() = default;

	virtual void update(bool _data, Topics _subjTopic) = 0;	//observer method
};

#endif //BACHELOR_OBSERVER_IBOOLOBSERVER_HPP_