#ifndef BACHELOR_IOBSERVER_HPP_
#define BACHELOR_IOBSERVER_HPP_

#include <bachelor/Topics.h>

template <typename T>
class IObserver
{
public:
	explicit IObserver() = default;
	virtual ~IObserver() = default;

	virtual void update(T &_data, Topics _subjTopic) = 0;	//observer method
	virtual bool doStuff(void) = 0;
};

#endif //BACHELOR_IOBSERVER_HPP_