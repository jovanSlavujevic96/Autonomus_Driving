#ifndef BACHELOR_IOBSERVER_HPP_
#define BACHELOR_IOBSERVER_HPP_

#include <bachelor/Topic.h>

class IPlatformRcv;

class IObserver
{
public:
	explicit IObserver() = default;
	virtual ~IObserver() = default;

	virtual bool doStuff(void) = 0;
	virtual void update(const IPlatformRcv* receiver) = 0;
};



#endif //BACHELOR_IOBSERVER_HPP_