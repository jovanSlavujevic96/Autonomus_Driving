#ifndef BACHELOR_DATASENDER_IDATASENDER_HPP_
#define BACHELOR_DATASENDER_IDATASENDER_HPP_

#include <bachelor/Topic.h>

template <typename T1>
class IDataSender
{
public:
    explicit IDataSender() = default;
    virtual ~IDataSender() = default;

    virtual void Publish(const T1& data) = 0;
    virtual Topic getTopic(void) const = 0;
};

#endif //BACHELOR_DATASENDER_IDATASENDER_HPP_