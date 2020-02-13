#ifndef BACHELOR_DATASENDER_IDATASENDER_HPP_
#define BACHELOR_DATASENDER_IDATASENDER_HPP_

#include <bachelor/Topics.h>

template <typename T1>
class IDataSender
{
public:
    explicit IDataSender() = default;
    virtual ~IDataSender() = default;

    virtual void Publish(T1 _data) = 0;
    virtual Topics getTopic(void) const = 0;
};

#endif //BACHELOR_DATASENDER_IDATASENDER_HPP_