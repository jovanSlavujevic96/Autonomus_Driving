#ifndef BACHELOR_DATASENDER_DATASENDER_HPP_
#define BACHELOR_DATASENDER_DATASENDER_HPP_

#include "IDataSender.hpp"

#include <memory>
#include <bachelor/Topics.h>

template <typename T1>
class DataSender : public IDataSender <T1>
{   
    class ImplDataSender;
    std::unique_ptr<ImplDataSender> m_PimplDataSender;
    Topics m_Topic;
public:
    DataSender(const Topics _topicName);
    virtual ~DataSender();

    void Publish(T1 _data) override;
    Topics getTopic(void) const override;
};

#endif //BACHELOR_DATASENDER_DATASENDER_HPP_