#ifndef BACHELOR_DATAPROTOCOL_IDATASENDER_HPP_
#define BACHELOR_DATAPROTOCOL_IDATASENDER_HPP_

template <typename T1, typename T2>
class IDataSender
{
public:
    explicit IDataSender() = default;
    virtual ~IDataSender() = default;

    virtual void Publish(T1 _data) = 0;
};

#endif //BACHELOR_DATAPROTOCOL_IDATASENDER_HPP_