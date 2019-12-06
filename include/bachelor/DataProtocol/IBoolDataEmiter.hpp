#ifndef BACHELOR_DATAPROTOCOL_IBOOLDATAEMITER_HPP_
#define BACHELOR_DATAPROTOCOL_IBOOLDATAEMITER_HPP_

//Bool Data Emiter interface
class IBoolDataEmiter
{
public:
    explicit IBoolDataEmiter() = default;
    virtual ~IBoolDataEmiter() = default;    

    virtual void Publish(const bool data) = 0;
};

#endif //BACHELOR_DATAPROTOCOL_IBOOLDATAEMITER_HPP_