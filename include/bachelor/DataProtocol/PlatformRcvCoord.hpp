#ifndef BACHELOR_DATAPROTOCOL_PLATFORMRCVCOORD_HPP_
#define BACHELOR_DATAPROTOCOL_PLATFORMRCVCOORD_HPP_

#include "PlatformRcv.hpp"
#include <bachelor/Coordinates.h>
#include <bachelor/Message/CoordMessage.hpp>

class PlatformRcvCoord : public PlatformRcv
{
    CoordMessage m_Message;
    void Callback(const bachelor::Coordinates& msg);

public:
    PlatformRcvCoord(const Topic topic);
    virtual ~PlatformRcvCoord() = default;
    
    const IMessage* getMessage(void) const override;
};

#endif //BACHELOR_DATAPROTOCOL_PLATFORMRCVCOORD_HPP_