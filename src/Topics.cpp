#include <bachelor/Topics.h>

#define NumOfTopics 9

static const std::string TopicNames[NumOfTopics] =
{
    "/fromDISPtoVIDEOP", "/fromOBJDETtoVIDEOP", "/fromOBJDETtoWDOG", "/fromOBJDETtoDISP", 
    "/fromVIDEOPtoOBJDET", "/fromVIDEOPtoTIMER", "/fromVIDEOPtoWDOG", "/fromTIMERtoVIDEOP", "/fromTIMERtoWDOG" 
};

std::string TopicName(Topics _whichTopic)
{
    for(int i=0; i<NumOfTopics; ++i)
        if(i == _whichTopic)
            return TopicNames[i];
}