#ifndef BACHELOR_TOPICS_H
#define BACHELOR_TOPICS_H

#include <string>

enum Topics
{
    fromDISPtoVIDEOP, fromOBJDETtoVIDEOP, fromOBJDETtoWDOG, fromOBJDETtoDISP, 
    fromVIDEOPtoOBJDET, fromVIDEOPtoTIMER, fromVIDEOPtoWDOG, fromTIMERtoVIDEOP, fromTIMERtoWDOG 
};

std::string TopicName(Topics _whichTopic);

#endif //BACHELOR_TOPICS_H