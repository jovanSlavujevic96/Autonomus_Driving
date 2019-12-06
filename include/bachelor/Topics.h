#ifndef BACHELOR_TOPICS_H
#define BACHELOR_TOPICS_H

#include <string>

#define NumOfTopics 9

enum Topics
{
    fromDISPtoVIDEOP, fromOBJDETtoVIDEOP, fromOBJDETtoWDOG, fromOBJDETtoDISP, 
    fromVIDEOPtoOBJDET, fromVIDEOPtoTIMER, fromVIDEOPtoWDOG, fromTIMERtoVIDEOP, fromTIMERtoWDOG 
};

const std::string TopicName[NumOfTopics] =
{
    "/fromDISPtoVIDEOP", "/fromOBJDETtoVIDEOP", "/fromOBJDETtoWDOG", "/fromOBJDETtoDISP", 
    "/fromVIDEOPtoOBJDET", "/fromVIDEOPtoTIMER", "/fromVIDEOPtoWDOG", "/fromTIMERtoVIDEOP", "/fromTIMERtoWDOG" 
};

#endif //BACHELOR_TOPICS_H