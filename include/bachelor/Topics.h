#ifndef BACHELOR_TOPICS_H
#define BACHELOR_TOPICS_H

#include <string>

#define NumOfTopics 10

enum Topics
{
    fromDISPtoVIDEOP, fromOBJDETtoVIDEOP, fromOBJDETtoWDOG, fromOBJDETtoDISP, fromVIDEOPtoOBJDET, 
    fromVIDEOPtoTIMER, fromVIDEOPtoWDOG, fromTIMERtoVIDEOP, fromTIMERtoWDOG, fromDISPtoWDOG
};

const std::string TopicName[NumOfTopics] =
{
    "/fromDISPtoVIDEOP", "/fromOBJDETtoVIDEOP", "/fromOBJDETtoWDOG", "/fromOBJDETtoDISP", "/fromVIDEOPtoOBJDET", 
    "/fromVIDEOPtoTIMER", "/fromVIDEOPtoWDOG", "/fromTIMERtoVIDEOP", "/fromTIMERtoWDOG", "/fromDISPtoWDOG"
};

#endif //BACHELOR_TOPICS_H