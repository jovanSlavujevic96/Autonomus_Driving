#ifndef BACHELOR_TOPICS_H
#define BACHELOR_TOPICS_H

#include <string>

#define NumOfTopics 7

enum Topics
{
    fromDISPtoCAM, fromOBJDETtoWDOG, fromOBJDETtoDISP, fromCAMtoOBJDET, 
    fromCAMtoWDOG, fromDISPtoWDOG, fromOBJDETtoDRAW
};

const std::string TopicName[NumOfTopics] =
{
    "/pause/play", "/Im_here/DET", "/processed_frame", "/raw_frame", 
    "/Im_here/CAM", "/Im_here/DISP", "/coordinates"
};

#endif //BACHELOR_TOPICS_H