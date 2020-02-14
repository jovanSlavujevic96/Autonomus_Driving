#ifndef BACHELOR_TOPICS_H
#define BACHELOR_TOPICS_H

#include <string>

#define NumOfTopics 10

enum Topics
{
    ImHere_CamSim,  ImHere_Visual,    ImHere_LaneDet,      ImHere_StopDet,       ImHere_LimDet,
    PauseOrPlay,    RawFrame,         Coord_LaneDet,       Coord_StopDet,        Coord_LimDet
};

const std::string TopicName[NumOfTopics] =
{
    "/Im_here/CAM", "/Im_here/VISUAL", "/Im_here/LANE_DET", "/Im_here/STOP_DET", "/Im_here/LIM_DET",
    "/pause/play",  "/raw_frame",      "/Coord/LANE_DET",   "/Coord/STOP_DET",   "/Coord/LIM_DET"
};

#endif //BACHELOR_TOPICS_H