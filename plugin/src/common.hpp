#ifndef COMMON_HPP
#define COMMON_HPP

#include <array>

enum PathState
{
    Home,
    ToGrabApproach,
    GrabApproach,
    ToPick,
    Pick,
    Picked,
    ToObstacleApproach,
    ObstacleApproach,
    ToObstacleExit,
    ObstacleExit,
    ToPlace,
    Place,
    Placed,
    ToPlaceExit,
    PlaceExit,
    ToHome
};

struct RobotState
{
    enum PathState path_state = PathState::Home;
};

extern std::array<struct RobotState, 2> robot_states;

#endif
