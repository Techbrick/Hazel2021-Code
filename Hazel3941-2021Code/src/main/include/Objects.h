#pragma once

#include <vector>
#include "frc/trajectory/TrajectoryConfig.h"
#include "frc/trajectory/Trajectory.h"

struct Ball{
    public:
        Ball(float val){
            x = val;
        }
        union{
            float theta;
            float x;
            float deg;
        };
        ~Ball(){}
};