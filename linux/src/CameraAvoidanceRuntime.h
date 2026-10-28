#pragma once
#include "CameraPlanWorker.h"
namespace CameraAvoidanceRuntime {
bool enabled();
bool supported();
void setEnabled(bool value);
void idle();
void update(CameraAvoidance::Pose pose,CameraAvoidance::Point goal,double desired,
            const CameraAvoidance::MapSnapshot& map,
            float& linear,float& angular,const CameraAvoidance::Point* lineStart=nullptr);
const std::vector<CameraAvoidance::Obstacle>& obstacles();
bool active();
bool discardTarget();
double deviation();
bool targetBlocked(CameraAvoidance::Point target);
}
