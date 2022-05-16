#ifndef CONVEXMPCBOUNDING_H
#define CONVEXMPCBOUNDING_H

#include "ConvexMPCLocomotion.h"  // include it for MPC gait (may need redefine gait class later)

class ConvexMPCBounding{
  public:
    ConvexMPCBounding(double _dt, int _iterations_between_mpc);

    void run(ControlFSMData& data);

}