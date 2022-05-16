#ifndef CONVEXMPCBOUNDING_H
#define CONVEXMPCBOUNDING_H

#include "ConvexMPCLocomotion.h"  // include it for MPC gait (may need redefine gait class later)

class ConvexMPCBounding{
  public:
    ConvexMPCBounding(double _dt, int _iterations_between_mpc);

    void run(ControlFSMData& data);
  
  private:
    void updateMPCforStance(int* mpcTable, ControlFSMData& data);
    void updateMPCforFlight(int* mpcTable, ControlFSMData& data);
    int iterationsBetweenMPC;
    int horizonLength;
    double dt;
    double dtMPC;
    int iterationCunter = 0;
    

}