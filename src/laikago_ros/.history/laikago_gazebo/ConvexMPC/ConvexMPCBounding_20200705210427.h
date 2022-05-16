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
    Vec3<double> f_ff[4];
    Vec4<double> swingTimes;
    FootSwingTrajectory<double> FootSwingTrajectories[4];
    Gait Bound;
    Mat3<double> Kp, Kd, Kp_stance, Kd_stance;
    bool firstRun = true;
    bool firstSwing[4];
    double swingTimeRemaining[4];
    double stand_traj[6];
    int current_gait;
    int gaitNumber;
    bool stance; // flag for stance/flight phase

    Vec3<double> world_position_desired;
    Vec3<double> rpy_int;
    Vec3<double> rpy_comp;
    Vec3<double> pFoot[4];
    CMPC_Result result;
    double trajAll[12*36];
}