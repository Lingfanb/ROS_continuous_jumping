#include <iostream>
#include "../include/Utilities/Timer.h"
#include "../include/Math/orientation_tools.h"
#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"
#include "../include/body.h"

using namespace ori;
using namespace laikago_model;

/* ========================= GAIT ========================= */
Gait::Gait(int nMPC_segments, Vec4<int> offsets, Vec4<int> durations, const std::string& name) :
  _offsets(offsets.array()),
  _durations(durations.array()),
  _nIterations(nMPC_segments)
{
    _mpc_table = new int[nMPC_segments * 4];

    _offsetsPhase = offsets.cast<double>() / (double) nMPC_segments;
    _durationsPhase = durations.cast<double>() / (double) nMPC_segments;

    _stance = durations[0];
    _swing = nMPC_segments - durations[0];
}

Gait::~Gait()
{
    delete[] _mpc_table;
}

Vec4<double> Gait::getContactSubPhase()
{
  Array4d progress = _phase - _offsetsPhase;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _durationsPhase[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsPhase[i];
    }
    
  }

  return progress.matrix();
}

Vec4<double> Gait::getSwingSubPhase()
{
  Array4d swing_offset = _offsetsPhase + _durationsPhase;
  for(int i = 0; i < 4; i++)
    if(swing_offset[i] > 1) swing_offset[i] -= 1.;
  Array4d swing_duration = 1. - _durationsPhase;

  Array4d progress = _phase - swing_offset;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > swing_duration[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }

  return progress.matrix();
}

int* Gait::mpc_gait()
{
  for(int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration + 1) % _nIterations;
    Array4i progress = iter - _offsets;
    
    // std::cout << "_nIterations: " << _nIterations << std::endl;
    // std::cout << "_iteration: " << _iteration << std::endl;
    // std::cout << "iter: " << iter << std::endl;
    // std::cout << "_offsets: " << _offsets << std::endl;
    // std::cout << "progress: " << progress<< std::endl;

    for(int j = 0; j < 4; j++)
    {
      if(progress[j] < 0) progress[j] += _nIterations;
      if(progress[j] < _durations[j])
        _mpc_table[i*4 + j] = 1;
      else
        _mpc_table[i*4 + j] = 0;
    }
    
  }
  
  //std::cout << "horizon: " << _iteration << std::endl; 
  std::cout << "--------------------------------------" << std::endl;

  return _mpc_table;
}

void Gait::setIterations(int iterationsPerMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  _phase = (double)(currentIteration % (iterationsPerMPC * _nIterations)) / (double) (iterationsPerMPC * _nIterations);
}

/* =========================== Controller ============================= */
ConvexMPCLocomotion::ConvexMPCLocomotion(double _dt, int _iterations_between_mpc) :
  iterationsBetweenMPC(_iterations_between_mpc),
  horizonLength(10),
  dt(_dt),
  galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(5,5,5,5),"Galloping"),
  pronking(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(4,4,4,4),"Pronking"),
  trotting(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),
  bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(5,5,5,5),"Bounding"),
  walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
  pacing(horizonLength, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing")
{
  gaitNumber = 1;
  dtMPC = dt * iterationsBetweenMPC;
  //std::cout << "dtMPC: " << dtMPC << std::endl;
  rpy_int[2] = 0;
  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;

  foot_position.open("foot_pos.txt");

  //------------ Read the optimization data from imported files --------------------
  // ------------Stored the data into vectors --------------------------------------

    
    // Read the joint velocity and position from optimization data
    std::ifstream myFile;
    myFile.open("src/laikago_ros/optimization_data/jump2D/data_Q.csv");

    // If cannot open the file, report an error
    if(!myFile.is_open())
    {
        std::cout << "Could not open file for position and velocity" << std::endl;
        //return 0;
    }
    cout << "Reading Optimization Data for Position and Velocity" << endl;
    string line;
    int index = 0;

    while(getline(myFile, line))
    {
                stringstream ss(line);
                double val;
                vector<double> Q;
                while(ss >> val)
                {
                    Q.push_back(val);
                    if(ss.peek() == ',') ss.ignore();
                }
                QDes.push_back(Q);
    }
    myFile.close();

    // Read the joint velocity and position from optimization data
    std::ifstream mytauFile("src/laikago_ros/optimization_data/jump2D/data_tau.csv");

    // If cannot open the file, report an error
    if(!mytauFile.is_open())
    {
        std::cout << "Could not open file for torque" << endl;
        //return 0;
    }
    cout << "Reading Optimization Data for Torque" << endl;
    
    index = 0;
    while(getline(mytauFile, line))
    {
        stringstream ss(line);
        double val;
        vector<double> tau_joint;
        while(ss >> val)
        {
            tau_joint.push_back(val/2.0);
            if(ss.peek() == ',') ss.ignore();
        }
        tauDes.push_back(tau_joint);
    }

    mytauFile.close();

        // Read the joint velocity and position from optimization data
    std::ifstream myFFile("src/laikago_ros/optimization_data/jump2D/data_F.csv");

    // If cannot open the file, report an error
    if(!myFFile.is_open())
    {
        std::cout << "Could not open file for force" << endl;
        //return 0;
    }
    cout << "Reading Optimization Data for force" << endl;
    
    index = 0;
    while(getline(myFFile, line))
    {
        stringstream ss(line);
        double val;
        vector<double> F_ref;
        while(ss >> val)
        {
            F_ref.push_back(val);
            if(ss.peek() == ',') ss.ignore();
        }
        FDes.push_back(F_ref);
    }

    myFFile.close();

}


void ConvexMPCLocomotion::run(ControlFSMData& data)
{
  bool omniMode = false;


//  auto* debugSphere = data.visualizationData->addSphere();
//  debugSphere->color = {1,1,1,0.5};
//  debugSphere->radius = 1;
 // data._legController->updateData();
 // data._stateEstimator->run();
  auto& seResult = data._stateEstimator->getResult();
  auto& stateCommand = data._desiredStateCommand;
  //std::cout << "in side mpc" << seResult.rBody << std::endl;;

  // pick gait
  Gait* gait = &trotting;
  if(gaitNumber == 1)
    gait = &bounding;
  else if(gaitNumber == 2)
    gait = &trotting;
  else if(gaitNumber == 3)
    gait = &walking;
  else if(gaitNumber == 4)
    gait = &pacing;
  else if(gaitNumber == 5)
    gait = &galloping;
  else if(gaitNumber == 6)
    gait = &pronking;
  current_gait = gaitNumber;
  // integrate position setpoint
  Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
  Vec3<double> v_des_world;
  //v_des_world = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]).transpose() * v_des_robot;
  //Vec3<double> v_robot = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2])*seResult.vWorld;

  v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<double> v_robot = seResult.vWorld;

  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];
  world_position_desired[2] = 0.3;
  
 // printf("p_des \t%.6f\n", dt * v_des_world[0]);
 //Integral-esque pitch and roll compensation
  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(stateCommand->data.stateDes[4] /*-hw_i->state_estimator->se_ground_pitch*/ - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(stateCommand->data.stateDes[3] /*-hw_i->state_estimator->se_ground_pitch*/
        - seResult.rpy[0])/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=6);  //turn off for pronking

  


  for(int i = 0; i < 4; i++) {
   pFoot[i] = seResult.position + seResult.rBody.transpose() 
            * (data._quadruped->getHipLocation(i) + data._legController->data[i].p);
   // pFoot[i] = data._legController->data[i].p;
    //std::cout << "pFoot" << i << "\n" << pFoot[i] << std::endl;
  }
  
  if(climb){
    bool allContact = true;
    for(int i = 0; i < 4; i++){
      if(lowState.footForce[i] < 3 ){
        allContact = false;
      }
    }
  if(iterationCounter % (iterationsBetweenMPC * horizonLength) == 0){
    for(int i = 0; i < 4; i++)
        {
          W.row(i) << 1, pFoot[i][0], pFoot[i][1];
          pz[i] = pFoot[i][2];
         // if(i != 0) W.row(i) << 0, pFoot[i][0], pFoot[i][1];
        }
      a = W.transpose() * W * (W.transpose()* W * W.transpose()*W).inverse()*  W.transpose() * pz;
      ground_pitch = acos(-1/sqrt(a[1]*a[1] + a[2]*a[2] +1)) - 3.14;
      //std::cout << "ground pitch: " << ground_pitch << std::endl;
      if(pz[0] < pz[2]){
        ground_pitch = -ground_pitch;
      }
      if(abs(pz[0] - pz[2]) < 0.01){
        ground_pitch = 0;
      }
    }
  }

  if(climb){
    if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(stateCommand->data.stateDes[4] - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(stateCommand->data.stateDes[3]
        - seResult.rpy[0])/v_robot[1];
  }
  }

  // some first time initialization
  if(firstRun)
  {
    std::cout << "Run MPC" << std::endl;
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];

    Vec3<double> v_des_robot(0,0,0);  // connect to desired state command later
    Vec3<double> v_des_world(0,0,0);  // connect to desired state command later

    Vec3<double> v_robot = seResult.vWorld;
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = world_position_desired[2];
    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0;

    pBody_RPY_des[0] = 0;
    pBody_RPY_des[1] = 0;
    pBody_RPY_des[2] = 0; // seResult.rpy[2];

    vBody_Ori_des[0] = 0;
    vBody_Ori_des[1] = 0;
    vBody_Ori_des[2] = 0; // set this for now

    for(int i = 0; i < 4; i++){
      footSwingTrajectories[i].setHeight(0.1);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      //std::cout << "orig foot pos " << i << pFoot[i] << std::endl;
    }
    firstRun = false;
  }
  
  // Update For WBC
  // pBody_des[0] = world_position_desired[0];
  // pBody_des[1] = world_position_desired[1];
  // pBody_des[2] = 0.4;

  // vBody_des[0] = v_des_world[0];
  // vBody_des[1] = v_des_world[1];
  // vBody_des[2] = 0.;

  // pBody_RPY_des[0] = 0.;
  // pBody_RPY_des[1] = 0.; 
  // pBody_RPY_des[2] = 0.; // stateCommand->data.stateDes[5];

  // vBody_Ori_des[0] = 0.;
  // vBody_Ori_des[1] = 0.;
  // vBody_Ori_des[2] = 0.; // stateCommand->data.stateDes[11];

  contact_state = gait->getContactSubPhase();

 
   // std::cout << "foot force 0 :" << lowState.footForce[0] << std::endl;
   // std::cout << "contact 0 : " << contact_state[0] << std::endl;
  
  // foot placement
  swingTimes[0] = dtMPC * gait->_swing;
  swingTimes[1] = dtMPC * gait->_swing;
  swingTimes[2] = dtMPC * gait->_swing;
  swingTimes[3] = dtMPC * gait->_swing;

  //std::cout << "Swing Time: " << swingTimes[0] << std::endl; 

  //std::cout << "Swing Time" << swingTimes << std::endl;
  double side_sign[4] = {-1, 1, -1, 1};
  double interleave_y[4] = {-0.08, 0.08, 0.01, -0.01};
  double interleave_gain = -0.2;
  double v_abs = std::fabs(seResult.vBody[0]);
  for(int i = 0; i < 4; i++)
  {

    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }

  
    if(firstSwing[i]) {
      
      footSwingTrajectories[i].setHeight(.1);
      Vec3<double> offset(0, side_sign[i] * data._quadruped->hipLinkLength, 0);
      // simple heuristic function
      //std::cout << "swing time" << swingTimeRemaining[i] << std::endl;
      
      Vec3<double> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);
      Vec3<double> pYawCorrected = coordinateRotation(CoordinateAxis::Z, -stateCommand->data.stateDes[11] * gait->_stance * dtMPC / 2) * pRobotFrame;
 
      Vec3<double> des_vel;
      
      des_vel[0] = stateCommand->data.stateDes(6);
      des_vel[1] = stateCommand->data.stateDes(7);
      des_vel[2] = stateCommand->data.stateDes(8);

      Vec3<double> Pf = seResult.position +
                       seResult.rBody.transpose() * (pYawCorrected
                       + des_vel * swingTimeRemaining[i]);

      //+ seResult.vWorld * swingTimeRemaining[i];

      double p_rel_max = 0.4;
      double pfx_rel = seResult.vWorld[0] * .5 * gait->_stance * dtMPC +
                      .03*(seResult.vWorld[0]-v_des_world[0]) +
                      (0.5*seResult.position[2]/9.81) * (seResult.vWorld[1]*stateCommand->data.stateDes[11]);
      double pfy_rel = seResult.vWorld[1] * .5 * gait->_stance * dtMPC +
                      .03*(seResult.vWorld[1]-v_des_world[1]) +
                      (0.5*seResult.position[2]/9.81) * (-seResult.vWorld[0]*stateCommand->data.stateDes[11]);
      pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
      pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
      Pf[0] +=  pfx_rel;
      Pf[1] +=  pfy_rel + interleave_y[i] * v_abs * interleave_gain;
      Pf[2] = -0.01;
      //Pf[2] = 0;// pFoot[i][2];
      if(climb){
      Pf[2] = pFoot[i][2];
      //a[0] + a[1] * Pf[0] + a[2] * Pf[1] - 0.02;
      }
      footSwingTrajectories[i].setFinalPosition(Pf);
    }
    

  }




  // calc gait
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  
  // load LCM leg swing gains
  Kp << 500, 0, 0,
      0, 500, 0,
      0, 0, 350;
  Kp_stance = 0*Kp;

  Kd << 30, 0, 0,
      0, 30, 0,
      0, 0, 30;
  Kd_stance = Kd;
  // gait
  Vec4<double> contactStates = gait->getContactSubPhase();
  Vec4<double> swingStates = gait->getSwingSubPhase();
  int* mpcTable = gait->mpc_gait();

  updateMPCIfNeeded(mpcTable, data, omniMode);
  
  iterationCounter++;

//  StateEstimator* se = hw_i->state_estimator;
  Vec4<double> se_contactState(0,0,0,0);
  // for(int i = 0; i < 4; i++){
  //   footSwingTrajectories[i].setHeight(0.1);
  //   footSwingTrajectories[i].setInitialPosition(pFoot[i]);
  //   footSwingTrajectories[i].setFinalPosition(pFoot[i]);
  // }

  for(int foot = 0; foot < 4; foot++)
  {
  
    double contactState = contactStates(foot);
    double swingState = swingStates(foot);

    //std::cout << "swing" << foot << ": " << swingState << std::endl;
    if(swingState > 0) // foot is in swing
    {
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);   
        //footSwingTrajectories[foot].setHeight(0.1);
      }
 
      //std::cout << "swing" << foot << ": " << swingState << std::endl;
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

       Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
       Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
       Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
       Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      // Vec3<double> pDesLeg = footSwingTrajectories[foot].getPosition();
      // Vec3<double> vDesLeg = footSwingTrajectories[foot].getVelocity();
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";
        data._legController->commands[foot].feedforwardForce << 0, 0, 0;
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;
        //std::cout << "foot Des world" << foot << ": \n " << pDesFootWorld(2) << std::endl; 
        //std::cout << "foot Des " << foot << ": \n " << pDesLeg(2) << std::endl; 
        //singularity barrier
        //data._legController->commands[foot].tau[2] = 
        //  50*(data._legController->data[foot].q(2)<.1)*data._legController->data[foot].q(2);
       // std::cout << "contat " << foot << ": " << contactState << std::endl;
       if(foot == 0){
         foot_position << pDesFootWorld[0] << " " << pDesFootWorld[1] << " " << pDesFootWorld[2] << "\n";
       }
      if(climb){
        if(lowState.footForce[foot] > 10 && swingState>0.5){
          //std::cout << "force changed for leg " << foot << std::endl;
          data._legController->commands[foot].kpCartesian = Kp_stance;
          data._legController->commands[foot].kdCartesian = 0 * Kd;
          data._legController->commands[foot].feedforwardForce << 0, 0, -10;
          contactState = 0;
          firstSwing[foot] = true;
        }
      }
      se_contactState[foot] = contactState;

      
    }
  
    else if(contactState > 0) // foot is in stance
    {
      firstSwing[foot] = true;
       footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
       Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition().cast<double>();
       Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity().cast<double>();
       Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
       Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      // Vec3<double> pDesLeg = footSwingTrajectories[foot].getPosition();
      // Vec3<double> vDesLeg = footSwingTrajectories[foot].getVelocity();
       //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";
       //std::cout << "robot pos: \n" << seResult.position << std::endl;
       //foot_swing << pDesFootWorld[2] << " ";
         data._legController->commands[foot].pDes = pDesLeg;
         data._legController->commands[foot].vDes = vDesLeg;
         data._legController->commands[foot].kpCartesian = Kp_stance; // 0
         data._legController->commands[foot].kdCartesian = Kd_stance;
       
        data._legController->commands[foot].feedforwardForce = f_ff[foot];
        //std::cout << "y: " << pDesLeg[1] << std::endl;
        //std::cout << "contact " << foot << " : \n" << contactState << std::endl;
        //std::cout << "foot force " << foot << " : \n " << data._legController->commands[foot].feedforwardForce(2) << std::endl; 
        //data._legController->commands[foot].kdJoint = Mat3<double>::odyIdentity() * 0.2
      
      //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
      se_contactState[foot] = contactState;

      // Update for WBC
      //Fr_des[foot] = -f_ff[foot];
    
      //foot_swing << "\n";
    // std::cout << "foot Des" << foot << " \n " << data._legController->commands[foot].pDes << std::endl;
    }
  
  // se->set_contact_state(se_contactState); todo removed
  data._stateEstimator->setContactPhase(se_contactState);
  //data._legController->updateCommand();

  }
}



void ConvexMPCLocomotion::updateMPCIfNeeded(int* mpcTable, ControlFSMData& data, bool omniMode) 
{
  //iterationsBetweenMPC = 30;
  //std::cout << "iterationCounter: "<< iterationCounter << std::endl;
  //std::cout << "MPC iteration: ";
  //std::cout << iterationCounter % iterationsBetweenMPC << std::endl;
  //data._legController->updateData();
  if((iterationCounter % iterationsBetweenMPC) == 0)
  {
    // std::cout << "state est: ";
  
    // for(int i = 0; i < 3; i++){
    //   std::cout << data._stateEstimator->getResult().rpy(i) << " " ; 
    //  }
    //  std::cout << endl;
   // std::cout << "runMPC" << std::endl;
    auto seResult = data._stateEstimator->getResult();
    auto& stateCommand = data._desiredStateCommand;

    double* p = seResult.position.data();
    double* v = seResult.vWorld.data();
    double* w = seResult.omegaWorld.data();
    double* q = seResult.orientation.data();

    // float* pf = (float*) seResult.position.data();
    // float* vf = (float*) seResult.vWorld.data();
    // float* wf = (float*) seResult.omegaWorld.data();
    // float* qf = (float*) seResult.orientation.data();

    double r[12];
    for(int i = 0; i < 12; i++)
      r[i] = pFoot[i%4][i/4] - seResult.position[i/4];

    double Q[12] = {0.25, 0.25, 50, 2.5, 2.5, 30, 0, 0, 0.3, 0.2, 0.2, 0.2};
    double yaw = seResult.rpy[2];
    double* weights = Q;
    double alpha = 1e-5; // make setting eventually

    //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

    if(alpha > 1e-4)
    {

      std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
      alpha = 1e-5;
    }
    Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
    //Vec3<double> v_des_world = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]).transpose() * v_des_robot;

    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;
    //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

    // if(current_gait == 4)
    // {
    //   double trajInitial[12] = {0, //(double)stateCommand->data.stateDes[3],
    //                             0, // (double)stateCommand->data.stateDes[4] /*-hw_i->state_estimator->se_ground_pitch*/,
    //                             seResult.rpy(2), // (double)stand_traj[5]/*+(float)stateCommand->data.stateDes[11]*/,
    //                             0, //(double)stand_traj[0]/*+(float)fsm->main_control_settings.p_des[0]*/,
    //                             0, //(double)stand_traj[1]/*+(float)fsm->main_control_settings.p_des[1]*/,
    //                            0.4/*fsm->main_control_settings.p_des[2]*/,
    //                            0,0,0,0,0,0};

    //   for(int i = 0; i < horizonLength; i++)
    //     for(int j = 0; j < 12; j++)
    //       trajAll[12*i+j] = trajInitial[j];
    // }

    // else
     //{
      //world_position_desired = seResult.position;
      const double max_pos_error = .1;
      double xStart = world_position_desired[0];
      double yStart = world_position_desired[1];
      //std::cout << "orig " << xStart << "  " << yStart << std::endl;
      //printf("orig \t%.6f\t%.6f\n", xStart, yStart);
      //printf("ref: \t%.6f\t%.6f\n", p[0], p[1]);

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      //printf("xys: \t%.3f\t%3.f\n", xStart, yStart);
      //printf("perr \t%.3f\t%.3f\n", p[0] - world_position_desired[0], p[1] - world_position_desired[1]);
      
      double trajInitial[12] = {rpy_comp[0] + stateCommand->data.stateDes[3],  // 0
                                rpy_comp[1] + stateCommand->data.stateDes[4],    // 1
                                stateCommand->data.stateDes[5],    // 2
                                xStart,                                   // 3
                                yStart,                                   // 4
                                0.3 ,   // 5
                                0,                                        // 6
                                0,                                        // 7
                                stateCommand->data.stateDes[11],  // 8
                                v_des_world[0],                           // 9
                                v_des_world[1],                           // 10
                                0};                                       // 11
      if(climb){
        trajInitial[1] = ground_pitch; 
        // trajInitial[5] += a[0] + a[1] * xStart + a[2] * yStart;
      }
      // double trajInitial[12] = {rpy_comp[0],  // 0
      //                           rpy_comp[1],    // 1
      //                           stateCommand->data.stateDes[5],    // 2
      //                           xStart,                                   // 3
      //                           yStart,                                   // 4
      //                           0.4,      // 5
      //                           0,                                        // 6
      //                           0,                                        // 7
      //                           stateCommand->data.stateDes[11],  // 8
      //                           v_des_world[0],                           // 9
      //                           v_des_world[1],                           // 10
      //                           0};                                       // 11

      // float trajInitial[12] = {(float)0., // 0
      //                          (float)0., // 1
      //                          (float)stateCommand->data.stateDes[5],    // 2
      //                          (float)xStart,                                   // 3
      //                          (float)yStart,                                   // 4f
      //                          (float)0.4,      // 5
      //                          0,                                        // 6
      //                          0,                                        // 7
      //                          (float)stateCommand->data.stateDes[11],  // 8
      //                          (float)v_des_world[0],                           // 9
      //                          (float)v_des_world[1],                           // 10
      //                          0};   

       for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          //trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * stateCommand->data.stateDes[11];
          //std::cout << "yaw traj" <<  trajAll[12*i + 2] << std::endl;
        }
      }
    //}

    

        // for(int i = 0; i < 12; i++)
        //     printf("%.4f, ", trajAll[i]);

        // printf("\n\n");



    Timer t1;
    t1.start();
    dtMPC = dt * iterationsBetweenMPC;
    setup_problem(dtMPC,horizonLength,0.2,250);
    update_solver_settings(1 /*_parameters->jcqp_max_iter*/,1e-7 /* _parameters->jcqp_rho */,
      1e-8 /*_parameters->jcqp_sigma*/, 1.5 /* _parameters->jcqp_alpha*/, 1 /*_parameters->jcqp_terminate*/,0 /* _parameters->use_jcqp*/);
    //t1.stopPrint("Setup MPC");
    //std::cout << t1.getMs() << std::endl;
    Timer t2;
    t2.start();
    //cout << "dtMPC: " << dtMPC << "\n";
    // update_problem_data(p, v, q, w, r, yaw, weights, trajAll, alpha, mpcTable);
    
    //t2.stopPrint("Run MPC");
    printf("MPC Solve time %f ms\n", t2.getMs());
    //std::cout << t2.getSeconds() << std::endl;
    for(int leg = 0; leg < 4; leg++)
    {
      Vec3<double> f;
      for(int axis = 0; axis < 3; axis++)
        f[axis] = get_solution(leg*3 + axis);
     
      //f_ff[leg] =  - coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]) * f;
      f_ff[leg] =  - seResult.rBody * f;
      
      //std::cout << "leg: " << leg << std::endl;
      //std::cout << f_ff[leg] << std::endl;
      //std::cout << "mpc solution" << leg << "\n" << seResult.rBody * f << std::endl;
      // Update for WBC
       // Fr_des[leg] = f;      
    }

    // printf("update time: %.3f\n", t1.getMs());
  }
}



void ConvexMPCLocomotion::run_Jump(ControlFSMData& data)
{
  // std::cout<<"dc: "<< dc<< std::endl;
  bool omniMode = false;


//  auto* debugSphere = data.visualizationData->addSphere();
//  debugSphere->color = {1,1,1,0.5};
//  debugSphere->radius = 1;
 // data._legController->updateData();
 // data._stateEstimator->run();
  auto& seResult = data._stateEstimator->getResult();
  auto& stateCommand = data._desiredStateCommand;
  //std::cout << "in side mpc" << seResult.rBody << std::endl;;

  // pick gait
  Gait* gait = &trotting;
  if(gaitNumber == 1)
    gait = &bounding;
  else if(gaitNumber == 2)
    gait = &trotting;
  else if(gaitNumber == 3)
    gait = &walking;
  else if(gaitNumber == 4)
    gait = &pacing;
  else if(gaitNumber == 5)
    gait = &galloping;
  else if(gaitNumber == 6)
    gait = &pronking;
  current_gait = gaitNumber;
  // integrate position setpoint
  Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
  Vec3<double> v_des_world;
  //v_des_world = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]).transpose() * v_des_robot;
  //Vec3<double> v_robot = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2])*seResult.vWorld;

  v_des_world = seResult.rBody.transpose() * v_des_robot;
  Vec3<double> v_robot = seResult.vWorld;

  world_position_desired[0] += dt * v_des_world[0];
  world_position_desired[1] += dt * v_des_world[1];
  world_position_desired[2] = 0.3;
  
 // printf("p_des \t%.6f\n", dt * v_des_world[0]);
 //Integral-esque pitch and roll compensation
  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(stateCommand->data.stateDes[4] /*-hw_i->state_estimator->se_ground_pitch*/ - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(stateCommand->data.stateDes[3] /*-hw_i->state_estimator->se_ground_pitch*/
        - seResult.rpy[0])/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=6);  //turn off for pronking

  


  for(int i = 0; i < 4; i++) {
   pFoot[i] = seResult.position + seResult.rBody.transpose() 
            * (data._quadruped->getHipLocation(i) + data._legController->data[i].p);
   // pFoot[i] = data._legController->data[i].p;
    //std::cout << "pFoot" << i << "\n" << pFoot[i] << std::endl;
  }
  
  // some first time initialization
  if(firstRun)
  {
    std::cout << "Run MPC" << std::endl;
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];

    Vec3<double> v_des_robot(0,0,0);  // connect to desired state command later
    Vec3<double> v_des_world(0,0,0);  // connect to desired state command later

    Vec3<double> v_robot = seResult.vWorld;
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = world_position_desired[2];
    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0;

    pBody_RPY_des[0] = 0;
    pBody_RPY_des[1] = 0;
    pBody_RPY_des[2] = 0; // seResult.rpy[2];

    vBody_Ori_des[0] = 0;
    vBody_Ori_des[1] = 0;
    vBody_Ori_des[2] = 0; // set this for now

    firstRun = false;
  }
  // contact_state = gait->getContactSubPhase();

 
   // std::cout << "foot force 0 :" << lowState.footForce[0] << std::endl;
   // std::cout << "contact 0 : " << contact_state[0] << std::endl;

  // calc gait
  // gait->setIterations(iterationsBetweenMPC, iterationCounter);
  
  // load LCM leg swing gains
  // Kp << 500, 0, 0,
  //     0, 500, 0,
  //     0, 0, 350;
  // Kp_stance = 0*Kp;

  // Kd << 30, 0, 0,
  //     0, 30, 0,
  //     0, 0, 30;
  // Kd_stance = Kd;
  // gait
  // Vec4<double> contactStates = gait->getContactSubPhase();
  // std::cout<< "contactStates:" << contactStates[0]<<"," <<contactStates[1]<<","<<contactStates[2]<<","<<contactStates[3]<<std::endl;
  // Vec4<double> swingStates = gait->getSwingSubPhase();
  int* mpcTable = gait->mpc_gait();

  std::cout << "iterationCounter:" << iterationCounter << std::endl;
  updateMPCIfNeeded_Jump(mpcTable, data, omniMode);
  
  for(int foot = 0; foot < 4; foot++)
  {
    //  std::cout<< "computed force of foot"<<foot<<":"<<f_ff[foot];
     data._legController->commands[foot].feedforwardForce = f_ff[foot];

  }
  iterationCounter++;
//  StateEstimator* se = hw_i->state_estimator;
  Vec4<double> se_contactState(0,0,0,0);
}

void ConvexMPCLocomotion::updateMPCIfNeeded_Jump(int* mpcTable, ControlFSMData& data, bool omniMode) 
{
  //iterationsBetweenMPC = 30;
  // std::cout << "iterationCounter: "<< iterationCounter << std::endl;
  std::cout << "iterationsBetweenMPC: "<< iterationsBetweenMPC << std::endl;
  std::cout << "MPC iteration: "<< iterationCounter % iterationsBetweenMPC << std::endl;
  std::cout << "N_TrajRef: "<< N_TrajRef << std::endl;

  if((iterationCounter % iterationsBetweenMPC) == 0 && iterationCounter<=N_TrajRef){
    //std::cout << "size qDes is: " << QDes[0].size() << std::endl;
    int idx = iterationCounter;
    std::cout << "current_MPC: "<< current_MPC << std::endl;

    std::cout << "tauDes is: " << tauDes[3][idx] << std::endl;
    std::cout << "qDes is: " << QDes[0][idx] << std::endl;
    std::cout << "qDes is: " << QDes[1][idx] << std::endl;
    std::cout << "qDes is: " << QDes[2][idx] << std::endl;
    std::cout << "qDes is: " << QDes[3][idx] << std::endl;  
    std::cout<<  "F_des:"<< FDes[3][idx]<< std::endl;
  }


  if((iterationCounter % iterationsBetweenMPC) == 0)
  {
    int current_MPC=iterationCounter/30;
    std::cout << "current_MPC: "<< current_MPC << std::endl;

    //Jump_ref();
    Timer t0;
    t0.start();
  
    auto seResult = data._stateEstimator->getResult();
    auto& stateCommand = data._desiredStateCommand;

    double* p = seResult.position.data();
    double* v = seResult.vWorld.data();
    double* w = seResult.omegaWorld.data();
    double* q = seResult.orientation.data();

    // float* pf = (float*) seResult.position.data();
    // float* vf = (float*) seResult.vWorld.data();
    // float* wf = (float*) seResult.omegaWorld.data();
    // float* qf = (float*) seResult.orientation.data();

    double r[12];
    for(int i = 0; i < 12; i++)
      r[i] = pFoot[i%4][i/4] - seResult.position[i/4];

    // double Q[12] = {0.25, 50, 0.25, 2.5, 2.5, 30, 0, 0.1, 0, 0.2, 0.2, 0.2};
    double Q[12] = {0.25, 0.25, 0.25, 2.5, 2.5, 10, 0, 0.0, 0.3, 0.2, 0.2, 0.2};
    double pitch = seResult.rpy[1];
    double* weights = Q;
    double alpha = 1e-5; // make setting eventually

    //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

    if(alpha > 1e-4)
    {

      std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
      alpha = 1e-5;
    }
    Vec3<double> v_des_robot(stateCommand->data.stateDes[6], stateCommand->data.stateDes[7],0);
    //Vec3<double> v_des_world = coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]).transpose() * v_des_robot;

    Vec3<double> v_des_world = seResult.rBody.transpose() * v_des_robot;
    //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

      //world_position_desired = seResult.position;
      const double max_pos_error = .1;
      double xStart = world_position_desired[0];
      double yStart = world_position_desired[1];
      //std::cout << "orig " << xStart << "  " << yStart << std::endl;
      //printf("orig \t%.6f\t%.6f\n", xStart, yStart);
      //printf("ref: \t%.6f\t%.6f\n", p[0], p[1]);

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      //printf("xys: \t%.3f\t%3.f\n", xStart, yStart);
      //printf("perr \t%.3f\t%.3f\n", p[0] - world_position_desired[0], p[1] - world_position_desired[1]);
      
      double trajInitial[12] = {rpy_comp[0] + stateCommand->data.stateDes[3],  // 0 roll
                                rpy_comp[1] + stateCommand->data.stateDes[4],    // 1 pitch
                                stateCommand->data.stateDes[5],    // 2 yaw
                                xStart,                                   // 3  px
                                yStart,                                   // 4  py
                                0.3 ,   // 5 pz
                                0,                                        // 6 wx
                                0,                                        // 7 wy
                                stateCommand->data.stateDes[11],  // 8 wz
                                v_des_world[0],                           // 9 vx
                                v_des_world[1],                           // 10 vy
                                0};                                       // 11 vz

      // current coordinates
      double trajInitial_Jump[12] =  {seResult.rpy[0],                              // 0 roll
                                      seResult.rpy[1],                              // 1 pitch
                                      seResult.rpy[2],                              // 2 yaw
                                      p[0],                                   // 3  px
                                      p[1],                                   // 4  py
                                      p[2] ,   // 5 pz
                                      w[0],                                        // 6 wx
                                      w[1],                                        // 7 wy
                                      w[2],  // 8 wz
                                      v[0],                           // 9 vx
                                      v[1],                           // 10 vy
                                      v[2]};                                       // 11 vz

      for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          //trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * stateCommand->data.stateDes[11];
          //std::cout << "yaw traj" <<  trajAll[12*i + 2] << std::endl;
        }
      }

      for (int j=0; j<12; j++){
          trajAll_Jump[0]=seResult.rpy[0]; // roll
          trajAll_Jump[1]=seResult.rpy[1]; // pitch
          trajAll_Jump[2]=seResult.rpy[2]; // yaw
          trajAll_Jump[3]=p[0]; // px
          trajAll_Jump[4]=p[1]; // py
          trajAll_Jump[5]=p[2]; // pz
          trajAll_Jump[6]=w[0]; // wx
          trajAll_Jump[7]=w[1]; // wy
          trajAll_Jump[8]=w[2]; // wz
          trajAll_Jump[9]=v[0]; // vx
          trajAll_Jump[10]=v[1]; // vy
          trajAll_Jump[11]=v[2]; // vz
      }
      // for(int j = 0; j < 12; j++)
      //     trajAll_Jump[j] = trajInitial_Jump[j]; // The first column of trajAll_Jump is always trajInitial_Jump

      for(int i = 1; i < horizonLength; i++)
      {
        if ((current_MPC+i)*iterationsBetweenMPC<=N_TrajRef)
        {
          trajAll_Jump[12*i+0]=0; // roll
          trajAll_Jump[12*i+1]=QDes[2][(current_MPC+i)*iterationsBetweenMPC]; // pitch
          trajAll_Jump[12*i+2]=0; // yaw
          trajAll_Jump[12*i+3]=QDes[0][(current_MPC+i)*iterationsBetweenMPC]; // px
          trajAll_Jump[12*i+4]=0; // py
          trajAll_Jump[12*i+5]=QDes[1][(current_MPC+i)*iterationsBetweenMPC]+0.15; // pz, 0.15 is initial height in Matlab
          trajAll_Jump[12*i+6]=0; // wx
          trajAll_Jump[12*i+7]=QDes[8][(current_MPC+i)*iterationsBetweenMPC]; // wy
          trajAll_Jump[12*i+8]=0; // wz
          trajAll_Jump[12*i+9]=QDes[7][(current_MPC+i)*iterationsBetweenMPC]; // vx
          trajAll_Jump[12*i+10]=0; // vy
          trajAll_Jump[12*i+11]=QDes[9][(current_MPC+i)*iterationsBetweenMPC]; // vz
        }


        else
        {
          trajAll_Jump[12*i+0]=0; // roll
          trajAll_Jump[12*i+1]=QDes[2][N_TrajRef]; // pitch
          trajAll_Jump[12*i+2]=0; // yaw
          trajAll_Jump[12*i+3]=QDes[0][N_TrajRef]; // px
          trajAll_Jump[12*i+4]=0; // py
          trajAll_Jump[12*i+5]=QDes[1][N_TrajRef]+0.15; // pz
          trajAll_Jump[12*i+6]=0; // wx
          trajAll_Jump[12*i+7]=QDes[8][N_TrajRef]; // wy
          trajAll_Jump[12*i+8]=0; // wz
          trajAll_Jump[12*i+9]=QDes[7][N_TrajRef]; // vx
          trajAll_Jump[12*i+10]=0; // vy
          trajAll_Jump[12*i+11]=QDes[9][N_TrajRef]; // vz
        }
      }
    

    for (int i=0; i< horizonLength; i++){

        if ((current_MPC+i)*iterationsBetweenMPC<=dc){
          gait_jumpTable[4*i+0]=1;
          gait_jumpTable[4*i+1]=1;
          gait_jumpTable[4*i+2]=1;
          gait_jumpTable[4*i+3]=1;
        }
        else if((current_MPC+i)*iterationsBetweenMPC>dc && (current_MPC+i)*iterationsBetweenMPC<= dc+rc){

          gait_jumpTable[4*i+0]=0;
          gait_jumpTable[4*i+1]=0;
          gait_jumpTable[4*i+2]=1;
          gait_jumpTable[4*i+3]=1;
        }
        else{
          gait_jumpTable[4*i+0]=0;
          gait_jumpTable[4*i+1]=0;
          gait_jumpTable[4*i+2]=0;
          gait_jumpTable[4*i+3]=0;
        }
    }
    // Vec12<double> Fjump_des;
    double Fjump_des[12]={0,0,0,0,0,0,0,0,0,0,0,0};


    if (current_MPC*iterationsBetweenMPC <=N_TrajRef){
      Fjump_des[0]=FDes[0][current_MPC*iterationsBetweenMPC];
      Fjump_des[1]=0;
      Fjump_des[2]=FDes[1][current_MPC*iterationsBetweenMPC];
      Fjump_des[3]=FDes[0][current_MPC*iterationsBetweenMPC];
      Fjump_des[4]=0;
      Fjump_des[5]=FDes[1][current_MPC*iterationsBetweenMPC];
      Fjump_des[6]=FDes[2][current_MPC*iterationsBetweenMPC];
      Fjump_des[7]=0;
      Fjump_des[8]=FDes[3][current_MPC*iterationsBetweenMPC];
      Fjump_des[9]=FDes[2][current_MPC*iterationsBetweenMPC];
      Fjump_des[10]=0;
      Fjump_des[11]=FDes[3][current_MPC*iterationsBetweenMPC];
    } else{
      for (int k=0; k<12;k++){
        Fjump_des[k]=0;
      }
    }
    // std::cout<<"Fjump_des:"<< Fjump_des[9]<<std::endl;

    // for(int leg = 0; leg < 4; leg++)
    // {
    //   std::cout<<"Fjump_des:"<< Fjump_des[leg*3] << std::endl;
    //   // for(int axis = 0; axis < 3; axis++){
    //   //   std::cout<<"Fjump_des:"<< Fjump_des[leg*3 + axis] << std::endl;
    //   // }    
    // }
    // for(int i = 0; i < horizonLength; i++)
    // {
    //   for(int j = 0; j < 4; j++)
    //   {
    //     std::cout<< "test jumpTable" << gait_jumpTable[i*4 + j]<< std::endl;
    //   }

    //   std::cout<<"--------------"<< std::endl;
    // }

        // for(int i = 0; i < 12; i++)
        //     printf("%.4f, ", trajAll[i]);

        // printf("\n\n");

    Timer t1;
    t1.start();
    dtMPC = dt * iterationsBetweenMPC;
    setup_problem(dtMPC,horizonLength,0.2,250);
    update_solver_settings(1 /*_parameters->jcqp_max_iter*/,1e-7 /* _parameters->jcqp_rho */,
      1e-8 /*_parameters->jcqp_sigma*/, 1.5 /* _parameters->jcqp_alpha*/, 1 /*_parameters->jcqp_terminate*/,0 /* _parameters->use_jcqp*/);
    //t1.stopPrint("Setup MPC");
    //std::cout << t1.getMs() << std::endl;
    Timer t2;
    t2.start();
    //cout << "dtMPC: " << dtMPC << "\n";
    update_problem_data(p, v, q, w, r, pitch, weights, trajAll_Jump, alpha, mpcTable, gait_jumpTable);
    // update_problem_data(p, v, q, w, r, yaw, weights, trajAll, alpha, mpcTable);
    // update_problem_data_jump(p, v, q, w, r, yaw, weights, trajAll, alpha, gait_jump);
    
    //t2.stopPrint("Run MPC");
    printf("MPC Solve time %f ms\n", t2.getMs());
    //std::cout << t2.getSeconds() << std::endl;
    for(int leg = 0; leg < 4; leg++)
    {
      Vec3<double> f;
      for(int axis = 0; axis < 3; axis++){
           f[axis] = get_solution(leg*3 + axis);// original
        // f[axis] = get_solution(leg*3 + axis)-Fjump_des[leg*3 + axis];
        // std::cout<<"Fjump_des:"<< Fjump_des[leg*3 + axis] << std::endl;
        std::cout<<"get_solution(leg*3 + axis):"<< get_solution(leg*3 + axis) << std::endl;
      }

      //f_ff[leg] =  - coordinateRotation(CoordinateAxis::Z, seResult.rpy[2]) * f;
      f_ff[leg] =  - seResult.rBody * f;
      
      //std::cout << "leg: " << leg << std::endl;
      //std::cout << f_ff[leg] << std::endl;
      //std::cout << "mpc solution" << leg << "\n" << seResult.rBody * f << std::endl;
      // Update for WBC
       // Fr_des[leg] = f;      
    }

    // printf("update time: %.3f\n", t1.getMs());
  }

}