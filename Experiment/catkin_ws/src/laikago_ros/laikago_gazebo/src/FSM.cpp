#include "../include/FSM.h"
#include "ros/ros.h"
#include "../include/Utilities/Timer.h"
#include "../include/body.h"


FSM_State::FSM_State(ControlFSMData* _controlFSMData):
_data(_controlFSMData),
Cmpc(0.001, 30) 
{
    std::cout<< "zero transition data" << std::endl;
    transitionData.zero();
    std::cout << "Initialize FSM" << std::endl;
}
void FSM_State::Successive_jumping(){
    int Jumping_N = 3;

    ofstream pos; // include QP landing period
    ofstream myfile;
    ofstream MPC_F;
    ofstream QP;
    ofstream z_pos;
    ofstream b_des;
    pos.open("pos.txt");
    myfile.open ("ori.txt");
    QP.open("QPsolution.txt");
    MPC_F.open("MPC_F.txt");
    z_pos.open("zPos.txt");
    b_des.open("b_des_z.txt");
    ros::Rate rate(1000);

    //------------ Read the optimization data from imported files --------------------
    // ------------Stored the data into vectors --------------------------------------
    // Position
    vector<vector<double>> QDes;
    // Torque
    vector<vector<double>> tauDes;
    // foot pos
    vector<vector<double>> pfDes;
    //foot vel
    vector<vector<double>> vfDes;
    //foot force
    vector<vector<double>> FDes;

    vector<vector<double>> QDes2;
    // Torque
    vector<vector<double>> tauDes2;
    // foot pos
    vector<vector<double>> pfDes2;
    //foot vel
    vector<vector<double>> vfDes2;

    double recording_MPC[12];
    
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
    bool getcontact = true;

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
    /////////////////////////////////////////////////////////////////////////

    // reset forces and steps to 0
    footFeedForwardForces = Mat34<double>::Zero();
    footstepLocations = Mat34<double>::Zero();
    double minForce = 5;
    double maxForce = 500;
    // double contactStateScheduled[4] = {1, 1, 1, 1};

    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

    Vec4<double> contactphase(0.5,0.5,0.5,0.5); 
    _data->_stateEstimator->setContactPhase(contactphase);
    //Timer time;
    //time.start();
    // double COM_weights_stance[3] = {5, 5, 10};
    double COM_weights_stance[3] = {5, 5, 10};
    double Base_weights_stance[3] = {10, 10, 20};
    double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
      omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
    double b_control[6];
    int counter = 0;
    
    std::cout << "counter0: " << counter << std::endl;
       // ros::spinOnce();
       _data->_legController->updateData();
       _data->_stateEstimator->run();
    // position & rpy desired
    for(int i = 0; i < 3; i++){
      //p_des[i] = _data->_stateEstimator->getResult().position(i); 
      v_des[i] = 0;
      omegaDes[i] = 0;
      rpy[i] = 0;
    }
      // p_des[2] = 0.3; // end com height position for robot
     //bool firstLocoRun = true;
    
    // bool runMPC = true;
    
    double Kp = 250;
    double Kd = 3; 
    double Hip_Kp = 250;
    double Hip_Kd =3;
    double Kp_l=5;
    double Kd_l=1;
    // double init_Pos[12] = {0, 1.2566, -2.355, 0, 1.2566, -2.355, 0, 1.2566, -2.355, 0, 1.2566, -2.355}; // pose 0, for jump 2D 60cm
    double init_Pos[12] = {0, 1.1310, -2.1206, 0, 1.1310, -2.1206, 0, 1.1310, -2.1206, 0, 1.1310, -2.1206}; // pose 0, backflip,cs=50,40,50
    double currentPos[12], percent;
    double new_init_Pos[12];
    double Ff_pos[3];

    /*=========== setup Cartesian PD controller parameters ==========*/
    double side_sign[4] = {-1, 1, -1, 1};
    Vec3<double> Cart_PD_des;
    Vec3<double> world_com;
    Vec3<double> Cart_PD_body_des;

    /*=========Jumping duration parameters=========*/
    int N_TrajRef=1400;
    int startjump=2000;
    int pose_time=200;


    double box_height=0;
    p_des[2] = 0.3+box_height; // end com height position for robot

    // int N_TrajRef=500;
    // int startjump=2000;
    // int pose_time=0;// need to check again


    int idx_pose=N_TrajRef-pose_time;
    int idx=0;

    double FR=0; double FL=0; double RR=0; double RL=0; // contact state
    double contactStateScheduled[4];
  /*===========Classify the parameters: Determine the jumping index=========*/
    double ddt = 0.001;
    
    int T_jump;T_jump = 110; // total simulation time
    int N_jump;N_jump = 110; // Number of the jumps we want

    double tn; tn = T_jump/ddt;
    int n_tn = (int)tn;
    
    int index_jump;
    int reset_counter;
    double act_x;
    double p_actworld[3];

    int balance_time = 300;
    int t_compensate = 300;

    double reset_comx;

    double n_jump[n_tn];             // setup the index of the jump
    double L_dist[N_jump];            // store the jumping distance
    double start[N_jump];             // store the starting time of each jump
    double Frcontact[n_tn];          // store the state of contact of front right foot
    double Flcontact[n_tn];          // store the state of contact of front left foot
    double Rrcontact[n_tn];          // store the state of contact of rear right foot
    double Rlcontact[n_tn];          // store the state of contact of rear left foot
    
    for (int i = 0; i < tn ; i++){n_jump[i] = 0;}
    for (int i = 0; i < N_jump; i++){L_dist[i] = 0;}
    for (int i = 0; i < N_jump; i++){start[i] = 0;}
    for (int i = 0; i < tn; i++){Frcontact[i] = 0;}
    for (int i = 0; i < tn; i++){Flcontact[i] = 0;}
    for (int i = 0; i < tn; i++){Rrcontact[i] = 0;}
    for (int i = 0; i < tn; i++){Rlcontact[i] = 0;}

    Vec3<double> pFeetVecCOM;
  /*============================================================*/
  // determine the contact phase
  int CP_Fr;
  int CP_Fl;
  int CP_Rr;
  int CP_Rl;
  // determine the duration for the trajectory
  int fc; // TO front contact
  int dc; // TO double contact
  int rc; // TO rear contact
  int fl; // TO flight phase
    while(ros::ok()) {
      /*-----------<test area>---------------*/
      // Cmpc.dc = 100;
      // std::cout<<"test: " <<Cmpc.dc<<std::endl;
    
      // rate.reset();
      _data->_legController->updateData();
      _data->_stateEstimator->run();

      // get foot position from the jacobian 
      for (int leg = 0; leg < 4; leg++) {
        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);

        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
      }
      // for(int i; i <3;i++){p_act[i] = _data->_stateEstimator->getResult().position(i);}

      p_actworld[0] = _data->_stateEstimator->getResult().position(0) + (pFeet[0] + pFeet[3]) / 2.00;// in x direction 
      p_actworld[1] = _data->_stateEstimator->getResult().position(1) + (pFeet[1] + pFeet[4]) / 2.00;
      p_actworld[2] = _data->_stateEstimator->getResult().position(2) + (pFeet[2] + pFeet[5]) / 2.00;

      
      /*===============which jumping: determine the index of the jumping and reset the coordinate=============*/
      if(boolean_runwhichjump){
        if(lowState.footForce[0]==0){Frcontact[counter]=0;CP_Fr = 0;}else{Frcontact[counter]=1;CP_Fr = 1;}
        if(lowState.footForce[1]==0){Flcontact[counter]=0;CP_Fl = 0;}else{Flcontact[counter]=1;CP_Fl = 1;}
        if(lowState.footForce[2]==0){Rrcontact[counter]=0;CP_Rr = 0;}else{Rrcontact[counter]=1;CP_Rr = 1;}
        if(lowState.footForce[3]==0){Rlcontact[counter]=0;CP_Rl = 0;}else{Rlcontact[counter]=1;CP_Rl = 1;}

        // record the index of the jumping  
        index_jump = n_jump[counter];
        if (counter > 2500){
          std::cout<<"No.Jumping: "<< index_jump << std::endl;
          
          if(counter > start[index_jump] + (balance_time + t_compensate + 200) && Frcontact[counter-1]==0 && Frcontact[counter]==1){
              
              int lval_njump = n_jump[counter];
              for(int i=0;i<n_tn;i++){n_jump[i]=lval_njump+1;};
              start[index_jump+1] = counter;
              L_dist[index_jump+1] =  p_actworld[0]; // read the current distance { issue: it should be the front feet x p}
              
              std::cout<<"L_dis: "<< L_dist[index_jump+1] << std::endl;
              
              int load_ref_dis = 60;
              switch(load_ref_dis){
                case(60):
                  
                  fc = 100; 
                  dc = 200;
                  rc = 250;
                  fl = 400;
                  N_TrajRef = fc + dc + rc + fl;
                  // setup the MPC trajecotry parameters
                  Cmpc.dc = fc+dc; // in this case the MPC start from 0.2s so the fc can be included into the front contact
                  Cmpc.rc = rc;
                  Cmpc.fl = fl;
                  Cmpc.N_TrajRef = N_TrajRef;
                  Cmpc.reset_pff = L_dist[index_jump+1];
                  // std::cout<<"test MPC parameter"<<Cmpc.reset_pff<< std::endl;

                  boolean_runPDnewref = true;
                  QDes.clear();
                  // Torque
                  tauDes.clear();
                  // foot pos
                  pfDes.clear();
                  //foot vel
                  vfDes.clear();

                  std::cout<<"reset the TO reference" <<std::endl;
                  std::ifstream myFile;
                  myFile.open("src/laikago_ros/optimization_data/TO_ref_gait_library/ref_60/60cm_Q.csv");
                  // src/laikago_ros/optimization_data/jump2D/data_Q.csv

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
                  std::ifstream mytauFile("src/laikago_ros/optimization_data/TO_ref_gait_library/ref_60/60cm_tau.csv");

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

                  std::ifstream myFFile("src/laikago_ros/optimization_data/TO_ref_gait_library/ref_60/60cm_F.csv");
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
                  for(int i = 0;i<2;i++){
                    new_init_Pos[i*3] = 0.0;
                    new_init_Pos[i*3+1] = -QDes[3][200];
                    new_init_Pos[i*3+2] = -QDes[4][200];

                    new_init_Pos[i*3+6] = 0.0;
                    new_init_Pos[i*3+7] = -QDes[5][200];
                    new_init_Pos[i*3+8] = -QDes[6][200];
                  }
                  // push the trajectory to the MPC
                  Cmpc.QDes = QDes;
                  Cmpc.FDes = FDes;
                  break;
              }        
          }
        }
      index_jump = n_jump[counter];
      reset_counter = counter - start[index_jump];
      act_x = _data->_stateEstimator->getResult().position(0);
      reset_comx = act_x - L_dist[index_jump];
      
      std::cout << "reset x : " << reset_comx << std::endl;
      std::cout << "actual x : " << act_x << std::endl;
      std::cout << "   " << std::endl;
    }
      /*================  The first jumping  =================*/
      if(n_jump[counter]<1){
          if (counter <startjump){  // Initialization
              runQP=false;
              boolean_runPD = false;
              boolean_runcartesianPD = false;
              std::cout << "Controller: PD controller initial the condition" << std::endl; 
              // std::cout << "FLfootCallback: " << lowState.footForce[1] << std::endl;
              // std::cout << "RLfootCallback: " << lowState.footForce[3] << std::endl;
              //double contactStateScheduled[4]={1,1,1,1}; // all legs are stance legs
            for(int i = 0; i < 4; i++){
                _data->_legController->commands[i].kpJoint = Vec3<double>(Hip_Kp,Kp,Kp).asDiagonal();
                _data->_legController->commands[i].kdJoint = Vec3<double>(Hip_Kd,Kd,Kd).asDiagonal();
                _data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
                _data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();
            }

            percent = (double) counter/ startjump; 
              std ::cout << "percent: " << percent << std::endl; 
              for (int i=0; i<4; i++){ 
                  for(int j=0; j<3; j++){ 
                    currentPos[3*i+j] = _data->_legController->data[i].q(j); 
                    _data->_legController->commands[i].qDes(j) = currentPos[3*i+j]*(1-percent) + init_Pos[3*i+j]*percent; 
                  } 
              } 
          }

          /*======================== MPC locomotion ====================*/
          if(counter>startjump && counter<startjump+N_TrajRef-pose_time){
            runQP = false;
            boolean_runPD = true;
            boolean_runcartesianPD = false;
            boolean_runMPC = false;
            std::cout << "Controller: PD controller tracks the TO" << std::endl;
            z_pos << _data->_stateEstimator->getResult().position[2] << " " <<  _data->_stateEstimator->getResult().position[1] << std::endl;
            Vec3<double> v_des(0, 0, 0); // v_body
            idx= counter-startjump;

          }
      }
      // The second jumping
      else{
          if(reset_counter<=balance_time){
              runQP=true;
              boolean_runPD = false;
              boolean_runcartesianPD = false;
              boolean_runMPC = false;
              std::cout<< "Controller: QP activated" << std::endl;
              if(getcontact){
                FR=lowState.footForce[0];FL=lowState.footForce[1];
                RR=lowState.footForce[2];RL=lowState.footForce[3];

                //reset the PD controller parameters
                _data->_legController->commands[0].kpJoint << Kp_l, 0, 0,0, Kp_l, 0,0, 0, Kp_l;
                _data->_legController->commands[0].kdJoint << Kd_l, 0, 0,0, Kd_l, 0,0, 0, Kd_l; 
                _data->_legController->commands[1].kpJoint << Kp_l, 0, 0,0, Kp_l, 0,0, 0, Kp_l;
                _data->_legController->commands[1].kdJoint << Kd_l, 0, 0,0, Kd_l, 0,0, 0, Kd_l;
                _data->_legController->commands[2].kpJoint << Kp_l, 0, 0,0, Kp_l, 0,0, 0, Kp_l;
                _data->_legController->commands[2].kdJoint << Kd_l, 0, 0,0, Kd_l, 0,0, 0, Kd_l;
                _data->_legController->commands[3].kpJoint << Kp_l, 0, 0, 0, Kp_l, 0,0, 0, Kp_l;
                _data->_legController->commands[3].kdJoint << Kd_l, 0, 0, 0, Kd_l, 0,0, 0, Kd_l;     
                if (FR>0){FR=1;}else{FR=1;}
                // std::cout <<"FR after if function: " << FR << std::endl;
                if (FL>0){FL=1;}else{FL=1;}
                // std::cout <<"FL after if function: " << FL << std::endl;
                if (RR>0){RR=1;}else{RR=0;}
                // std::cout <<"RR after if function: " << RR << std::endl;
                if (RL>0){RL=1;}else{RL=0;}
                // std::cout <<"RL after if function: " << RL << std::endl;
                contactStateScheduled[0]=1;contactStateScheduled[1]=1;
                contactStateScheduled[2]=1;contactStateScheduled[3]=1;
              }
              std::cout <<"GRF1: " << lowState.footForce[0]
                        <<" GRF2: " << lowState.footForce[1]
                        <<" GRF3: " << lowState.footForce[2]
                        <<" GRF4: " << lowState.footForce[3]
                        <<std::endl;          
              std::cout << "contactStateScheduled: " << contactStateScheduled[0] << "," << contactStateScheduled[1]<<"," << contactStateScheduled[2]<< "," << contactStateScheduled[3]<< std::endl;
              index = reset_counter;
              if(index>199){index = 199;}
              if(lowState.footForce[2]==0){}else{index=199;}
              if(RR == 1 || RL ==1) {boolean_runPD_flightphase = false;}
              // if(boolean_runPD_flightphase){
              //   std::cout << "Activate RR PD controller " << std::endl;
              //   _data->_legController->commands[2].kpJoint = Vec3<double>(Hip_Kp,Kp,Kp).asDiagonal();
              //   // add PD controller
              //   _data->_legController->commands[2].tau << 0, -tauDes[2][idx], -tauDes[3][idx];// rear right leg
              //   _data->_legController->commands[2].qDes << 0, -QDes[5][idx], -QDes[6][idx];
              //   _data->_legController->commands[2].qdDes << 0, -QDes[12][idx], -QDes[13][idx];
              // }
              // if(boolean_runPD_flightphase){
              //   std::cout << "Activate RL PD controller " << std::endl;
              //   _data->_legController->commands[3].kpJoint = Vec3<double>(Hip_Kp,Kp,Kp).asDiagonal();
              //   // add PD controller
              //   _data->_legController->commands[3].tau << 0, -tauDes[2][idx], -tauDes[3][idx];// rear right leg
              //   _data->_legController->commands[3].qDes << 0, -QDes[5][idx], -QDes[6][idx];
              //   _data->_legController->commands[3].qdDes << 0, -QDes[12][idx], -QDes[13][idx];
              // }
          }
          else{
              /*<------------compensate phase---------------->*/
              int re_aftb = reset_counter-balance_time;//initial setup
              //compensate difference a little
              boolean_runPD_flightphase = true;
              if(re_aftb<=t_compensate){
                boolean_runPD = false;
                runQP=false;
                boolean_runcartesianPD = false;
                boolean_runMPC = false;
                // std::cout << "Controller: PD controller initial the condition" << std::endl; 
                // std::cout << "FLfootCallback: " << lowState.footForce[1] << std::endl;
                // std::cout << "RLfootCallback: " << lowState.footForce[3] << std::endl;
                  //double contactStateScheduled[4]={1,1,1,1}; // all legs are stance legs
                for(int i = 0; i < 4; i++){
                    _data->_legController->commands[i].kpJoint = Vec3<double>(Hip_Kp,Kp,Kp).asDiagonal();
                    _data->_legController->commands[i].kdJoint = Vec3<double>(Hip_Kd,Kd,Kd).asDiagonal();
                    _data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
                    _data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();
                }

                percent = (double) re_aftb/ t_compensate; 
                  std ::cout << "percent: " << percent << std::endl; 
                  for (int i=0; i<4; i++){ 
                    for(int j=0; j<3; j++){ 
                      currentPos[3*i+j] = _data->_legController->data[i].q(j); 
                      _data->_legController->commands[i].qDes(j) = currentPos[3*i+j]*(1-percent) + new_init_Pos[3*i+j]*percent; 
                    }
                  }
              }
              /*<------------jumping phase---------------->*/
              else{
                    boolean_runPD = true;
                    runQP=false;
                    boolean_runcartesianPD = false;
                    boolean_runMPC = true;

                  std::cout << "Controller: PD controller tracks the TO" << std::endl;
                  std::cout << "Controller: delta_MPC controller tracks the TO" << std::endl; 
 
                  idx= reset_counter-balance_time-t_compensate+200;
                  if(idx<=0){idx=1;}
                  if(idx>=N_TrajRef){idx=N_TrajRef;}
                  std::cout<< "idx: " << idx <<std::endl;
                  

                  if(p_actworld[2]<0.1 && CP_Fr == 0 && idx>500){
                    std::cout<< "front feet is nealy 5 cm height: " <<p_actworld[2]<< std::endl;
                    // input the desire world frame coordinate (front leg Cartesian PD control)
                    boolean_runPD = false;
                    runQP=false;
                    boolean_runcartesianPD = true;
                    boolean_runMPC = false;
                    
                    for(int leg = 0; leg < 2 ; leg++){

                      Cart_PD_des[0] = 0.55;
                      Cart_PD_des[1] = 0.13*side_sign[leg];
                      Cart_PD_des[2] = 0;

                      // convert world frame to the body frame
                      world_com[0] = Cart_PD_des[0] - _data->_stateEstimator->getResult().position(0);
                      world_com[1] = Cart_PD_des[1] - _data->_stateEstimator->getResult().position(1);
                      world_com[2] = Cart_PD_des[2] - _data->_stateEstimator->getResult().position(2);

                      Cart_PD_body_des = _data->_stateEstimator->getResult().rBody*world_com - _data->_quadruped->getHipLocation(leg);
                      
                      _data->_legController->commands[leg].kpCartesian = Vec3<double>(20,20,20).asDiagonal();
                      _data->_legController->commands[leg].kdCartesian = Vec3<double>(20,20,20).asDiagonal();

                      _data->_legController->commands[leg].pDes << Cart_PD_body_des[0], Cart_PD_body_des[1], Cart_PD_body_des[2];
                      _data->_legController->commands[leg].vDes << 0,0,0;
                    
                    }
                  }
                  else{
                      boolean_runPD = true;
                      runQP=false;
                      boolean_runcartesianPD = false;
                      for(int i = 0; i < 4; i++){
                        _data->_legController->commands[i].kpCartesian = Vec3<double>(0,0,0).asDiagonal();
                        _data->_legController->commands[i].kdCartesian = Vec3<double>(0,0,0).asDiagonal();
                      }
                  }
              }
          }
      }
      // For 2 jumps
      if(n_jump[counter]<Jumping_N){

      }
      else{
        boolean_runPD = false;
        runQP=true;
        boolean_runcartesianPD = false;
        boolean_runMPC = false;
      }

      // std::cout << "GRF1: " << lowState.footForce[0]<< std::endl;
      // std::cout << "GRF2: " << lowState.footForce[2]<< std::endl;


      // activate QP controller
      if(runQP){
        for(int i = 0;i < 4 ; i++){
          _data->_legController->commands[i].kpJoint = Mat3<double>::Zero();
          _data->_legController->commands[i].kdJoint = Mat3<double>::Zero();
          _data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
          _data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();
        }
        
        for (int i = 0; i < 4; i++) {
          se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
        }
        // v_des[2] = 0.1;
        for (int i = 0; i < 3; i++) {
        //rpy[i] = 0;
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        
        v_act[i] = _data->_stateEstimator->getResult().vWorld(i);

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaWorld(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vWorld(i);
        
        // Set the translational and orientation gains
        // kpCOM[i] = 80;   //_data->controlParameters->kpCOM(i);
        // kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
        // kpBase[i] = 800;     //_data->controlParameters->kpBase(i);
        // kdBase[i] = 20; //  _data->controlParameters->kdBase(i);

          kpCOM[i] = 30;   //_data->controlParameters->kpCOM(i);
          kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
          kpBase[i] = 80;     //_data->controlParameters->kpBase(i);
          kdBase[i] = 20; //  _data->controlParameters->kdBase(i);
        }

        // kpCOM[0] = 100;
        // kpCOM[1] = 0;
        // kpCOM[2] = 100;

        // kdCOM[0] = 20;
        // kdCOM[1] = 20; 
        // kdCOM[2] = 20;


        // kpBase[0] = 100;
        // kpBase[1] = 600;
          // kpCOM[0] =100;
          // kpCOM[2] = 50;
          // kpBase[0] = 0;
          // kpBase[1] = 2000;


        // se_xfb[4] = reset_comx;

        //Vec3<double> pFeetVec;
        Vec3<double> pFeetVecCOM;

        // Get the foot locations relative to COM
        for (int leg = 0; leg < 4; leg++) {
          // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
          //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
          //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                    //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

          pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
          (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);

          pFeet[leg * 3] = pFeetVecCOM[0];
          pFeet[leg * 3 + 1] = pFeetVecCOM[1];
          pFeet[leg * 3 + 2] = pFeetVecCOM[2];
          //std::cout << "leg" << leg << std::endl;
          //std::cout << "pFeet" << pFeetVecCOM << std::endl;
        }
        _data->_stateEstimator->setContactPhase(contactphase);
        // input the desired 



        if(boolean_runPDnewref){
          // if(index>=200){index = 199;}
          // desired rotation
          rpy[0] = 0;
          rpy[1] = QDes[2][index];
          // rpy[1] = 0;
          rpy[2] = 0;
          // desired COM Position
          // p_des[0] = QDes[0][index];//x
          // p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10]) / 4.00;//y

          p_des[0] = p_act[0] + (pFeet[0] + pFeet[3] + pFeet[6] + pFeet[9]) / 4.0;
          p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10]) / 4.0;


          p_des[2] = QDes[1][index];//z
          //desire COM Velocity
          v_des[0] = QDes[7][index];
          v_des[1] = 0;
          v_des[2] = QDes[8][index];

          omegaDes[1] = QDes[9][index];
        }
        else{
          p_des[0] = p_act[0] + (pFeet[0] + pFeet[3] + pFeet[6] + pFeet[9]) / 4.00;
          p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10]) / 4.00;
          p_des[2] = 0.2;

        }

        if(n_jump[counter]==1){
            p_des[0] = p_act[0] + (pFeet[0] + pFeet[3] + pFeet[6] + pFeet[9]) / 4.0;
            p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10]) / 4.0;
            p_des[2] = 0.2;
        }

        std::cout<<"x_d[0]: "<< p_des[0] <<std::endl;
        std::cout<<"z_d[0]: "<< p_des[2] <<std::endl;
        std::cout<<"act_comresetx: "<< se_xfb[4] <<std::endl;
        std::cout<<"act_comy: "<< se_xfb[6] <<std::endl;

        //  myfile << "\n";
        // std::cout << j << std::endl;
        //std::cout << "run QP" << std::endl;
        balanceController.set_alpha_control(0.01); 
        balanceController.set_friction(0.5);
        balanceController.set_mass(_data->_quadruped->mass);
        balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
        balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
        // std::cout<<"test rpy: "<< rpy[1] <<std::endl;
        balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
        balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
        balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                          O_err, _data->_stateEstimator->getResult().rpy(2));
        // balanceController.print_QPData();
        double fOpt[12];
        balanceController.solveQP_nonThreaded(fOpt);

        std::cout << "zForce leg 0: " << fOpt[2] << std::endl;
        std::cout << "zForce leg 1: " << fOpt[5] << std::endl;
        std::cout << "zForce leg 2: " << fOpt[8] << std::endl;
        std::cout << "zForce leg 3: " << fOpt[11] << std::endl;

        std::cout << "xForce leg 0: " << fOpt[0] << std::endl;
        std::cout << "xForce leg 1: " << fOpt[3] << std::endl;
        std::cout << "xForce leg 2: " << fOpt[6] << std::endl;
        std::cout << "xForce leg 3: " << fOpt[9] << std::endl;
        //balanceController.get_b_matrix(b_control);
        //b_des << b_control[2] << "\n";

        // Publish the results over ROS
        // balanceController.publish_data_lcm();

        // Copy the results to the feed forward forces
          
        //_data->_stateEstimator->run();


        for (int leg = 0; leg < 4; leg++) {
            footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
            fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame
            
            _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
            //_data->_stateEstimator->getResult().rBody.transpose() * footFeedForwardForces.col(leg); 
            //std::cout <<"zForce leg "<< _data->_legController->commands[leg].feedforwardForce[2]<< "\n";
            //QP <<_data->_legController->commands[i].feedforwardForce[2] << " ";
        }
      }

      // std::cout<< "ref size: " << sizeof(QDes)/sizeof(QDes[0]) <<std::endl;
      if(boolean_runPD){
          std::cout<< "idx: " << idx <<std::endl;
          _data->_legController->commands[0].tau << 0, -tauDes[0][idx], -tauDes[1][idx];// front right leg
          _data->_legController->commands[1].tau << 0, -tauDes[0][idx], -tauDes[1][idx];// front right leg
          _data->_legController->commands[2].tau << 0, -tauDes[2][idx], -tauDes[3][idx];// rear right leg
          _data->_legController->commands[3].tau << 0, -tauDes[2][idx], -tauDes[3][idx];// rear right leg

          _data->_legController->commands[0].qDes << 0, -QDes[3][idx], -QDes[4][idx]; // front right leg
          _data->_legController->commands[1].qDes << 0, -QDes[3][idx], -QDes[4][idx];
          _data->_legController->commands[2].qDes << 0, -QDes[5][idx], -QDes[6][idx];// rear right leg
          _data->_legController->commands[3].qDes << 0, -QDes[5][idx], -QDes[6][idx];

          _data->_legController->commands[0].qdDes << 0, -QDes[10][idx], -QDes[11][idx]; // front right leg
          _data->_legController->commands[1].qdDes << 0, -QDes[10][idx], -QDes[11][idx];
          _data->_legController->commands[2].qdDes << 0, -QDes[12][idx], -QDes[13][idx]; // rear right leg
          _data->_legController->commands[3].qdDes << 0, -QDes[12][idx], -QDes[13][idx];
      }
      
      if(boolean_runMPC){    
          //MPC controller

          Cmpc.iterationCounter = idx;
          // Cmpc.iterationCounter = counter - startjump;
          Cmpc.run_Jump(*_data);
          std::cout<<"test index"<<idx<<std::endl;
          // std::cout<<"test_MPC iteration counter"<<Cmpc.iterationCounter<<std::endl;
      }
      // print the state of the world frame and the robot state
      std::cout << " " << std::endl;
      std::cout << "<-----State of the world and robot----->" << std::endl;
      std::cout << "com_x/y/z: " <<_data->_stateEstimator->getResult().position(0)<<", " << _data->_stateEstimator->getResult().position(1) <<", " << _data->_stateEstimator->getResult().position(2) << std::endl;
      // std::cout << "foot x position: " << pFeet[0]<<", " << pFeet[3] <<", " << pFeet[6]<<", " << pFeet[9] << std::endl;
      // std::cout << "foot z position: " << pFeet[2]<<", " << pFeet[5] <<", " << pFeet[8]<<", " << pFeet[11] << std::endl;
    
      // print the front foot position
      std::cout<<" Fr_posx: "<<(_data->_stateEstimator->getResult().position(0) + pFeet[0])
               <<" Fl_posx: "<<(_data->_stateEstimator->getResult().position(0) + pFeet[3])
               <<" Rr_posx: "<<(_data->_stateEstimator->getResult().position(0) + pFeet[6])
               <<" Fl_posx: "<<(_data->_stateEstimator->getResult().position(0) + pFeet[9])
               <<std::endl;
    _data->_legController->updateCommand();
    // write the data into the file
    if(boolean_runMPC){
      for(int i=0;i<4;i++){
        for(int j=0;j<3;j++){
          recording_MPC[i*3+j] = _data->_legController->commands[i].feedforwardForce(j);
        }
      }
    }
    else{      
      for(int i=0;i<4;i++){
        for(int j=0;j<3;j++){
          recording_MPC[i*3+j] = 0;
        }
      }
    }
      

    if (counter >=startjump && counter<= startjump+N_TrajRef+5000){

        pos << _data->_stateEstimator->getResult().vWorld(0) << " " << _data->_stateEstimator->getResult().vWorld(1) << " " <<  _data->_stateEstimator->getResult().vWorld(2) << " " 
        <<_data->_stateEstimator->getResult().position(0) << " " << _data->_stateEstimator->getResult().position(1) << " " << _data->_stateEstimator->getResult().position(2) << " " 
        << _data->_stateEstimator->getResult().rpy(0) << " " << _data->_stateEstimator->getResult().rpy(1) << " " << _data->_stateEstimator->getResult().rpy(2) << " " 
        << _data->_legController->commands[0].feedforwardForce(2) << " " <<  _data->_legController->commands[1].feedforwardForce(2) << " " <<  _data->_legController->commands[2].feedforwardForce(2) << " " <<  _data->_legController->commands[3].feedforwardForce(2) << " " 
        << _data->_stateEstimator->getResult().contactEstimate(0) << " " << _data->_stateEstimator->getResult().contactEstimate(1) << " " << _data->_stateEstimator->getResult().contactEstimate(2) << " " << _data->_stateEstimator->getResult().contactEstimate(3)<<" "
        << lowState.footForce[0] << " " << lowState.footForce[1] <<" "<< lowState.footForce[2]<<" " << lowState.footForce[3]<< " " << std::endl;
    
        MPC_F << recording_MPC[0]<< " "<< recording_MPC[1]<< " "<< recording_MPC[2]<< " " 
        << recording_MPC[3]<< " "<< recording_MPC[4]<< " "<< recording_MPC[5]<< " "
        << recording_MPC[6]<< " "<< recording_MPC[7]<< " "<< recording_MPC[8]<< " "
        << recording_MPC[9]<< " "<< recording_MPC[10]<< " "<< recording_MPC[11]<< " "<< std::endl;
    }

    rate.sleep();

    // std::cout << "L_dist[]: " << L_dist[2] - L_dist[1]<< std::endl;    
    std::cout << "counter: " << counter << std::endl;
    std::cout << "reset counter: " << reset_counter << std::endl;
    counter++;
    std::cout<<"------------------------------------------------"<<std::endl;
    //std::cout<<"counter2: " << counter << std::endl;
    //std::cout << "force" << std::endl;
 
}
 
 b_des.close();
 z_pos.close();
 myfile.close();
 QP.close();
 pos.close();
 MPC_F.close();

 std::cout << "stand up finished" << std::endl;

}

void FSM_State::Jump_MPC(){
    
    ofstream pos; // include QP landing period
    ofstream pos2; // include QP landing period
    // ofstream myfile;
    ofstream QP;
    // ofstream z_pos;
    // ofstream b_des;

    ofstream com_act; // to compare with pos_des
    ofstream com_des;
    ofstream joint_act;
    ofstream joint_des;
    // ofstream foot_act;
    // ofstream foot_des;
    ofstream torque;


    ofstream Voltage;
    ofstream Current;
    pos.open("pos.txt");
    pos2.open("pos2.txt");
    // myfile.open ("ori.txt");
    QP.open("QPsolution.txt");
    // z_pos.open("zPos.txt");
    // b_des.open("b_des_z.txt");
    com_act.open("com_act.txt");
    com_des.open("com_des.txt");
    joint_act.open("joint_act.txt");
    joint_des.open("joint_des.txt");
    torque.open("torque.txt");

    Voltage.open("voltage.txt");
    Current.open("current.txt");
    ros::Rate rate(1000);

    //------------ Read the optimization data from imported files --------------------
    // ------------Stored the data into vectors --------------------------------------
    // Position
    vector<vector<double>> QDes;
    // Torque
    vector<vector<double>> tauDes;
    // foot pos
    vector<vector<double>> pfDes;
    //foot vel
    vector<vector<double>> vfDes;

    vector<vector<double>> FDes;
    
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
    /////////////////////////////////////////////////////////////////////////

    // reset forces and steps to 0
    footFeedForwardForces = Mat34<double>::Zero();
    footstepLocations = Mat34<double>::Zero();
    double minForce = 5;
    double maxForce = 500;
    double contactStateScheduled[4] = {1, 1, 1, 1};

    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

    Vec4<double> contactphase(0.5,0.5,0.5,0.5); 
    _data->_stateEstimator->setContactPhase(contactphase);
    //Timer time;
    //time.start();
    double COM_weights_stance[3] = {5, 5, 10};
    double Base_weights_stance[3] = {10, 10, 20};
    double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
      omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
    double b_control[6];
    int counter = 0;
    std::cout << "counter0: " << counter << std::endl;
       // ros::spinOnce();
       _data->_legController->updateData();
       _data->_stateEstimator->run();
    // position & rpy desired
    for(int i = 0; i < 3; i++){
      //p_des[i] = _data->_stateEstimator->getResult().position(i); 
      v_des[i] = 0;
      omegaDes[i] = 0;
      rpy[i] = 0;
    }
      // p_des[2] = 0.3; // end com height position for robot
     //bool firstLocoRun = true;
    bool runMPC = true;
    

    // Motor Dynamics
    //V= IR+L*dI/dt+ke*w = IR+ke*w = T*R/kt +ke*w = T*R/k+k*w; here Kt=Ke=K
    double R=0.29; 
    double K=0.11; 
    double gr=8.3;
    double voltage[4]; // Motor voltage in Front thigh, front calf, rear thigh, rear calf.
    double current[4]; // Motor current in Front thigh, front calf, rear thigh, rear calf.
    double ff_mpc_torque[12]; // feedforward torque + torque from mpc
    
    double Voltage_max=21.5; // maximum current
    double torque_max=33.5; // torque max

    double Kp = 300;
    double Kd = 3; 
    double Hip_Kp = 300;
    double Hip_Kd =3;
    double Kp_l=5;
    double Kd_l=1;
    double init_Pos[12] = {0, 1.2566, -2.355, 0, 1.2566, -2.355, 0, 1.2566, -2.355, 0, 1.2566, -2.355}; // pose 0, for jump 2D 60cm
    // double init_Pos[12] = {0, 1.1310, -2.1206, 0, 1.1310, -2.1206, 0, 1.1310, -2.1206, 0, 1.1310, -2.1206}; // pose 0, backflip,cs=50,40,50
    double currentPos[12], percent;

    int N_TrajRef=1300;
    int startjump=2000;
    int pose_time=200;



    double box_height=0.2;
    p_des[2] = 0.3+box_height; // end com height position for robot

    // int N_TrajRef=500;
    // int startjump=2000;
    // int pose_time=0;// need to check again


    int idx_pose=N_TrajRef-pose_time;
    int idx=0;

    double FR=0; double FL=0; double RR=0; double RL=0; // contact state

    while(ros::ok()) {
     //std::cout << ros::Time::now() << std::endl;
      // rate.reset();
      _data->_legController->updateData();
      _data->_stateEstimator->run();

      /*================  Standing by PD controller  =================*/

      if (counter <startjump){  // Initialization
          runQP=false;
          std::cout << "FLfootCallback: " << lowState.footForce[1] << std::endl;
          std::cout << "RLfootCallback: " << lowState.footForce[3] << std::endl;
          //double contactStateScheduled[4]={1,1,1,1}; // all legs are stance legs
        for(int i = 0; i < 4; i++){
            _data->_legController->commands[i].kpJoint = Vec3<double>(Hip_Kp,Kp,Kp).asDiagonal();
            _data->_legController->commands[i].kdJoint = Vec3<double>(Hip_Kd,Kd,Kd).asDiagonal();
            _data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
            _data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();
        }

        percent = (double) counter/ startjump; 
           std ::cout << "percent: " << percent << std::endl; 
           for (int i=0; i<4; i++){ 
               for(int j=0; j<3; j++){ 
                 currentPos[3*i+j] = _data->_legController->data[i].q(j); 
                 _data->_legController->commands[i].qDes(j) = currentPos[3*i+j]*(1-percent) + init_Pos[3*i+j]*percent; 
               } 
           } 
      }

      if(counter>startjump && counter<startjump+N_TrajRef-pose_time){

        // for(int i = 0; i <4; i++){
        //       _data->_legController->commands[i].kpJoint << Mat3<double>::Zero();
        //       _data->_legController->commands[i].kdJoint << Mat3<double>::Zero();
        //       _data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
        //       _data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();
        // }

        // for(int i = 0; i < 4; i++){
        //     _data->_legController->commands[i].kpJoint = Vec3<double>(10,10,10).asDiagonal();
        //     _data->_legController->commands[i].kdJoint = Vec3<double>(1,1,1).asDiagonal();
        //     _data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
        //     _data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();
        // }

        // z_pos << _data->_stateEstimator->getResult().position[2] << " " <<  _data->_stateEstimator->getResult().position[1] << std::endl;
        //std::cout << "contact " << _data->_stateEstimator->getResult().contactEstimate << std::endl;
        //ROS_INFO("a");
        //std::cout << "start motion" << std::endl;

        Vec3<double> v_des(0, 0, 0); // v_body
        double yaw_rate = 0;
        double roll = 0;
        double pitch = 0;

        Timer tmpc;
        tmpc.start();

        // _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des, yaw_rate);
        Cmpc.run_Jump(*_data);

        printf("MPC Solve time %f ms\n", tmpc.getMs());
        
        //  for(int i = 0; i < 4; i++){
        //     std::cout << " leg " << i << ":  " << _data->_legController->commands[i].feedforwardForce[2] << std::endl;
        //   }
        // for(int i = 0; i < 4; i++){
        //   QP << _data->_legController->commands[i].feedforwardForce[2] << " ";
        //   }
        // QP << "\n";


        idx= counter-startjump;
        // std::cout<< "idx: " << idx <<std::endl;
        _data->_legController->commands[0].tau << 0, -tauDes[0][idx], -tauDes[1][idx];// front right leg
        _data->_legController->commands[1].tau << _data->_legController->commands[0].tau;// front right leg
        _data->_legController->commands[2].tau << 0, -tauDes[2][idx], -tauDes[3][idx];// rear right leg
        _data->_legController->commands[3].tau << _data->_legController->commands[2].tau;// rear right leg


        // std::cout<< "command tau in fsm:" << _data->_legController->commands[2].tau << std::endl;

        _data->_legController->commands[0].qDes << 0, -QDes[3][idx], -QDes[4][idx]; // front right leg
        _data->_legController->commands[1].qDes << _data->_legController->commands[0].qDes;
        _data->_legController->commands[2].qDes << 0, -QDes[5][idx], -QDes[6][idx];// rear right leg
        _data->_legController->commands[3].qDes << _data->_legController->commands[2].qDes;

        _data->_legController->commands[0].qdDes << 0, -QDes[10][idx], -QDes[11][idx]; // front right leg
        _data->_legController->commands[1].qdDes << _data->_legController->commands[0].qdDes;
        _data->_legController->commands[2].qdDes << 0, -QDes[12][idx], -QDes[13][idx];// rear right leg
        _data->_legController->commands[3].qdDes << _data->_legController->commands[2].qdDes;


        for (int i=0; i<4; i++)
        {

          Vec3<double> legTorque = _data->_legController->commands[i].tau;
          Vec3<double> footForce = _data->_legController->commands[i].feedforwardForce;
          // std::cout << "footForce_chuong" << footForce << std::endl;
          legTorque +=_data->_legController->data[i].J.transpose()*footForce;

          // for (int j =0; j<3; j++){
          //   if (legTorque(j) > 33.5){
          //     legTorque(j)=33.5;
          //   }
          //   if (legTorque(j) < -33.5){
          //     legTorque(j)=-33.5;
          //   }

          //   // std::cout<< "legTorque(j)" << legTorque(j) << std::endl;
          // }

          _data->_legController->commands[i].tau = legTorque;

          // std::cout<< "command tau in FSM:" << _data->_legController->commands[i].tau << std::endl;


        }
        // Compute current based on feedback torque
        current[0]=(1/K)*(1/gr)*(_data ->_legController->data[0].tau(1));
        current[1]=(1/K)*(1/gr)*(_data ->_legController->data[0].tau(2));
        current[2]=(1/K)*(1/gr)*(_data ->_legController->data[2].tau(1));
        current[3]=(1/K)*(1/gr)*(_data ->_legController->data[2].tau(2));
        // Compute voltage

        voltage[0]=R*current[0]+ K* _data ->_legController->data[0].qd(1)*gr;
        voltage[1]=R*current[1]+ K* _data ->_legController->data[0].qd(2)*gr;
        voltage[2]=R*current[2]+ K* _data ->_legController->data[2].qd(1)*gr;
        voltage[3]=R*current[3]+ K* _data ->_legController->data[2].qd(2)*gr;

        runQP = false;
      }

      // Prepare landing.

      if (counter>=startjump+N_TrajRef-pose_time && counter<=startjump+N_TrajRef-pose_time+100){
          // use PD for swing legs

          _data->_legController->commands[0].tau << 0, 0, 0;// front right leg
          _data->_legController->commands[1].tau << 0, 0, 0; // front left leg
          _data->_legController->commands[2].tau << 0, 0, 0;// rear right leg
          _data->_legController->commands[3].tau << 0, 0, 0; // rear left leg

          _data->_legController->commands[0].qDes << 0, -QDes[3][idx_pose]-0.2, -QDes[4][idx_pose]-0.2; // front right leg
          _data->_legController->commands[1].qDes <<_data->_legController->commands[0].qDes; // front left leg
          _data->_legController->commands[2].qDes << 0, -QDes[5][idx_pose]-0.2, -QDes[6][idx_pose-0.2];// rear right leg
          _data->_legController->commands[3].qDes << _data->_legController->commands[2].qDes;; // rear left leg

          _data->_legController->commands[0].qdDes << 0, 0, 0; // front right leg
          _data->_legController->commands[1].qdDes << 0, 0, 0; // front left leg
          _data->_legController->commands[2].qdDes << 0, 0, 0;// rear right leg
          _data->_legController->commands[3].qdDes << 0, 0, 0; // rear left leg

          for (int i=0; i<4; i++){
            _data->_legController->commands[i].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[i].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kp_l;
          }
          runQP=false;
      }

      if(counter>=startjump+N_TrajRef-pose_time+100){

          std::cout<< "QP activated" << std::endl;

          // Get contact state
          FR=lowState.footForce[0];
          FL=lowState.footForce[1];
          RR=lowState.footForce[2];
          RL=lowState.footForce[3];
          if (FR>0){
            FR=1;
            _data->_legController->commands[0].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[0].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
          }
          else{
            FR=0;
          }
          if (FL>0){
            FL=1;
            _data->_legController->commands[1].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[1].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
          }
          else{
            FL=0;
          }
          if (RR>0){
            RR=1;
            _data->_legController->commands[2].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[2].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
          }
          else{
            RR=0;
          }
          if (RL>0){
            RL=1;
            _data->_legController->commands[3].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[3].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
          }
          else{
            RR=0;
          }
          
          double contactStateScheduled[4]={FR,FL,RR,RL};
          std::cout << "contactStateScheduled: " << contactStateScheduled[0] << "," << contactStateScheduled[1]<<"," << contactStateScheduled[2]<< "," << contactStateScheduled[3]<< std::endl;
          runQP=true;
      }

      if(runQP){
        for (int i = 0; i < 4; i++) {
          se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
        }
        // v_des[2] = 0.1;
        for (int i = 0; i < 3; i++) {
        //rpy[i] = 0;
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        
        v_act[i] = _data->_stateEstimator->getResult().vWorld(i);

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaWorld(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vWorld(i);
        
        // Set the translational and orientation gains
        kpCOM[i] = 30;   //_data->controlParameters->kpCOM(i);
        kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
        kpBase[i] = 80;     //_data->controlParameters->kpBase(i);
        kdBase[i] = 20; //  _data->controlParameters->kdBase(i);
        }

        kpCOM[2] = 50;
        kpBase[0] = 300;
        kpBase[1] = 200;

        //Vec3<double> pFeetVec;
        Vec3<double> pFeetVecCOM;

        // Get the foot locations relative to COM
        for (int leg = 0; leg < 4; leg++) {
          // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
          //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
          //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                    //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

          pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
          (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);

          pFeet[leg * 3] = pFeetVecCOM[0];
          pFeet[leg * 3 + 1] = pFeetVecCOM[1];
          pFeet[leg * 3 + 2] = pFeetVecCOM[2];
          //std::cout << "leg" << leg << std::endl;
          //std::cout << "pFeet" << pFeetVecCOM << std::endl;
        }
        _data->_stateEstimator->setContactPhase(contactphase);

        p_des[0] = p_act[0] + (pFeet[0] + pFeet[3] + pFeet[6] + pFeet[9]) / 4.0;
        p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10]) / 4.0;
        //  myfile << "\n";
        // std::cout << j << std::endl;
        //std::cout << "run QP" << std::endl;
        balanceController.set_alpha_control(0.01); 
        balanceController.set_friction(0.2);
        balanceController.set_mass(_data->_quadruped->mass);
        balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
        balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
        balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
        balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
        balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                          O_err, _data->_stateEstimator->getResult().rpy(2));
        // balanceController.print_QPData();
        double fOpt[12];
        balanceController.solveQP_nonThreaded(fOpt);

        std::cout << "zForce leg 0: " << fOpt[2] << std::endl;
        //balanceController.get_b_matrix(b_control);
        //b_des << b_control[2] << "\n";

        // Publish the results over ROS
        // balanceController.publish_data_lcm();

        // Copy the results to the feed forward forces
          
        //_data->_stateEstimator->run();


        for (int leg = 0; leg < 4; leg++) {
            footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
            fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame
            
            _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
            //_data->_stateEstimator->getResult().rBody.transpose() * footFeedForwardForces.col(leg); 
            //std::cout <<"zForce leg "<< _data->_legController->commands[leg].feedforwardForce[2]<< "\n";
            //QP <<_data->_legController->commands[i].feedforwardForce[2] << " ";
        }
    }

    // _data->_legController->updateCommand();

    // if (counter >=startjump && counter<= startjump+N_TrajRef+1000){
    if (counter >=startjump && counter<= startjump+N_TrajRef-pose_time+2000){

      int idx= counter-startjump;

        // Final torque sent to motor = ff+PD
        double final_torque_h0=0+ Kp*(_data ->_legController->commands[0].qDes(0)-_data ->_legController->data[0].q(0))+Kd*(_data ->_legController->commands[0].qdDes(0)-_data ->_legController->data[0].qd(0));
        double final_torque_h1=0+ Kp*(_data ->_legController->commands[1].qDes(0)-_data ->_legController->data[1].q(0))+Kd*(_data ->_legController->commands[1].qdDes(0)-_data ->_legController->data[1].qd(0));
        double final_torque_h2=0+ Kp*(_data ->_legController->commands[2].qDes(0)-_data ->_legController->data[2].q(0))+Kd*(_data ->_legController->commands[2].qdDes(0)-_data ->_legController->data[2].qd(0));
        double final_torque_h3=0+ Kp*(_data ->_legController->commands[3].qDes(0)-_data ->_legController->data[3].q(0))+Kd*(_data ->_legController->commands[3].qdDes(0)-_data ->_legController->data[3].qd(0));
        double final_torque_t0=_data ->_legController->commands[0].tau(1)+ Kp*(_data ->_legController->commands[0].qDes(1)-_data ->_legController->data[0].q(1))+Kd*(_data ->_legController->commands[0].qdDes(1)-_data ->_legController->data[0].qd(1));
        double final_torque_t1=_data ->_legController->commands[1].tau(1)+ Kp*(_data ->_legController->commands[1].qDes(1)-_data ->_legController->data[1].q(1))+Kd*(_data ->_legController->commands[1].qdDes(1)-_data ->_legController->data[1].qd(1));
        double final_torque_t2=_data ->_legController->commands[2].tau(1)+ Kp*(_data ->_legController->commands[2].qDes(1)-_data ->_legController->data[2].q(1))+Kd*(_data ->_legController->commands[2].qdDes(1)-_data ->_legController->data[2].qd(1));
        double final_torque_t3=_data ->_legController->commands[3].tau(1)+ Kp*(_data ->_legController->commands[3].qDes(1)-_data ->_legController->data[3].q(1))+Kd*(_data ->_legController->commands[3].qdDes(1)-_data ->_legController->data[3].qd(1));
        double final_torque_c0=_data ->_legController->commands[0].tau(2)+ Kp*(_data ->_legController->commands[0].qDes(2)-_data ->_legController->data[0].q(2))+Kd*(_data ->_legController->commands[0].qdDes(2)-_data ->_legController->data[0].qd(2));
        double final_torque_c1=_data ->_legController->commands[1].tau(2)+ Kp*(_data ->_legController->commands[1].qDes(2)-_data ->_legController->data[1].q(2))+Kd*(_data ->_legController->commands[1].qdDes(2)-_data ->_legController->data[1].qd(2));
        double final_torque_c2=_data ->_legController->commands[2].tau(2)+ Kp*(_data ->_legController->commands[2].qDes(2)-_data ->_legController->data[2].q(2))+Kd*(_data ->_legController->commands[2].qdDes(2)-_data ->_legController->data[2].qd(2));
        double final_torque_c3=_data ->_legController->commands[3].tau(2)+ Kp*(_data ->_legController->commands[3].qDes(2)-_data ->_legController->data[3].q(2))+Kd*(_data ->_legController->commands[3].qdDes(2)-_data ->_legController->data[3].qd(2));
        

        com_act << _data->_stateEstimator->getResult().vWorld(0) << " " << _data->_stateEstimator->getResult().vWorld(1) << " " <<  _data->_stateEstimator->getResult().vWorld(2) << " " 
        <<_data->_stateEstimator->getResult().position(0) << " " << _data->_stateEstimator->getResult().position(1) << " " << _data->_stateEstimator->getResult().position(2) << " " 
        << _data->_stateEstimator->getResult().rpy(0) << " " << _data->_stateEstimator->getResult().rpy(1) << " " << _data->_stateEstimator->getResult().rpy(2) << std::endl;

        com_des << QDes[7][idx] << " " << 0 << " " << QDes[8][idx] << " " 
        << QDes[0][idx] << " " << 0 << " " << QDes[1][idx] << " "
        << 0 << " " << QDes[2][idx] << " " << 0 << std::endl;

        // _data->_legController->commands[0].tau << 0, -tauDes[0][idx], -tauDes[1][idx];// front right leg
        // _data->_legController->commands[1].tau << _data->_legController->commands[0].tau; // front left leg
        // _data->_legController->commands[2].tau << 0, -tauDes[2][idx], -tauDes[3][idx];// rear right leg
        // _data->_legController->commands[3].tau << _data->_legController->commands[2].tau; // rear left leg

        torque << _data ->_legController->commands[0].tau(0) <<" "<< _data ->_legController->commands[1].tau(0) <<" "<<_data ->_legController->commands[2].tau(0)<<" "<<_data ->_legController->commands[3].tau(0)<<" "
    	  << _data ->_legController->commands[0].tau(1) <<" "<< _data ->_legController->commands[1].tau(1) <<" "<<_data ->_legController->commands[2].tau(1)<<" "<<_data ->_legController->commands[3].tau(1)<<" "
    	  << _data ->_legController->commands[0].tau(2) <<" "<< _data ->_legController->commands[1].tau(2) <<" "<<_data ->_legController->commands[2].tau(2)<<" "<<_data ->_legController->commands[3].tau(2)<<" "
        << final_torque_h0 << " " << final_torque_h1 << " " << final_torque_h2 << " " << final_torque_h3 << " " 
        << final_torque_t0 << " " << final_torque_t1 << " " << final_torque_t2 << " " << final_torque_t3 << " "
        << final_torque_c0 << " " << final_torque_c1 << " " << final_torque_c2 << " " << final_torque_c3 << " " << std::endl;

        joint_act << _data ->_legController->data[0].q(0) <<" "<< _data ->_legController->data[1].q(0) <<" "<<_data ->_legController->data[2].q(0)<<" "<<_data ->_legController->data[3].q(0)<<" "
        <<_data ->_legController->data[0].q(1) <<" "<< _data ->_legController->data[1].q(1) <<" "<<_data ->_legController->data[2].q(1)<<" "<<_data ->_legController->data[3].q(1)<<" " 
        <<_data ->_legController->data[0].q(2) <<" "<< _data ->_legController->data[1].q(2) <<" "<<_data ->_legController->data[2].q(2)<<" "<<_data ->_legController->data[3].q(2)<<" "
        <<_data ->_legController->data[0].qd(0) <<" "<<_data ->_legController->data[1].qd(0) <<" "<<_data ->_legController->data[2].qd(0)<<" "<<_data ->_legController->data[3].qd(0)<< " "
        <<_data ->_legController->data[0].qd(1) <<" "<< _data ->_legController->data[1].qd(1) <<" "<<_data ->_legController->data[2].qd(1)<<" "<<_data ->_legController->data[3].qd(1) << " "
        <<_data ->_legController->data[0].qd(2) <<" "<< _data ->_legController->data[1].qd(2) <<" "<<_data ->_legController->data[2].qd(2)<<" "<<_data ->_legController->data[3].qd(2)<<" "
        <<_data ->_legController->data[0].tau(0) <<" "<< _data ->_legController->data[1].tau(0) <<" "<<_data ->_legController->data[2].tau(0)<<" "  <<_data ->_legController->data[3].tau(0) << " "
        <<_data ->_legController->data[0].tau(1) <<" "<< _data ->_legController->data[1].tau(1) <<" "<<_data ->_legController->data[2].tau(1)<<" "<<_data ->_legController->data[3].tau(1) << " "
        <<_data ->_legController->data[0].tau(2) <<" "<< _data ->_legController->data[1].tau(2) <<" "<<_data ->_legController->data[2].tau(2)<<" "<<_data ->_legController->data[3].tau(2) <<std::endl;
    
        joint_des << _data ->_legController->commands[0].qDes(0) <<" "<< _data ->_legController->commands[1].qDes(0) <<" "<<_data ->_legController->commands[2].qDes(0)<<" "<<_data ->_legController->commands[3].qDes(0)<<" "
    	  << _data ->_legController->commands[0].qDes(1) <<" "<< _data ->_legController->commands[1].qDes(1) <<" "<<_data ->_legController->commands[2].qDes(1)<<" "<<_data ->_legController->commands[3].qDes(1)<<" "
    	  << _data ->_legController->commands[0].qDes(2) <<" "<< _data ->_legController->commands[1].qDes(2) <<" "<<_data ->_legController->commands[2].qDes(2)<<" "<<_data ->_legController->commands[3].qDes(2)<<" "
    	  << _data ->_legController->commands[0].qdDes(0) <<" "<< _data ->_legController->commands[1].qdDes(0) <<" "<<_data ->_legController->commands[2].qdDes(0)<<" "<<_data ->_legController->commands[3].qdDes(0)<<" "
    	  << _data ->_legController->commands[0].qdDes(1) <<" "<< _data ->_legController->commands[1].qdDes(1) <<" "<<_data ->_legController->commands[2].qdDes(1)<<" "<<_data ->_legController->commands[3].qdDes(1)<<" "
    	  << _data ->_legController->commands[0].qdDes(2) <<" "<< _data ->_legController->commands[1].qdDes(2) <<" "<<_data ->_legController->commands[2].qdDes(2)<<" "<<_data ->_legController->commands[3].qdDes(2)<<" "
    	  << 0 <<" "<< 0 <<" "<<0<<" "<<0<<" "
    	  << -tauDes[0][idx] <<" "<< -tauDes[0][idx] <<" "<<-tauDes[2][idx] <<" "<<-tauDes[2][idx]<<" "
    	  << -tauDes[1][idx] <<" "<< -tauDes[1][idx] <<" "<<-tauDes[3][idx]<<" "<<-tauDes[3][idx]<<std::endl;

        Voltage << voltage[0] << " " << voltage[1] << " " << voltage[2] << " " << voltage[3] <<" " << std::endl;
        Current << current[0] << " " << current[1] << " " << current[2] << " " << current[3] <<" " << std::endl;

        pos << _data->_stateEstimator->getResult().vWorld(0) << " " << _data->_stateEstimator->getResult().vWorld(1) << " " <<  _data->_stateEstimator->getResult().vWorld(2) << " " 
        <<_data->_stateEstimator->getResult().position(0) << " " << _data->_stateEstimator->getResult().position(1) << " " << _data->_stateEstimator->getResult().position(2) << " " 
        << _data->_stateEstimator->getResult().rpy(0) << " " << _data->_stateEstimator->getResult().rpy(1) << " " << _data->_stateEstimator->getResult().rpy(2) << " " 
        << _data->_legController->commands[0].force_MPC(0) << " " <<  _data->_legController->commands[1].force_MPC(0) << " " <<  _data->_legController->commands[2].force_MPC(0) << " " <<  _data->_legController->commands[3].force_MPC(0) << " " 
        << _data->_legController->commands[0].force_MPC(1) << " " <<  _data->_legController->commands[1].force_MPC(1) << " " <<  _data->_legController->commands[2].force_MPC(1) << " " <<  _data->_legController->commands[3].force_MPC(1) << " " 
        << _data->_legController->commands[0].force_MPC(2) << " " <<  _data->_legController->commands[1].force_MPC(2) << " " <<  _data->_legController->commands[2].force_MPC(2) << " " <<  _data->_legController->commands[3].force_MPC(2) << " " 
        << -FDes[0][idx]/2 <<" "<< -FDes[1][idx]/2 <<" "<< -FDes[2][idx]/2 <<" "<< -FDes[3][idx]/2 << " "
        << _data->_legController->commands[0].feedforwardForce(0) << " " <<  _data->_legController->commands[1].feedforwardForce(0) << " " <<  _data->_legController->commands[2].feedforwardForce(0) << " " <<  _data->_legController->commands[3].feedforwardForce(0) << " " 
        << _data->_legController->commands[0].feedforwardForce(1) << " " <<  _data->_legController->commands[1].feedforwardForce(1) << " " <<  _data->_legController->commands[2].feedforwardForce(1) << " " <<  _data->_legController->commands[3].feedforwardForce(1) << " " 
        << _data->_legController->commands[0].feedforwardForce(2) << " " <<  _data->_legController->commands[1].feedforwardForce(2) << " " <<  _data->_legController->commands[2].feedforwardForce(2) << " " <<  _data->_legController->commands[3].feedforwardForce(2) << " " 
        << _data->_stateEstimator->getResult().contactEstimate(0) << " " << _data->_stateEstimator->getResult().contactEstimate(1) << " " << _data->_stateEstimator->getResult().contactEstimate(2) << " " << _data->_stateEstimator->getResult().contactEstimate(3)<<" "
        << lowState.footForce[0] << " " << lowState.footForce[1] <<" "<< lowState.footForce[2]<<" " << lowState.footForce[3]<< " " << std::endl;


    }


    _data->_legController->updateCommand_Jump2D();


    rate.sleep();

    std::cout << "counter1: " << counter << std::endl;
    counter++;
    //std::cout<<"counter2: " << counter << std::endl;
    //std::cout << "force" << std::endl;
 
}
 

 QP.close();
 pos.close();
 com_act.close();
 com_des.close();
 joint_act.close();
 joint_des.close();
 torque.close();
 Voltage.close();
 Current.close();

}


void FSM_State::Jump_MPC_testPSU(){
    
    ofstream pos; // include QP landing period
    // ofstream myfile;
    ofstream QP;
    // ofstream z_pos;
    // ofstream b_des;

    ofstream com_act; // to compare with pos_des
    ofstream com_des;
    ofstream joint_act;
    ofstream joint_des;
    // ofstream foot_act;
    // ofstream foot_des;
    ofstream torque;


    ofstream Voltage;
    ofstream Current;
    pos.open("pos.txt");
    // myfile.open ("ori.txt");
    QP.open("QPsolution.txt");
    // z_pos.open("zPos.txt");
    // b_des.open("b_des_z.txt");
    com_act.open("com_act.txt");
    com_des.open("com_des.txt");
    joint_act.open("joint_act.txt");
    joint_des.open("joint_des.txt");
    torque.open("torque.txt");

    Voltage.open("voltage.txt");
    Current.open("current.txt");
    ros::Rate rate(1000);

    //------------ Read the optimization data from imported files --------------------
    // ------------Stored the data into vectors --------------------------------------
    // Position
    vector<vector<double>> QDes;
    // Torque
    vector<vector<double>> tauDes;
    // foot pos
    vector<vector<double>> pfDes;
    //foot vel
    vector<vector<double>> vfDes;
    
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
    /////////////////////////////////////////////////////////////////////////

    // reset forces and steps to 0
    footFeedForwardForces = Mat34<double>::Zero();
    footstepLocations = Mat34<double>::Zero();
    double minForce = 5;
    double maxForce = 500;
    double contactStateScheduled[4] = {1, 1, 1, 1};

    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

    Vec4<double> contactphase(0.5,0.5,0.5,0.5); 
    _data->_stateEstimator->setContactPhase(contactphase);
    //Timer time;
    //time.start();
    double COM_weights_stance[3] = {5, 5, 10};
    double Base_weights_stance[3] = {10, 10, 20};
    double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
      omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
    double b_control[6];
    int counter = 0;
    std::cout << "counter0: " << counter << std::endl;
       // ros::spinOnce();
       _data->_legController->updateData();
       _data->_stateEstimator->run();
    // position & rpy desired
    for(int i = 0; i < 3; i++){
      //p_des[i] = _data->_stateEstimator->getResult().position(i); 
      v_des[i] = 0;
      omegaDes[i] = 0;
      rpy[i] = 0;
    }
      // p_des[2] = 0.3; // end com height position for robot
     //bool firstLocoRun = true;
    bool runMPC = true;
    

    // Motor Dynamics
    //V= IR+L*dI/dt+ke*w = IR+ke*w = T*R/kt +ke*w = T*R/k+k*w; here Kt=Ke=K
    double R=0.29; 
    double K=0.11; 
    double gr=8.3;
    double voltage[4]; // Motor voltage in Front thigh, front calf, rear thigh, rear calf.
    double current[4]; // Motor current in Front thigh, front calf, rear thigh, rear calf.
    double Voltage_max=21.5; // maximum current
    double torque_max=33.5; // torque max

    double Kp = 500;
    double Kd = 5; 
    double Hip_Kp = 500;
    double Hip_Kd =5;
    double Kp_l=5;
    double Kd_l=1;
    double init_Pos[12] = {0, 1.2566, -2.355, 0, 1.2566, -2.355, 0, 1.2566, -2.355, 0, 1.2566, -2.355}; // pose 0, for jump 2D 60cm
    // double init_Pos[12] = {0, 1.1310, -2.1206, 0, 1.1310, -2.1206, 0, 1.1310, -2.1206, 0, 1.1310, -2.1206}; // pose 0, backflip,cs=50,40,50
    double currentPos[12], percent;

    int N_TrajRef=1200;
    int startjump=2000;
    int pose_time=200;


    // int N_TrajRef=500;
    // int pose_time=0;

    double box_height=0.2;
    p_des[2] = 0.3+box_height; // end com height position for robot

    // int N_TrajRef=500;
    // int startjump=2000;
    // int pose_time=0;// need to check again


    int idx_pose=N_TrajRef-pose_time;
    int idx=0;

    double FR=0; double FL=0; double RR=0; double RL=0; // contact state

    while(ros::ok()) {
     //std::cout << ros::Time::now() << std::endl;
      // rate.reset();
      _data->_legController->updateData();
      _data->_stateEstimator->run();

      /*================  Standing by PD controller  =================*/

      if (counter <startjump){  // Initialization
          runQP=false;
          std::cout << "FLfootCallback: " << lowState.footForce[1] << std::endl;
          std::cout << "RLfootCallback: " << lowState.footForce[3] << std::endl;
          //double contactStateScheduled[4]={1,1,1,1}; // all legs are stance legs
        for(int i = 0; i < 4; i++){
            _data->_legController->commands[i].kpJoint = Vec3<double>(Hip_Kp,Kp,Kp).asDiagonal();
            _data->_legController->commands[i].kdJoint = Vec3<double>(Hip_Kd,Kd,Kd).asDiagonal();
            _data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
            _data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();
        }

        percent = (double) counter/ startjump; 
           std ::cout << "percent: " << percent << std::endl; 
           for (int i=0; i<4; i++){ 
               for(int j=0; j<3; j++){ 
                 currentPos[3*i+j] = _data->_legController->data[i].q(j); 
                 _data->_legController->commands[i].qDes(j) = currentPos[3*i+j]*(1-percent) + init_Pos[3*i+j]*percent; 
               } 
           } 
      }
 

      /*======================== MPC locomotion ====================*/
      if(counter>startjump && counter<startjump+N_TrajRef-pose_time){

        // for(int i = 0; i <4; i++){
        //       _data->_legController->commands[i].kpJoint << Mat3<double>::Zero();
        //       _data->_legController->commands[i].kdJoint << Mat3<double>::Zero();
        //       _data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
        //       _data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();
        // }

        // for(int i = 0; i < 4; i++){
        //     _data->_legController->commands[i].kpJoint = Vec3<double>(Hip_Kp,Kp,Kp).asDiagonal();
        //     _data->_legController->commands[i].kdJoint = Vec3<double>(Hip_Kd,Kd,Kd).asDiagonal();
        //     _data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
        //     _data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();
        // }

        // z_pos << _data->_stateEstimator->getResult().position[2] << " " <<  _data->_stateEstimator->getResult().position[1] << std::endl;
        //std::cout << "contact " << _data->_stateEstimator->getResult().contactEstimate << std::endl;
        //ROS_INFO("a");
        //std::cout << "start motion" << std::endl;

        Vec3<double> v_des(0, 0, 0); // v_body
        double yaw_rate = 0;
        double roll = 0;
        double pitch = 0;
        //  std::cout << "state estimate rpy ";
        //  for(int i = 0; i < 3; i++){
        //  std::cout << _data->_stateEstimator->getResult().rpy(i) << " " ; 
        // }
        
        _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des, yaw_rate);
        Cmpc.run_Jump(*_data);
        
        //  for(int i = 0; i < 4; i++){
        //     std::cout << " leg " << i << ":  " << _data->_legController->commands[i].feedforwardForce[2] << std::endl;
        //   }
        // for(int i = 0; i < 4; i++){
        //   QP << _data->_legController->commands[i].feedforwardForce[2] << " ";
        //   }
        // QP << "\n";


        idx= counter-startjump;
        std::cout<< "idx: " << idx <<std::endl;

        
        _data->_legController->commands[0].tau << 0, -tauDes[0][idx], -tauDes[1][idx];// front right leg
        _data->_legController->commands[1].tau << _data->_legController->commands[0].tau;// front right leg
        _data->_legController->commands[2].tau << 0, -tauDes[2][idx], -tauDes[3][idx];// rear right leg
        _data->_legController->commands[3].tau << _data->_legController->commands[2].tau;// rear right leg

        _data->_legController->commands[0].qDes << 0, -QDes[3][idx], -QDes[4][idx]; // front right leg
        _data->_legController->commands[1].qDes << _data->_legController->commands[0].qDes;
        _data->_legController->commands[2].qDes << 0, -QDes[5][idx], -QDes[6][idx];// rear right leg
         _data->_legController->commands[3].qDes << _data->_legController->commands[2].qDes;

        _data->_legController->commands[0].qdDes << 0, -QDes[10][idx], -QDes[11][idx]; // front right leg
        _data->_legController->commands[1].qdDes << _data->_legController->commands[0].qdDes;
        _data->_legController->commands[2].qdDes << 0, -QDes[12][idx], -QDes[13][idx];// rear right leg
        _data->_legController->commands[3].qdDes << _data->_legController->commands[2].qdDes;

        // // Compute current 
        // current[0]=(1/K)*(1/gr)*(_data->_legController->commands[0].tau(1) + Kp*(_data->_legController->commands[0].qDes(1)-_data ->_legController->data[0].q(1))+Kd*(_data->_legController->commands[0].qdDes(1)-_data ->_legController->data[0].qd(1)));
        // current[1]=(1/K)*(1/gr)*(_data->_legController->commands[0].tau(2) + Kp*(_data->_legController->commands[0].qDes(2)-_data ->_legController->data[0].q(2))+Kd*(_data->_legController->commands[0].qdDes(2)-_data ->_legController->data[0].qd(2)));
        // current[2]=(1/K)*(1/gr)*(_data->_legController->commands[2].tau(1) + Kp*(_data->_legController->commands[2].qDes(1)-_data ->_legController->data[2].q(1))+Kd*(_data->_legController->commands[2].qdDes(1)-_data ->_legController->data[2].qd(1)));
        // current[3]=(1/K)*(1/gr)*(_data->_legController->commands[2].tau(2) + Kp*(_data->_legController->commands[2].qDes(2)-_data ->_legController->data[2].q(2))+Kd*(_data->_legController->commands[2].qdDes(2)-_data ->_legController->data[0].qd(2)));
        

         // Compute current based on feedback torque
        current[0]=(1/K)*(1/gr)*(_data ->_legController->data[0].tau(1));
        current[1]=(1/K)*(1/gr)*(_data ->_legController->data[0].tau(2));
        current[2]=(1/K)*(1/gr)*(_data ->_legController->data[2].tau(1));
        current[3]=(1/K)*(1/gr)*(_data ->_legController->data[2].tau(2));
        // Compute voltage

        voltage[0]=R*current[0]+ K* _data ->_legController->data[0].qd(1)*gr;
        voltage[1]=R*current[1]+ K* _data ->_legController->data[0].qd(2)*gr;
        voltage[2]=R*current[2]+ K* _data ->_legController->data[2].qd(1)*gr;
        voltage[3]=R*current[3]+ K* _data ->_legController->data[2].qd(2)*gr;
        
      //   // Limit total torque applied to joints
      //   double final_torque_t0=_data ->_legController->commands[0].tau(1)+ Kp*(_data ->_legController->commands[0].qDes(1)-_data ->_legController->data[0].q(1))+Kd*(_data ->_legController->commands[0].qdDes(1)-_data ->_legController->data[0].qd(1));
      //   double final_torque_c0=_data ->_legController->commands[0].tau(2)+ Kp*(_data ->_legController->commands[0].qDes(2)-_data ->_legController->data[0].q(2))+Kd*(_data ->_legController->commands[0].qdDes(2)-_data ->_legController->data[0].qd(2));
        
      //   double final_torque_t2=_data ->_legController->commands[2].tau(1)+ Kp*(_data ->_legController->commands[2].qDes(1)-_data ->_legController->data[2].q(1))+Kd*(_data ->_legController->commands[2].qdDes(1)-_data ->_legController->data[2].qd(1));
      //   double final_torque_c2=_data ->_legController->commands[2].tau(2)+ Kp*(_data ->_legController->commands[2].qDes(2)-_data ->_legController->data[2].q(2))+Kd*(_data ->_legController->commands[2].qdDes(2)-_data ->_legController->data[2].qd(2));


      //   // front thigh
      //   if (final_torque_t0>=torque_max){
      //       _data->_legController->commands[0].tau(1)=torque_max-Kp*(_data ->_legController->commands[0].qDes(1)-_data ->_legController->data[0].q(1))-Kd*(_data ->_legController->commands[0].qdDes(1)-_data ->_legController->data[0].qd(1));
      //       _data->_legController->commands[1].tau(1) = _data->_legController->commands[0].tau(1);
      //   }
      //   if (final_torque_t0<=-torque_max){
      //       _data->_legController->commands[0].tau(1)=-torque_max-Kp*(_data ->_legController->commands[0].qDes(1)-_data ->_legController->data[0].q(1))-Kd*(_data ->_legController->commands[0].qdDes(1)-_data ->_legController->data[0].qd(1));
      //       _data->_legController->commands[1].tau(1) = _data->_legController->commands[0].tau(1);
      //  }

      //  // front calf

      //   if(final_torque_c0>=torque_max){
      //       _data ->_legController->commands[0].tau(2)= torque_max- Kp*(_data ->_legController->commands[0].qDes(2)-_data ->_legController->data[0].q(2))-Kd*(_data ->_legController->commands[0].qdDes(2)-_data ->_legController->data[0].qd(2));
      //       _data->_legController->commands[1].tau(2) = _data->_legController->commands[0].tau(2);
      //   } 
      //   if(final_torque_c0<=-torque_max){
      //       _data ->_legController->commands[0].tau(2)= -torque_max- Kp*(_data ->_legController->commands[0].qDes(2)-_data ->_legController->data[0].q(2))-Kd*(_data ->_legController->commands[0].qdDes(2)-_data ->_legController->data[0].qd(2));
      //       _data->_legController->commands[1].tau(2) = _data->_legController->commands[0].tau(2);
      //   } 

      //   // rear thigh

      //   if (final_torque_t2>=torque_max){
      //       _data->_legController->commands[2].tau(1)=torque_max-Kp*(_data ->_legController->commands[2].qDes(1)-_data ->_legController->data[2].q(1))- Kd*(_data ->_legController->commands[2].qdDes(1)-_data ->_legController->data[2].qd(1));
      //       _data->_legController->commands[3].tau(1) = _data->_legController->commands[2].tau(1);
      //   }
      //   if (final_torque_t2<=-torque_max){
      //       _data->_legController->commands[2].tau(1)=-torque_max-Kp*(_data ->_legController->commands[2].qDes(1)-_data ->_legController->data[2].q(1))- Kd*(_data ->_legController->commands[2].qdDes(1)-_data ->_legController->data[2].qd(1));
      //       _data->_legController->commands[3].tau(1) = _data->_legController->commands[2].tau(1);
      //  }

      //  // rear calf
      
      //  if (final_torque_c2>=torque_max){
      //       _data->_legController->commands[2].tau(2)=torque_max-Kp*(_data ->_legController->commands[2].qDes(2)-_data ->_legController->data[2].q(2))-Kd*(_data ->_legController->commands[2].qdDes(2)-_data ->_legController->data[2].qd(2));
      //       _data->_legController->commands[3].tau(2) = _data->_legController->commands[2].tau(2);
      //   }
      //   if (final_torque_c2<=-torque_max){
      //       _data->_legController->commands[2].tau(2)=-torque_max-Kp*(_data ->_legController->commands[2].qDes(2)-_data ->_legController->data[2].q(2))-Kd*(_data ->_legController->commands[2].qdDes(2)-_data ->_legController->data[2].qd(2));
      //       _data->_legController->commands[3].tau(2) = _data->_legController->commands[2].tau(2);
      //  }




                

        // // Limit voltage --> limit ff torque apply to the joint

        // // front thigh

        // if(voltage[0]>=Voltage_max){
        //   _data->_legController->commands[0].tau(1) = (1/R)*(Voltage_max-K* _data ->_legController->data[0].qd(1)*gr)*(K*gr); // total torque limit
        //   _data->_legController->commands[0].tau(1) -= Kp*(_data->_legController->commands[0].qDes(1)-_data ->_legController->data[0].q(1))+Kd*(_data->_legController->commands[0].qdDes(1)-_data ->_legController->data[0].qd(1));
        //   _data->_legController->commands[1].tau(1) = _data->_legController->commands[0].tau(1);
        //   // voltage[0]=Voltage_max;
        //   // current[0]=(1/R)*(Voltage_max-K* _data ->_legController->data[0].qd(1)*gr);
        // }
        // if(voltage[0]<=-Voltage_max){
        //   _data->_legController->commands[0].tau(1) = (-1/R)*(Voltage_max-K* _data ->_legController->data[0].qd(1)*gr)*(K*gr); // total torque limit
        //   _data->_legController->commands[0].tau(1) -= Kp*(_data->_legController->commands[0].qDes(1)-_data ->_legController->data[0].q(1))+Kd*(_data->_legController->commands[0].qdDes(1)-_data ->_legController->data[0].qd(1));
        //   _data->_legController->commands[1].tau(1) = _data->_legController->commands[0].tau(1);
        //   // voltage[0]=-Voltage_max;
        //   // current[0]=(1/R)*(-Voltage_max-K* _data ->_legController->data[0].qd(1)*gr);
        // }

        // // Front calf
        
        // if(voltage[1]>=Voltage_max){
        //   _data->_legController->commands[0].tau(2) = (1/R)*(Voltage_max-K* _data ->_legController->data[0].qd(2)*gr)*(K*gr); // total torque limit
        //   _data->_legController->commands[0].tau(2) -= Kp*(_data->_legController->commands[0].qDes(2)-_data ->_legController->data[0].q(2))+Kd*(_data->_legController->commands[0].qdDes(2)-_data ->_legController->data[0].qd(2));
        //   _data->_legController->commands[1].tau(2) = _data->_legController->commands[0].tau(2);
        //   // voltage[1]=Voltage_max;
        //   // current[1]=(1/R)*(Voltage_max-K* _data ->_legController->data[0].qd(2)*gr);
        
        // }
        // if(voltage[1]<=-Voltage_max){
        //   _data->_legController->commands[0].tau(2) = (-1/R)*(Voltage_max-K* _data ->_legController->data[0].qd(2)*gr)*(K*gr); // total torque limit
        //   _data->_legController->commands[0].tau(2) -= Kp*(_data->_legController->commands[0].qDes(2)-_data ->_legController->data[0].q(2))+Kd*(_data->_legController->commands[0].qdDes(2)-_data ->_legController->data[0].qd(2));
        //   _data->_legController->commands[1].tau(2) = _data->_legController->commands[0].tau(2);
        //   // voltage[1]=-Voltage_max;
        //   // current[1]=(1/R)*(-Voltage_max-K* _data ->_legController->data[0].qd(2)*gr);
        // }

        // // Rear thigh

        // if(voltage[2]>=Voltage_max){
        //   _data->_legController->commands[2].tau(1) = (1/R)*(Voltage_max-K* _data ->_legController->data[2].qd(1)*gr)*(K*gr); // total torque limit
        //   _data->_legController->commands[2].tau(1) -= Kp*(_data->_legController->commands[2].qDes(1)-_data ->_legController->data[2].q(1))+Kd*(_data->_legController->commands[2].qdDes(1)-_data ->_legController->data[2].qd(1));
        //   _data->_legController->commands[3].tau(1) = _data->_legController->commands[2].tau(1);
        //   // voltage[2]=Voltage_max;
        //   // current[2]=(1/R)*(Voltage_max-K* _data ->_legController->data[2].qd(1)*gr);
        // }
        // if(voltage[2]<=-Voltage_max){
        //   _data->_legController->commands[2].tau(1) = (-1/R)*(Voltage_max-K* _data ->_legController->data[2].qd(1)*gr)*(K*gr); // total torque limit
        //   _data->_legController->commands[2].tau(1) -= Kp*(_data->_legController->commands[2].qDes(1)-_data ->_legController->data[2].q(1))+Kd*(_data->_legController->commands[2].qdDes(1)-_data ->_legController->data[2].qd(1));
        //   _data->_legController->commands[3].tau(1) = _data->_legController->commands[2].tau(1);
        //   // voltage[2]=-Voltage_max;
        //   // current[2]=(1/R)*(-Voltage_max-K* _data ->_legController->data[2].qd(1)*gr);
        // }

        // // Rear calf

        // if(voltage[3]>=Voltage_max){
        //   _data->_legController->commands[2].tau(2) = (1/R)*(Voltage_max-K* _data ->_legController->data[2].qd(2)*gr)*(K*gr); // total torque limit
        //   _data->_legController->commands[2].tau(2) -= Kp*(_data->_legController->commands[2].qDes(2)-_data ->_legController->data[2].q(2))+Kd*(_data->_legController->commands[2].qdDes(2)-_data ->_legController->data[2].qd(2));
        //   _data->_legController->commands[3].tau(2) = _data->_legController->commands[2].tau(2);
        //   // voltage[3]=Voltage_max;
        //   // current[3]=(1/R)*(Voltage_max-K* _data ->_legController->data[2].qd(2)*gr);
        
        // }
        // if(voltage[3]<=-Voltage_max){
        //   _data->_legController->commands[2].tau(2) = (-1/R)*(Voltage_max-K* _data ->_legController->data[2].qd(2)*gr)*(K*gr); // total torque limit
        //   _data->_legController->commands[2].tau(2) -= Kp*(_data->_legController->commands[2].qDes(2)-_data ->_legController->data[2].q(2))+Kd*(_data->_legController->commands[2].qdDes(2)-_data ->_legController->data[2].qd(2));
        //   _data->_legController->commands[3].tau(2) = _data->_legController->commands[2].tau(2);
        //   // voltage[3]=-Voltage_max;
        //   // current[3]=(1/R)*(-Voltage_max-K* _data ->_legController->data[2].qd(2)*gr);
        // }

        runQP = false;
      }

      // if (counter>=startjump+N_TrajRef-pose_time && counter<=startjump+N_TrajRef-pose_time+100){
      //     // use PD for swing legs

      //     _data->_legController->commands[0].tau << 0, 0, 0;// front right leg
      //     _data->_legController->commands[1].tau << 0, 0, 0; // front left leg
      //     _data->_legController->commands[2].tau << 0, 0, 0;// rear right leg
      //     _data->_legController->commands[3].tau << 0, 0, 0; // rear left leg

      //     _data->_legController->commands[0].qDes << 0, -QDes[3][idx_pose]-0.2, -QDes[4][idx_pose]-0.2; // front right leg
      //     _data->_legController->commands[1].qDes <<_data->_legController->commands[0].qDes; // front left leg
      //     _data->_legController->commands[2].qDes << 0, -QDes[5][idx_pose]-0.2, -QDes[6][idx_pose-0.2];// rear right leg
      //     _data->_legController->commands[3].qDes << _data->_legController->commands[2].qDes;; // rear left leg

      //     _data->_legController->commands[0].qdDes << 0, 0, 0; // front right leg
      //     _data->_legController->commands[1].qdDes << 0, 0, 0; // front left leg
      //     _data->_legController->commands[2].qdDes << 0, 0, 0;// rear right leg
      //     _data->_legController->commands[3].qdDes << 0, 0, 0; // rear left leg

      //     for (int i=0; i<4; i++){
      //       _data->_legController->commands[i].kpJoint << Hip_Kp, 0, 0,
      //                                                     0, Kp, 0,
      //                                                     0, 0, Kp;
      //       _data->_legController->commands[i].kdJoint << Hip_Kd, 0, 0,
      //                                                     0, Kd, 0,
      //                                                     0, 0, Kp;

      //     }
      //     runQP=false;
      // }


      

      if(counter>=startjump+N_TrajRef-pose_time){

          std::cout<< "QP activated" << std::endl;

          // Get contact state
          FR=lowState.footForce[0];
          FL=lowState.footForce[1];
          RR=lowState.footForce[2];
          RL=lowState.footForce[3];
          if (FR>0){
            FR=1;
            _data->_legController->commands[0].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[0].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
          }
          else{
            FR=0;
          }
          if (FL>0){
            FL=1;
            _data->_legController->commands[1].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[1].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
          }
          else{
            FL=0;
          }
          if (RR>0){
            RR=1;
            _data->_legController->commands[2].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[2].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
          }
          else{
            RR=0;
          }
          if (RL>0){
            RL=1;
            _data->_legController->commands[3].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[3].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
          }
          else{
            RR=0;
          }
          
          double contactStateScheduled[4]={FR,FL,RR,RL};
          std::cout << "contactStateScheduled: " << contactStateScheduled[0] << "," << contactStateScheduled[1]<<"," << contactStateScheduled[2]<< "," << contactStateScheduled[3]<< std::endl;
          runQP=true;
      }

      if(runQP){
        for (int i = 0; i < 4; i++) {
          se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
        }
        // v_des[2] = 0.1;
        for (int i = 0; i < 3; i++) {
        //rpy[i] = 0;
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        
        v_act[i] = _data->_stateEstimator->getResult().vWorld(i);

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaWorld(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vWorld(i);
        
        // Set the translational and orientation gains
        kpCOM[i] = 30;   //_data->controlParameters->kpCOM(i);
        kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
        kpBase[i] = 80;     //_data->controlParameters->kpBase(i);
        kdBase[i] = 20; //  _data->controlParameters->kdBase(i);
        }

        kpCOM[2] = 50;
        kpBase[0] = 300;
        kpBase[1] = 200;

        //Vec3<double> pFeetVec;
        Vec3<double> pFeetVecCOM;

        // Get the foot locations relative to COM
        for (int leg = 0; leg < 4; leg++) {
          // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
          //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
          //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                    //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

          pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
          (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);

          pFeet[leg * 3] = pFeetVecCOM[0];
          pFeet[leg * 3 + 1] = pFeetVecCOM[1];
          pFeet[leg * 3 + 2] = pFeetVecCOM[2];
          //std::cout << "leg" << leg << std::endl;
          //std::cout << "pFeet" << pFeetVecCOM << std::endl;
        }
        _data->_stateEstimator->setContactPhase(contactphase);

        p_des[0] = p_act[0] + (pFeet[0] + pFeet[3] + pFeet[6] + pFeet[9]) / 4.0;
        p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10]) / 4.0;
        //  myfile << "\n";
        // std::cout << j << std::endl;
        //std::cout << "run QP" << std::endl;
        balanceController.set_alpha_control(0.01); 
        balanceController.set_friction(0.2);
        balanceController.set_mass(_data->_quadruped->mass);
        balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
        balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
        balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
        balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
        balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                          O_err, _data->_stateEstimator->getResult().rpy(2));
        // balanceController.print_QPData();
        double fOpt[12];
        balanceController.solveQP_nonThreaded(fOpt);

        std::cout << "zForce leg 0: " << fOpt[2] << std::endl;
        //balanceController.get_b_matrix(b_control);
        //b_des << b_control[2] << "\n";

        // Publish the results over ROS
        // balanceController.publish_data_lcm();

        // Copy the results to the feed forward forces
          
        //_data->_stateEstimator->run();


        for (int leg = 0; leg < 4; leg++) {
            footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
            fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame
            
            _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
            //_data->_stateEstimator->getResult().rBody.transpose() * footFeedForwardForces.col(leg); 
            //std::cout <<"zForce leg "<< _data->_legController->commands[leg].feedforwardForce[2]<< "\n";
            //QP <<_data->_legController->commands[i].feedforwardForce[2] << " ";
        }
    }


    // _data->_legController->updateCommand();

    
    if (counter >=startjump && counter<= startjump+N_TrajRef-pose_time){

      int idx= counter-startjump;

        // Final torque sent to motor = ff+PD
        double final_torque_h0=0+ Kp*(_data ->_legController->commands[0].qDes(0)-_data ->_legController->data[0].q(0))+Kd*(_data ->_legController->commands[0].qdDes(0)-_data ->_legController->data[0].qd(0));
        double final_torque_h1=0+ Kp*(_data ->_legController->commands[1].qDes(0)-_data ->_legController->data[1].q(0))+Kd*(_data ->_legController->commands[1].qdDes(0)-_data ->_legController->data[1].qd(0));
        double final_torque_h2=0+ Kp*(_data ->_legController->commands[2].qDes(0)-_data ->_legController->data[2].q(0))+Kd*(_data ->_legController->commands[2].qdDes(0)-_data ->_legController->data[2].qd(0));
        double final_torque_h3=0+ Kp*(_data ->_legController->commands[3].qDes(0)-_data ->_legController->data[3].q(0))+Kd*(_data ->_legController->commands[3].qdDes(0)-_data ->_legController->data[3].qd(0));
        double final_torque_t0=_data ->_legController->commands[0].tau(1)+ Kp*(_data ->_legController->commands[0].qDes(1)-_data ->_legController->data[0].q(1))+Kd*(_data ->_legController->commands[0].qdDes(1)-_data ->_legController->data[0].qd(1));
        double final_torque_t1=_data ->_legController->commands[1].tau(1)+ Kp*(_data ->_legController->commands[1].qDes(1)-_data ->_legController->data[1].q(1))+Kd*(_data ->_legController->commands[1].qdDes(1)-_data ->_legController->data[1].qd(1));
        double final_torque_t2=_data ->_legController->commands[2].tau(1)+ Kp*(_data ->_legController->commands[2].qDes(1)-_data ->_legController->data[2].q(1))+Kd*(_data ->_legController->commands[2].qdDes(1)-_data ->_legController->data[2].qd(1));
        double final_torque_t3=_data ->_legController->commands[3].tau(1)+ Kp*(_data ->_legController->commands[3].qDes(1)-_data ->_legController->data[3].q(1))+Kd*(_data ->_legController->commands[3].qdDes(1)-_data ->_legController->data[3].qd(1));
        double final_torque_c0=_data ->_legController->commands[0].tau(2)+ Kp*(_data ->_legController->commands[0].qDes(2)-_data ->_legController->data[0].q(2))+Kd*(_data ->_legController->commands[0].qdDes(2)-_data ->_legController->data[0].qd(2));
        double final_torque_c1=_data ->_legController->commands[1].tau(2)+ Kp*(_data ->_legController->commands[1].qDes(2)-_data ->_legController->data[1].q(2))+Kd*(_data ->_legController->commands[1].qdDes(2)-_data ->_legController->data[1].qd(2));
        double final_torque_c2=_data ->_legController->commands[2].tau(2)+ Kp*(_data ->_legController->commands[2].qDes(2)-_data ->_legController->data[2].q(2))+Kd*(_data ->_legController->commands[2].qdDes(2)-_data ->_legController->data[2].qd(2));
        double final_torque_c3=_data ->_legController->commands[3].tau(2)+ Kp*(_data ->_legController->commands[3].qDes(2)-_data ->_legController->data[3].q(2))+Kd*(_data ->_legController->commands[3].qdDes(2)-_data ->_legController->data[3].qd(2));
        

        // com_act << _data->_stateEstimator->getResult().vWorld(0) << " " << _data->_stateEstimator->getResult().vWorld(1) << " " <<  _data->_stateEstimator->getResult().vWorld(2) << " " 
        // <<_data->_stateEstimator->getResult().position(0) << " " << _data->_stateEstimator->getResult().position(1) << " " << _data->_stateEstimator->getResult().position(2) << " " 
        // << _data->_stateEstimator->getResult().rpy(0) << " " << _data->_stateEstimator->getResult().rpy(1) << " " << _data->_stateEstimator->getResult().rpy(2) << std::endl;

        // com_des << QDes[7][idx] << " " << 0 << " " << QDes[8][idx] << " " 
        // << QDes[0][idx] << " " << 0 << " " << QDes[1][idx] << " "
        // << 0 << " " << QDes[2][idx] << " " << 0 << std::endl;

        // _data->_legController->commands[0].tau << 0, -tauDes[0][idx], -tauDes[1][idx];// front right leg
        // _data->_legController->commands[1].tau << _data->_legController->commands[0].tau; // front left leg
        // _data->_legController->commands[2].tau << 0, -tauDes[2][idx], -tauDes[3][idx];// rear right leg
        // _data->_legController->commands[3].tau << _data->_legController->commands[2].tau; // rear left leg

        torque << _data ->_legController->commands[0].tau(0) <<" "<< _data ->_legController->commands[1].tau(0) <<" "<<_data ->_legController->commands[2].tau(0)<<" "<<_data ->_legController->commands[3].tau(0)<<" "
    	  << _data ->_legController->commands[0].tau(1) <<" "<< _data ->_legController->commands[1].tau(1) <<" "<<_data ->_legController->commands[2].tau(1)<<" "<<_data ->_legController->commands[3].tau(1)<<" "
    	  << _data ->_legController->commands[0].tau(2) <<" "<< _data ->_legController->commands[1].tau(2) <<" "<<_data ->_legController->commands[2].tau(2)<<" "<<_data ->_legController->commands[3].tau(2)<<" "
        << final_torque_h0 << " " << final_torque_h1 << " " << final_torque_h2 << " " << final_torque_h3 << " " 
        << final_torque_t0 << " " << final_torque_t1 << " " << final_torque_t2 << " " << final_torque_t3 << " "
        << final_torque_c0 << " " << final_torque_c1 << " " << final_torque_c2 << " " << final_torque_c3 << " " << std::endl;

        joint_act << _data ->_legController->data[0].q(0) <<" "<< _data ->_legController->data[1].q(0) <<" "<<_data ->_legController->data[2].q(0)<<" "<<_data ->_legController->data[3].q(0)<<" "
        <<_data ->_legController->data[0].q(1) <<" "<< _data ->_legController->data[1].q(1) <<" "<<_data ->_legController->data[2].q(1)<<" "<<_data ->_legController->data[3].q(1)<<" " 
        <<_data ->_legController->data[0].q(2) <<" "<< _data ->_legController->data[1].q(2) <<" "<<_data ->_legController->data[2].q(2)<<" "<<_data ->_legController->data[3].q(2)<<" "
        <<_data ->_legController->data[0].qd(0) <<" "<<_data ->_legController->data[1].qd(0) <<" "<<_data ->_legController->data[2].qd(0)<<" "<<_data ->_legController->data[3].qd(0)<< " "
        <<_data ->_legController->data[0].qd(1) <<" "<< _data ->_legController->data[1].qd(1) <<" "<<_data ->_legController->data[2].qd(1)<<" "<<_data ->_legController->data[3].qd(1) << " "
        <<_data ->_legController->data[0].qd(2) <<" "<< _data ->_legController->data[1].qd(2) <<" "<<_data ->_legController->data[2].qd(2)<<" "<<_data ->_legController->data[3].qd(2)<<" "
        <<_data ->_legController->data[0].tau(0) <<" "<< _data ->_legController->data[1].tau(0) <<" "<<_data ->_legController->data[2].tau(0)<<" "  <<_data ->_legController->data[3].tau(0) << " "
        <<_data ->_legController->data[0].tau(1) <<" "<< _data ->_legController->data[1].tau(1) <<" "<<_data ->_legController->data[2].tau(1)<<" "<<_data ->_legController->data[3].tau(1) << " "
        <<_data ->_legController->data[0].tau(2) <<" "<< _data ->_legController->data[1].tau(2) <<" "<<_data ->_legController->data[2].tau(2)<<" "<<_data ->_legController->data[3].tau(2) <<std::endl;
    
        joint_des << _data ->_legController->commands[0].qDes(0) <<" "<< _data ->_legController->commands[1].qDes(0) <<" "<<_data ->_legController->commands[2].qDes(0)<<" "<<_data ->_legController->commands[3].qDes(0)<<" "
    	  << _data ->_legController->commands[0].qDes(1) <<" "<< _data ->_legController->commands[1].qDes(1) <<" "<<_data ->_legController->commands[2].qDes(1)<<" "<<_data ->_legController->commands[3].qDes(1)<<" "
    	  << _data ->_legController->commands[0].qDes(2) <<" "<< _data ->_legController->commands[1].qDes(2) <<" "<<_data ->_legController->commands[2].qDes(2)<<" "<<_data ->_legController->commands[3].qDes(2)<<" "
    	  << _data ->_legController->commands[0].qdDes(0) <<" "<< _data ->_legController->commands[1].qdDes(0) <<" "<<_data ->_legController->commands[2].qdDes(0)<<" "<<_data ->_legController->commands[3].qdDes(0)<<" "
    	  << _data ->_legController->commands[0].qdDes(1) <<" "<< _data ->_legController->commands[1].qdDes(1) <<" "<<_data ->_legController->commands[2].qdDes(1)<<" "<<_data ->_legController->commands[3].qdDes(1)<<" "
    	  << _data ->_legController->commands[0].qdDes(2) <<" "<< _data ->_legController->commands[1].qdDes(2) <<" "<<_data ->_legController->commands[2].qdDes(2)<<" "<<_data ->_legController->commands[3].qdDes(2)<<" "
    	  << 0 <<" "<< 0 <<" "<<0<<" "<<0<<" "
    	  << -tauDes[0][idx] <<" "<< -tauDes[0][idx] <<" "<<-tauDes[2][idx] <<" "<<-tauDes[2][idx]<<" "
    	  << -tauDes[1][idx] <<" "<< -tauDes[0][idx] <<" "<<-tauDes[3][idx]<<" "<<-tauDes[3][idx]<<std::endl;

        // // FR, FL, RR, RL
        // foot_act<< _data ->_legController ->data[0].p(0)<< " " << _data ->_legController ->data[0].p(1)<<" "<< _data ->_legController ->data[0].p(2)<<" "
        // <<_data ->_legController ->data[1].p(0)<< " " <<_data ->_legController ->data[1].p(1)<<" "<< _data ->_legController ->data[1].p(2)<<" "
        // <<_data ->_legController ->data[2].p(0)<< " " << _data ->_legController ->data[2].p(1)<<" "<< _data ->_legController ->data[2].p(2)<<" "
        // <<_data ->_legController ->data[3].p(0)<< " " << _data ->_legController ->data[3].p(1)<<" "<< _data ->_legController ->data[3].p(2)<<" "
        // <<_data ->_legController ->data[0].v(0)<< " " << _data ->_legController ->data[0].v(1)<<" "<< _data ->_legController ->data[0].v(2)<<" "
        // <<_data ->_legController ->data[1].v(0)<< " " << _data ->_legController ->data[1].v(1)<<" "<< _data ->_legController ->data[1].v(2)<<" "
        // <<_data ->_legController ->data[2].v(0)<< " " << _data ->_legController ->data[2].v(1)<<" "<< _data ->_legController ->data[2].v(2)<<" "
        // <<_data ->_legController ->data[3].v(0)<< " " << _data ->_legController ->data[3].v(1)<<" "<< _data ->_legController ->data[3].v(2)<<std::endl;
        
        // // FR, FL, RR, RL
        // foot_des<< _data ->_legController ->commands[0].pDes(0)<< " " << _data ->_legController ->commands[0].pDes(1)<<" "<< _data ->_legController ->commands[0].pDes(2)<<" "
        // <<_data ->_legController ->commands[1].pDes(0)<< " " << _data ->_legController ->commands[1].pDes(1)<<" "<< _data ->_legController ->commands[1].pDes(2)<<" "
        // <<_data ->_legController ->commands[2].pDes(0)<< " " << _data ->_legController ->commands[2].pDes(1)<<" "<< _data ->_legController ->commands[2].pDes(2)<<" "
        // <<_data ->_legController ->commands[3].pDes(0)<< " " << _data ->_legController ->commands[3].pDes(1)<<" "<< _data ->_legController ->commands[3].pDes(2)<<" "
        // <<_data ->_legController ->commands[0].vDes(0)<< " " << _data ->_legController ->commands[0].vDes(1)<<" "<< _data ->_legController ->commands[0].vDes(2)<<" "
        // <<_data ->_legController ->commands[1].vDes(0)<< " " << _data ->_legController ->commands[1].vDes(1)<<" "<< _data ->_legController ->commands[1].vDes(2)<<" "
        // <<_data ->_legController ->commands[2].vDes(0)<< " " << _data ->_legController ->commands[2].vDes(1)<<" "<< _data ->_legController ->commands[2].vDes(2)<<" "
        // <<_data ->_legController ->commands[3].vDes(0)<< " " << _data ->_legController ->commands[3].vDes(1)<<" "<< _data ->_legController ->commands[3].vDes(2)<<std::endl;
        

        pos << _data->_stateEstimator->getResult().vWorld(0) << " " << _data->_stateEstimator->getResult().vWorld(1) << " " <<  _data->_stateEstimator->getResult().vWorld(2) << " " 
        <<_data->_stateEstimator->getResult().position(0) << " " << _data->_stateEstimator->getResult().position(1) << " " << _data->_stateEstimator->getResult().position(2) << " " 
        << _data->_stateEstimator->getResult().rpy(0) << " " << _data->_stateEstimator->getResult().rpy(1) << " " << _data->_stateEstimator->getResult().rpy(2) << " " 
        << _data->_legController->commands[0].feedforwardForce(2) << " " <<  _data->_legController->commands[1].feedforwardForce(2) << " " <<  _data->_legController->commands[2].feedforwardForce(2) << " " <<  _data->_legController->commands[3].feedforwardForce(2) << " " 
        << _data->_stateEstimator->getResult().contactEstimate(0) << " " << _data->_stateEstimator->getResult().contactEstimate(1) << " " << _data->_stateEstimator->getResult().contactEstimate(2) << " " << _data->_stateEstimator->getResult().contactEstimate(3)<<" "
        << lowState.footForce[0] << " " << lowState.footForce[1] <<" "<< lowState.footForce[2]<<" " << lowState.footForce[3]<< " " << std::endl;
    
    }

    if (counter >=startjump && counter<= startjump+N_TrajRef-pose_time){

        Voltage << voltage[0] << " " << voltage[1] << " " << voltage[2] << " " << voltage[3] <<" " << std::endl;
        Current << current[0] << " " << current[1] << " " << current[2] << " " << current[3] <<" " << std::endl;
    }    


    _data->_legController->updateCommand();


    rate.sleep();

    std::cout << "counter1: " << counter << std::endl;
    counter++;
    //std::cout<<"counter2: " << counter << std::endl;
    //std::cout << "force" << std::endl;
 
}
 
//  b_des.close();
//  z_pos.close();
//  myfile.close();

 QP.close();
 pos.close();
 com_act.close();
 com_des.close();
 joint_act.close();
 joint_des.close();
 torque.close();

 Voltage.close();
 Current.close();

 std::cout << "stand up finished" << std::endl;
}


void FSM_State::Jump2D(){

    ofstream pos; // include QP landing period
    ofstream com_act; // to compare with pos_des
    ofstream com_des;
    ofstream joint_act;
    ofstream joint_des;
    ofstream foot_act;
    ofstream foot_des;
    ofstream torque;

    pos.open("pos.txt");
    com_act.open("com_act.txt");
    com_des.open("com_des.txt");
    joint_act.open("joint_act.txt");
    joint_des.open("joint_des.txt");
    foot_act.open("foot_act.txt");
    foot_des.open("foot_des.txt");
    torque.open("torque.txt");


    ros::Rate rate(1000);
    // reset forces and steps to 0
    footFeedForwardForces = Mat34<double>::Zero();
    footstepLocations = Mat34<double>::Zero();
    double minForce = 5;
    double maxForce = 500;
    double contactStateScheduled[4] = {1, 1, 1, 1};

    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

    Vec4<double> contactphase(0.5,0.5,0.5,0.5); 
    _data->_stateEstimator->setContactPhase(contactphase);
    //Timer time;
    //time.start();
    //double COM_weights_stance[3] = {50, 50, 50};
    //double Base_weights_stance[3] = {500, 150, 30};// for QP locomotion
    double COM_weights_stance[3] = {5, 5, 10};
    double Base_weights_stance[3] = {10, 10, 20};
    double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
      omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
    double b_control[6];
    int counter = 0;
       // ros::spinOnce();
       _data->_legController->updateData();
       _data->_stateEstimator->run();
    // se_xfb[3] = 1.0; 
    //  int during = 10;
    // position & rpy desired
    for(int i = 0; i < 3; i++){
      //p_des[i] = _data->_stateEstimator->getResult().position(i); 
      v_des[i] = 0;
      omegaDes[i] = 0;
      rpy[i] = 0;
    }
       //rpy[2] = _data->_stateEstimator->getResult().rpy(2);
      p_des[2] = 0.2; // for jump 3D (final position)
      //p_des[2] = 0.1950;
    //double v_des[3] = {0, 0, 0};


    //------------ Read the optimization data from imported files --------------------
    // ------------Stored the data into vectors --------------------------------------
    Timer t0;
    t0.start();
    // Position
    vector<vector<double>> QDes;

    // Torque
    vector<vector<double>> tauDes;

    // foot pos
    vector<vector<double>> pfDes;

    //foot vel
    vector<vector<double>> vfDes;
    

    // Read the joint velocity and position from optimization data
    std::ifstream myFile;
    myFile.open("src/laikago_ros/optimization_data/3D_jump2D/data_Q.csv");

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
                // tau: from 0 to 11
                // FL(tau 0,1,2); FR(tau 3,4,5); RL(tau 6,7,8); RR(tau 9,10,11)
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

    // Read the joint torque from optimization data
    std::ifstream mytauFile("src/laikago_ros/optimization_data/3D_jump2D/data_tau.csv");

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
            tau_joint.push_back(val);
            if(ss.peek() == ',') ss.ignore();
        }
        tauDes.push_back(tau_joint);
    }

    mytauFile.close();

        //Read the foot pos_actition from optimization data
    std::ifstream mypfFile("src/laikago_ros/optimization_data/3D_jump2D/data_pf.csv");

    // If cannot open the file, report an error
    if(!mypfFile.is_open())
    {
        std::cout << "Could not open file for foot pos_actition" << endl;
        //return 0;
    }
    cout << "Reading Optimization Data for foot pos_actition" << endl;

    index = 0;

    while(getline(mypfFile, line))
    {
        // tau: from 0 to 11
        // FL(tau 0,1,2); FR(tau 3,4,5); RL(tau 6,7,8); RR(tau 9,10,11)
        stringstream ss(line);
        double val;
        vector<double> pfoot;
        while(ss >> val)
        {
            pfoot.push_back(val);
            if(ss.peek() == ',') ss.ignore();
        }
        pfDes.push_back(pfoot);
    }
    mypfFile.close();

    // Read the foot velocity from optimization data
    std::ifstream myvfFile("src/laikago_ros/optimization_data/3D_jump2D/data_vf.csv");

    // If cannot open the file, report an error
    if(!myvfFile.is_open())
    {
        std::cout << "Could not open file for foot velocity" << endl;
        //return 0;
    }
    cout << "Reading Optimization Data for foot velocity" << endl;

    index = 0;

    while(getline(myvfFile, line))
    {
        // tau: from 0 to 11
        // FL(tau 0,1,2); FR(tau 3,4,5); RL(tau 6,7,8); RR(tau 9,10,11)
        stringstream ss(line);
        double val;
        vector<double> vfoot;
        while(ss >> val)
        {
            vfoot.push_back(val);
            if(ss.peek() == ',') ss.ignore();
        }
        vfDes.push_back(vfoot);
    }
    myvfFile.close(); 


    printf("Time for reading jumping optimization data: %f ms\n", t0.getMs());
    //int data_size=1020;
    int data_size=1250; // 1 roll
    //int data_size=1530; // 2 roll
    int pose_time=250;
    int idx_pose=data_size-pose_time;
    
    //std::cout << "tauDes is: " << tauDes[0][1] << std::endl;
    //std::cout << "tauDes is: " << tauDes[1][1] << std::endl;
    //std::cout << "tauDes is: " << tauDes[2][1] << std::endl;
    //std::cout << "tauDes is: " << tauDes[3][1] << std::endl;
    //std::cout << "qDes is: " << qDes[0][1] << std::endl;
    //std::cout << "qDes is: " << qDes[1][1] << std::endl;
    //std::cout << "qDes is: " << qDes[2][1] << std::endl;
    //std::cout << "qDes is: " << qDes[3][1] << std::endl;

    // hip sign is same for both side
    // right legs: hip >0
    // left legs: hip>0 
   // double init_Pos[12] = {0, 1.2566, -2.355, 0, 1.2566, -2.355, 0, 1.2566, -2.355, 0, 1.2566, -2.355}; // pose 0
    double init_Pos[12] = {0, 1.1310, -2.1206, 0, 1.1310, -2.1206, 0, 1.1310, -2.1206, 0, 1.1310, -2.1206}; // 64, -121

    double currentPos[12], percent;
    double FR=0; double FL=0; double RR=0; double RL=0; // contact state
    while(ros::ok()){
       //std::cout << ros::Time::now() << std::endl;
       //std::cout << "counter1: " << counter << std::endl;
       // rate.reset();

       _data->_legController->updateData();
       _data->_stateEstimator->run();

        double Kp = 300;
        double Kd = 3; 
        double Hip_Kp = 300;
        double Hip_Kd =3;
        double Kp_Ca= 0;
        double Kd_Ca =0;
        double Kp_l=6; // Kp for landing
        double Kd_l=2; // Kd for landing
        int idx = 0; // data index
        int cs = 0; // contact state. To check if any leg touch the ground.
        
        int startjump = 3000; // index start jumping
        //std::cout << "Start jumping" << std::endl;

        if (counter <startjump){  // Initialization
          runQP=false;
          std::cout << "FLfootCallback: " << lowState.footForce[1] << std::endl;
          std::cout << "RLfootCallback: " << lowState.footForce[3] << std::endl;
          //double contactStateScheduled[4]={1,1,1,1}; // all legs are stance legs
        for(int i = 0; i < 4; i++){
            _data->_legController->commands[i].kpJoint = Vec3<double>(Hip_Kp,Kp,Kp).asDiagonal();
            _data->_legController->commands[i].kdJoint = Vec3<double>(Hip_Kd,Kd,Kd).asDiagonal();
            _data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
            _data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();
        }

        percent = (double) counter/ startjump; 
           std ::cout << "percent: " << percent << std::endl; 
           for (int i=0; i<4; i++){ 
               for(int j=0; j<3; j++){ 
                 currentPos[3*i+j] = _data->_legController->data[i].q(j); 
                 _data->_legController->commands[i].qDes(j) = currentPos[3*i+j]*(1-percent) + init_Pos[3*i+j]*percent; 
               } 
            } 
        }

        if (counter >=startjump && counter < startjump+data_size-pose_time){ // 
          runQP=false;
          for(int i = 0; i <4; i++){
              _data->_legController->commands[i].kpJoint << Hip_Kp, 0, 0,
                                                          0, Kp, 0,
                                                          0, 0, Kp;

              _data->_legController->commands[i].kdJoint << Hip_Kd, 0, 0,
                                                          0, Kd, 0,
                                                          0, 0, Kd;

              _data->_legController->commands[i].kpCartesian << Kp_Ca, 0, 0,
                                                                0, Kp_Ca, 0,
                                                                0, 0, Kp_Ca;
              _data->_legController->commands[i].kdCartesian << Kd_Ca, 0, 0,
                                                                0, Kd_Ca, 0,
                                                                0, 0, Kd_Ca;
              //_data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
              //_data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();
          }
          idx= counter-startjump;
          std::cout<< "idx: " << idx <<std::endl;
          _data->_legController->commands[0].tau << tauDes[3][idx], tauDes[4][idx], tauDes[5][idx];// front right leg
          _data->_legController->commands[1].tau << _data->_legController->commands[0].tau;// front right leg
          // _data->_legController->commands[1].tau << tauDes[0][idx], tauDes[1][idx], tauDes[2][idx]; // front left leg
          _data->_legController->commands[2].tau << tauDes[9][idx], tauDes[10][idx], tauDes[11][idx];// rear right leg
          _data->_legController->commands[3].tau << _data->_legController->commands[2].tau;// rear right leg
          // _data->_legController->commands[3].tau << tauDes[6][idx], tauDes[7][idx], tauDes[8][idx]; // rear left leg

          _data->_legController->commands[0].qDes << QDes[7][idx], QDes[8][idx], QDes[9][idx]; // front right leg
          _data->_legController->commands[1].qDes << _data->_legController->commands[0].qDes;
          // _data->_legController->commands[1].qDes << QDes[4][idx], QDes[5][idx], QDes[6][idx]; // front left leg
          _data->_legController->commands[2].qDes << QDes[13][idx], QDes[14][idx], QDes[15][idx];// rear right leg
          _data->_legController->commands[3].qDes << _data->_legController->commands[2].qDes;
          // _data->_legController->commands[3].qDes << QDes[10][idx],QDes[11][idx], QDes[12][idx]; // rear left leg

          _data->_legController->commands[0].qdDes << QDes[23][idx], QDes[24][idx], QDes[25][idx]; // front right leg
          _data->_legController->commands[1].qdDes << _data->_legController->commands[0].qdDes;
          // _data->_legController->commands[1].qdDes << QDes[20][idx], QDes[21][idx], QDes[22][idx]; // front left leg
          _data->_legController->commands[2].qdDes << QDes[29][idx], QDes[30][idx], QDes[31][idx];// rear right leg
          _data->_legController->commands[3].qdDes << _data->_legController->commands[2].qdDes;
          // _data->_legController->commands[3].qdDes << QDes[26][idx], QDes[27][idx], QDes[28][idx]; // rear left leg

          _data->_legController->commands[0].pDes << pfDes[3][idx], pfDes[4][idx], pfDes[5][idx];// front right leg
          _data->_legController->commands[1].pDes << pfDes[0][idx], pfDes[1][idx], pfDes[2][idx];// front left leg
          _data->_legController->commands[2].pDes << pfDes[9][idx], pfDes[10][idx], pfDes[11][idx];// rear right leg
          _data->_legController->commands[3].pDes << pfDes[6][idx], pfDes[7][idx], pfDes[8][idx];// rear left leg

          _data->_legController->commands[0].vDes << vfDes[3][idx], vfDes[4][idx], vfDes[5][idx];// front right leg
          _data->_legController->commands[1].vDes << vfDes[0][idx], vfDes[1][idx], vfDes[2][idx];// front left leg
          _data->_legController->commands[2].vDes << vfDes[9][idx], vfDes[10][idx], vfDes[11][idx];// rear right leg
          _data->_legController->commands[3].vDes << vfDes[6][idx], vfDes[7][idx], vfDes[8][idx];// rear left leg

          
          
          for (int leg = 0; leg < 4; leg++) {
             _data->_legController->commands[leg].feedforwardForce = Vec3<double>::Zero();
          }

          lowState.footForce[0]=0; lowState.footForce[1]=0; lowState.footForce[2]=0; lowState.footForce[3]=0;


        }

    
        if (counter >=startjump + data_size-pose_time){ // run QP

        // use PD for swing legs

          _data->_legController->commands[0].tau << 0, 0, 0;// front right leg
          _data->_legController->commands[1].tau << 0, 0, 0; // front left leg
          _data->_legController->commands[2].tau << 0, 0, 0;// rear right leg
          _data->_legController->commands[3].tau << 0, 0, 0; // rear left leg

          _data->_legController->commands[0].qDes << QDes[7][idx_pose], QDes[8][idx_pose], QDes[9][idx_pose]; // front right leg
          _data->_legController->commands[1].qDes << QDes[4][idx_pose], QDes[5][idx_pose], QDes[6][idx_pose]; // front left leg
          _data->_legController->commands[2].qDes << QDes[13][idx_pose], QDes[14][idx_pose], QDes[15][idx_pose];// rear right leg
          _data->_legController->commands[3].qDes << QDes[10][idx_pose],QDes[11][idx_pose], QDes[12][idx_pose]; // rear left leg

          _data->_legController->commands[0].qdDes << 0, 0, 0; // front right leg
          _data->_legController->commands[1].qdDes << 0, 0, 0; // front left leg
          _data->_legController->commands[2].qdDes << 0, 0, 0;// rear right leg
          _data->_legController->commands[3].qdDes << 0, 0, 0; // rear left leg

          for (int i=0; i<4; i++){
              _data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
              _data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();

          }

          // Get contact state
          FR=lowState.footForce[0];
          FL=lowState.footForce[1];
          RR=lowState.footForce[2];
          RL=lowState.footForce[3];
          if (FR>0){
            FR=1;
            _data->_legController->commands[0].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[0].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
            // _data->_legController->commands[0].kpCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[0].kdCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[0].tau << Vec3<double>::Zero();// front right leg
          }
          else{
            FR=0;
          }
          if (FL>0){
            FL=1;
            _data->_legController->commands[1].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[1].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
            // _data->_legController->commands[1].kpCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[1].kdCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[1].tau << Vec3<double>::Zero();// front right leg

          }
          else{
            FL=0;
          }
          if (RR>0){
            RR=1;
            _data->_legController->commands[2].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[2].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
            // _data->_legController->commands[2].kpCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[2].kdCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[2].tau << Vec3<double>::Zero();// front right leg
          }
          else{
            RR=0;
          }
          if (RL>0){
            RL=1;
            _data->_legController->commands[3].kpJoint << Kp_l, 0, 0,
                                                          0, Kp_l, 0,
                                                          0, 0, Kp_l;
            _data->_legController->commands[3].kdJoint << Kd_l, 0, 0,
                                                          0, Kd_l, 0,
                                                          0, 0, Kd_l;
            // _data->_legController->commands[3].kpCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[3].kdCartesian << Mat3<double>::Zero();
            // _data->_legController->commands[3].tau << Vec3<double>::Zero();// front right leg
          }
          else{
            RR=0;
          }

          /*
          if (FR>0 && FL >0 && RR >0 && RL>0){
            std::cout << "Debug Chuong here" <<std::endl;
            FR=1; FL=1; RR=1; RL=1;
            for(int i = 0; i <4; i++){
              _data->_legController->commands[i].kpJoint << Mat3<double>::Zero();
              _data->_legController->commands[i].kdJoint << Mat3<double>::Zero();
              _data->_legController->commands[i].tau << Vec3<double>::Zero();// front right leg
              //_data->_legController->commands[i].kpCartesian << Mat3<double>::Zero();
              //_data->_legController->commands[i].kdCartesian << Mat3<double>::Zero();
          }
          */

          double contactStateScheduled[4]={FR,FL,RR,RL};
          std::cout << "contactStateScheduled: " << contactStateScheduled[0] << "," << contactStateScheduled[1]<<"," << contactStateScheduled[2]<< "," << contactStateScheduled[3]<< std::endl;
          runQP=true;

          // // Set cs=1 when one of each leg touch the ground
          // if (FR==1 || FL==1 || RL==1 || RR==1){
          //   cs=1;
          // }
          
        }

    std::cout << "contact state" << cs << std::endl;  
    /* =================================================================*/
    if(runQP){
      for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
      }
       // v_des[2] = 0.1;
      for (int i = 0; i < 3; i++) {
        //rpy[i] = 0;
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        
        v_act[i] = _data->_stateEstimator->getResult().vWorld(i);
        // v_des[i] = 0;
        //v_des[2] = 0.005;

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaWorld(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vWorld(i);
        
    // Set the translational and orientation gains
        kpCOM[i] = 30;   //_data->controlParameters->kpCOM(i);
        kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
        kpBase[i] = 80;     //_data->controlParameters->kpBase(i);
        kdBase[i] = 20; //  _data->controlParameters->kdBase(i);
        }

      kpCOM[2] = 50;
      kpBase[0] = 300;
      kpBase[1] = 200;

    //Vec3<double> pFeetVec;
    Vec3<double> pFeetVecCOM;

    // Get the foot locations relative to COM
      for (int leg = 0; leg < 4; leg++) {
        // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
        //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
        //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);

        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
        //std::cout << "leg" << leg << std::endl;
        //std::cout << "pFeet" << pFeetVecCOM << std::endl;
      }
     _data->_stateEstimator->setContactPhase(contactphase);

    p_des[0] = p_act[0] + (pFeet[0] + pFeet[3] + pFeet[6] + pFeet[9]) / 4.0;
    p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10]) / 4.0;
    //  myfile << "\n";
     // std::cout << j << std::endl;
     //std::cout << "run QP" << std::endl;
    balanceController.set_alpha_control(0.01); 
    balanceController.set_friction(0.2);
    balanceController.set_mass(_data->_quadruped->mass);
    balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
    balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
    balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _data->_stateEstimator->getResult().rpy(2));
   // balanceController.print_QPData();
    double fOpt[12];
    balanceController.solveQP_nonThreaded(fOpt);

    std::cout << "zForce leg 0: " << fOpt[2] << std::endl;
    //balanceController.get_b_matrix(b_control);
    //b_des << b_control[2] << "\n";

  // Publish the results over ROS
  // balanceController.publish_data_lcm();

  // Copy the results to the feed forward forces
    
     //_data->_stateEstimator->run();


    for (int leg = 0; leg < 4; leg++) {
        footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
        fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame
        
        _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
        //_data->_stateEstimator->getResult().rBody.transpose() * footFeedForwardForces.col(leg); 
        //std::cout <<"zForce leg "<< _data->_legController->commands[leg].feedforwardForce[2]<< "\n";
        //QP <<_data->_legController->commands[i].feedforwardForce[2] << " ";
    }
    }


    // Plot data
    //QP << _data->_legController->commands[0].feedforwardForce[2] << " "<<_data->_legController->commands[1].feedforwardForce[2] << " "<<_data->_legController->commands[2].feedforwardForce[2] << " " << _data->_legController->commands[3].feedforwardForce[2]<<"\n";
    
    _data->_legController->updateCommand_Jump3D_4base();


    if (counter >=startjump && counter<= startjump+data_size+ 1000){

        pos << _data->_stateEstimator->getResult().vWorld(0) << " " << _data->_stateEstimator->getResult().vWorld(1) << " " <<  _data->_stateEstimator->getResult().vWorld(2) << " " 
        <<_data->_stateEstimator->getResult().position(0) << " " << _data->_stateEstimator->getResult().position(1) << " " << _data->_stateEstimator->getResult().position(2) << " " 
        << _data->_stateEstimator->getResult().rpy(0) << " " << _data->_stateEstimator->getResult().rpy(1) << " " << _data->_stateEstimator->getResult().rpy(2) << " " 
        << _data->_legController->commands[0].feedforwardForce(2) << " " <<  _data->_legController->commands[1].feedforwardForce(2) << " " <<  _data->_legController->commands[2].feedforwardForce(2) << " " <<  _data->_legController->commands[3].feedforwardForce(2) << " " 
        << _data->_stateEstimator->getResult().contactEstimate(0) << " " << _data->_stateEstimator->getResult().contactEstimate(1) << " " << _data->_stateEstimator->getResult().contactEstimate(2) << " " << _data->_stateEstimator->getResult().contactEstimate(3)<<" "
        << lowState.footForce[0] << " " << lowState.footForce[1] <<" "<< lowState.footForce[2]<<" " << lowState.footForce[3]<< " " << cs << std::endl;
    }

    if (counter >=startjump && counter < startjump+ data_size && runQP==false){

        int idx= counter-startjump;

        double final_torque_h0=lowCmd.motorCmd[0].torque+ Kp*(_data ->_legController->commands[0].qDes(0)-_data ->_legController->data[0].q(0))+Kd*(_data ->_legController->commands[0].qdDes(0)-_data ->_legController->data[0].qd(0));
        double final_torque_h1=lowCmd.motorCmd[3].torque+ Kp*(_data ->_legController->commands[1].qDes(0)-_data ->_legController->data[1].q(0))+Kd*(_data ->_legController->commands[1].qdDes(0)-_data ->_legController->data[1].qd(0));
        double final_torque_h2=lowCmd.motorCmd[6].torque+ Kp*(_data ->_legController->commands[2].qDes(0)-_data ->_legController->data[2].q(0))+Kd*(_data ->_legController->commands[2].qdDes(0)-_data ->_legController->data[2].qd(0));
        double final_torque_h3=lowCmd.motorCmd[9].torque+ Kp*(_data ->_legController->commands[3].qDes(0)-_data ->_legController->data[3].q(0))+Kd*(_data ->_legController->commands[3].qdDes(0)-_data ->_legController->data[3].qd(0));
        double final_torque_t0=lowCmd.motorCmd[1].torque+ Kp*(_data ->_legController->commands[0].qDes(1)-_data ->_legController->data[0].q(1))+Kd*(_data ->_legController->commands[0].qdDes(1)-_data ->_legController->data[0].qd(1));
        double final_torque_t1=lowCmd.motorCmd[4].torque+ Kp*(_data ->_legController->commands[1].qDes(1)-_data ->_legController->data[1].q(1))+Kd*(_data ->_legController->commands[1].qdDes(1)-_data ->_legController->data[1].qd(1));
        double final_torque_t2=lowCmd.motorCmd[7].torque+ Kp*(_data ->_legController->commands[2].qDes(1)-_data ->_legController->data[2].q(1))+Kd*(_data ->_legController->commands[2].qdDes(1)-_data ->_legController->data[2].qd(1));
        double final_torque_t3=lowCmd.motorCmd[10].torque+ Kp*(_data ->_legController->commands[3].qDes(1)-_data ->_legController->data[3].q(1))+Kd*(_data ->_legController->commands[3].qdDes(1)-_data ->_legController->data[3].qd(1));
        double final_torque_c0=lowCmd.motorCmd[2].torque+ Kp*(_data ->_legController->commands[0].qDes(2)-_data ->_legController->data[0].q(2))+Kd*(_data ->_legController->commands[0].qdDes(2)-_data ->_legController->data[0].qd(2));
        double final_torque_c1=lowCmd.motorCmd[5].torque+ Kp*(_data ->_legController->commands[1].qDes(2)-_data ->_legController->data[1].q(2))+Kd*(_data ->_legController->commands[1].qdDes(2)-_data ->_legController->data[1].qd(2));
        double final_torque_c2=lowCmd.motorCmd[8].torque+ Kp*(_data ->_legController->commands[2].qDes(2)-_data ->_legController->data[2].q(2))+Kd*(_data ->_legController->commands[2].qdDes(2)-_data ->_legController->data[2].qd(2));
        double final_torque_c3=lowCmd.motorCmd[11].torque+ Kp*(_data ->_legController->commands[3].qDes(2)-_data ->_legController->data[3].q(2))+Kd*(_data ->_legController->commands[3].qdDes(2)-_data ->_legController->data[3].qd(2));
        


        com_act << _data->_stateEstimator->getResult().vWorld(0) << " " << _data->_stateEstimator->getResult().vWorld(1) << " " <<  _data->_stateEstimator->getResult().vWorld(2) << " " 
        <<_data->_stateEstimator->getResult().position(0) << " " << _data->_stateEstimator->getResult().position(1) << " " << _data->_stateEstimator->getResult().position(2) << " " 
        << _data->_stateEstimator->getResult().rpy(0) << " " << _data->_stateEstimator->getResult().rpy(1) << " " << _data->_stateEstimator->getResult().rpy(2) << std::endl;

        com_des << QDes[16][idx] << " " << QDes[17][idx] << " " << QDes[18][idx] << " " 
        << QDes[0][idx] << " " << QDes[1][idx] << " " << QDes[2][idx] << " "
        << QDes[3][idx] << " " << 0 << " " << 0 << std::endl;

        _data->_legController->commands[0].tau << tauDes[3][idx], tauDes[4][idx], tauDes[5][idx];// front right leg
        _data->_legController->commands[1].tau << tauDes[0][idx], tauDes[1][idx], tauDes[2][idx]; // front left leg
        _data->_legController->commands[2].tau << tauDes[9][idx], tauDes[10][idx], tauDes[11][idx];// rear right leg
        _data->_legController->commands[3].tau << tauDes[6][idx], tauDes[7][idx], tauDes[8][idx]; // rear left leg

        torque << _data ->_legController->commands[0].tau(0) <<" "<< _data ->_legController->commands[1].tau(0) <<" "<<_data ->_legController->commands[2].tau(0)<<" "<<_data ->_legController->commands[3].tau(0)<<" "
    	  << _data ->_legController->commands[0].tau(1) <<" "<< _data ->_legController->commands[1].tau(1) <<" "<<_data ->_legController->commands[2].tau(1)<<" "<<_data ->_legController->commands[3].tau(1)<<" "
    	  << _data ->_legController->commands[0].tau(2) <<" "<< _data ->_legController->commands[1].tau(2) <<" "<<_data ->_legController->commands[2].tau(2)<<" "<<_data ->_legController->commands[3].tau(2)<<" "
        << final_torque_h0 << " " << final_torque_h1 << " " << final_torque_h2 << " " << final_torque_h3 << " " 
        << final_torque_t0 << " " << final_torque_t1 << " " << final_torque_t2 << " " << final_torque_t3 << " "
        << final_torque_c0 << " " << final_torque_c1 << " " << final_torque_c2 << " " << final_torque_c3 << " " << std::endl;

        joint_act << _data ->_legController->data[0].q(0) <<" "<< _data ->_legController->data[1].q(0) <<" "<<_data ->_legController->data[2].q(0)<<" "<<_data ->_legController->data[3].q(0)<<" "
        <<_data ->_legController->data[0].q(1) <<" "<< _data ->_legController->data[1].q(1) <<" "<<_data ->_legController->data[2].q(1)<<" "<<_data ->_legController->data[3].q(1)<<" " 
        <<_data ->_legController->data[0].q(2) <<" "<< _data ->_legController->data[1].q(2) <<" "<<_data ->_legController->data[2].q(2)<<" "<<_data ->_legController->data[3].q(2)<<" "
        <<_data ->_legController->data[0].qd(0) <<" "<<_data ->_legController->data[1].qd(0) <<" "<<_data ->_legController->data[2].qd(0)<<" "<<_data ->_legController->data[3].qd(0)<< " "
        <<_data ->_legController->data[0].qd(1) <<" "<< _data ->_legController->data[1].qd(1) <<" "<<_data ->_legController->data[2].qd(1)<<" "<<_data ->_legController->data[3].qd(1) << " "
        <<_data ->_legController->data[0].qd(2) <<" "<< _data ->_legController->data[1].qd(2) <<" "<<_data ->_legController->data[2].qd(2)<<" "<<_data ->_legController->data[3].qd(2)<<" "
        <<_data ->_legController->data[0].tau(0) <<" "<< _data ->_legController->data[1].tau(0) <<" "<<_data ->_legController->data[2].tau(0)<<" "  <<_data ->_legController->data[3].tau(0) << " "
        <<_data ->_legController->data[0].tau(1) <<" "<< _data ->_legController->data[1].tau(1) <<" "<<_data ->_legController->data[2].tau(1)<<" "<<_data ->_legController->data[3].tau(1) << " "
        <<_data ->_legController->data[0].tau(2) <<" "<< _data ->_legController->data[1].tau(2) <<" "<<_data ->_legController->data[2].tau(2)<<" "<<_data ->_legController->data[3].tau(2) <<std::endl;
    
        joint_des << _data ->_legController->commands[0].qDes(0) <<" "<< _data ->_legController->commands[1].qDes(0) <<" "<<_data ->_legController->commands[2].qDes(0)<<" "<<_data ->_legController->commands[3].qDes(0)<<" "
    	  << _data ->_legController->commands[0].qDes(1) <<" "<< _data ->_legController->commands[1].qDes(1) <<" "<<_data ->_legController->commands[2].qDes(1)<<" "<<_data ->_legController->commands[3].qDes(1)<<" "
    	  << _data ->_legController->commands[0].qDes(2) <<" "<< _data ->_legController->commands[1].qDes(2) <<" "<<_data ->_legController->commands[2].qDes(2)<<" "<<_data ->_legController->commands[3].qDes(2)<<" "
    	  << _data ->_legController->commands[0].qdDes(0) <<" "<< _data ->_legController->commands[1].qdDes(0) <<" "<<_data ->_legController->commands[2].qdDes(0)<<" "<<_data ->_legController->commands[3].qdDes(0)<<" "
    	  << _data ->_legController->commands[0].qdDes(1) <<" "<< _data ->_legController->commands[1].qdDes(1) <<" "<<_data ->_legController->commands[2].qdDes(1)<<" "<<_data ->_legController->commands[3].qdDes(1)<<" "
    	  << _data ->_legController->commands[0].qdDes(2) <<" "<< _data ->_legController->commands[1].qdDes(2) <<" "<<_data ->_legController->commands[2].qdDes(2)<<" "<<_data ->_legController->commands[3].qdDes(2)<<" "
    	  << lowCmd.motorCmd[0].torque <<" "<< lowCmd.motorCmd[3].torque <<" "<<lowCmd.motorCmd[6].torque<<" "<<lowCmd.motorCmd[9].torque<<" "
    	  << lowCmd.motorCmd[1].torque <<" "<< lowCmd.motorCmd[4].torque <<" "<<lowCmd.motorCmd[7].torque<<" "<<lowCmd.motorCmd[10].torque<<" "
    	  << lowCmd.motorCmd[2].torque <<" "<< lowCmd.motorCmd[5].torque <<" "<<lowCmd.motorCmd[8].torque<<" "<<lowCmd.motorCmd[11].torque<<std::endl;

        // FR, FL, RR, RL
        foot_act<< _data ->_legController ->data[0].p(0)<< " " << _data ->_legController ->data[0].p(1)<<" "<< _data ->_legController ->data[0].p(2)<<" "
        <<_data ->_legController ->data[1].p(0)<< " " <<_data ->_legController ->data[1].p(1)<<" "<< _data ->_legController ->data[1].p(2)<<" "
        <<_data ->_legController ->data[2].p(0)<< " " << _data ->_legController ->data[2].p(1)<<" "<< _data ->_legController ->data[2].p(2)<<" "
        <<_data ->_legController ->data[3].p(0)<< " " << _data ->_legController ->data[3].p(1)<<" "<< _data ->_legController ->data[3].p(2)<<" "
        <<_data ->_legController ->data[0].v(0)<< " " << _data ->_legController ->data[0].v(1)<<" "<< _data ->_legController ->data[0].v(2)<<" "
        <<_data ->_legController ->data[1].v(0)<< " " << _data ->_legController ->data[1].v(1)<<" "<< _data ->_legController ->data[1].v(2)<<" "
        <<_data ->_legController ->data[2].v(0)<< " " << _data ->_legController ->data[2].v(1)<<" "<< _data ->_legController ->data[2].v(2)<<" "
        <<_data ->_legController ->data[3].v(0)<< " " << _data ->_legController ->data[3].v(1)<<" "<< _data ->_legController ->data[3].v(2)<<std::endl;
        
        // FR, FL, RR, RL
        foot_des<< _data ->_legController ->commands[0].pDes(0)<< " " << _data ->_legController ->commands[0].pDes(1)<<" "<< _data ->_legController ->commands[0].pDes(2)<<" "
        <<_data ->_legController ->commands[1].pDes(0)<< " " << _data ->_legController ->commands[1].pDes(1)<<" "<< _data ->_legController ->commands[1].pDes(2)<<" "
        <<_data ->_legController ->commands[2].pDes(0)<< " " << _data ->_legController ->commands[2].pDes(1)<<" "<< _data ->_legController ->commands[2].pDes(2)<<" "
        <<_data ->_legController ->commands[3].pDes(0)<< " " << _data ->_legController ->commands[3].pDes(1)<<" "<< _data ->_legController ->commands[3].pDes(2)<<" "
        <<_data ->_legController ->commands[0].vDes(0)<< " " << _data ->_legController ->commands[0].vDes(1)<<" "<< _data ->_legController ->commands[0].vDes(2)<<" "
        <<_data ->_legController ->commands[1].vDes(0)<< " " << _data ->_legController ->commands[1].vDes(1)<<" "<< _data ->_legController ->commands[1].vDes(2)<<" "
        <<_data ->_legController ->commands[2].vDes(0)<< " " << _data ->_legController ->commands[2].vDes(1)<<" "<< _data ->_legController ->commands[2].vDes(2)<<" "
        <<_data ->_legController ->commands[3].vDes(0)<< " " << _data ->_legController ->commands[3].vDes(1)<<" "<< _data ->_legController ->commands[3].vDes(2)<<std::endl;
        
    }
    // Send command to low-level controller

    //_data->_legController->updateCommand_Jump3D_4base();
    rate.sleep();

    counter++;
    std::cout<<"counter2: " << counter << std::endl;
    std::cout<<"------------------------------------"<< std::endl;
    //std::cout << "force" << std::endl;
    
}
 pos.close();
 com_act.close();
 com_des.close();
 joint_act.close();
 joint_des.close();
 foot_act.close();
 foot_des.close();
 torque.close();
 
 std::cout << " Saved data and Finished" << std::endl;
}

void FSM_State::JointPD_Stand(){ // need to improve 
    ros::Rate rate(1000);
    int counter = 0;
    double init_Pos[12] = {0, 1.2566, -2.355, 0, 1.2566, -2.355, 0, 1.2566, -2.355, 0, 1.2566, -2.355}; // pose 0
    while(ros::ok()){
       std::cout << ros::Time::now() << std::endl;
       std::cout << "counter1: " << counter << std::endl;
       // rate.reset();

       _data->_legController->updateData();
       _data->_stateEstimator->run();

        double Kp = 250;
        double Kd = 3;
        double Hip_Kp = 250;
        double Hip_Kd = 3;
        //double KpCartesian = 0;
        //double KdCartesian = 0;
        int idx = 0; // data index
        int startjump = 2000; // index start jumping
        //std::cout << "Start jumping" << std::endl;
        double pos[12] ,currentPos[12], percent;
        
        
        for(int i = 0; i < 4; i++){
            _data->_legController->commands[i].kpJoint = Vec3<double>(250,250,250).asDiagonal();
            _data->_legController->commands[i].kdJoint = Vec3<double>(3,3,3).asDiagonal();
        }

        if (counter <= startjump-1){ 
          
           percent = (double) counter/ startjump; 
           std ::cout << "percent: " << percent << std::endl; 
           for (int i=0; i<4; i++){ 
               for(int j=0; j<3; j++){ 
                 currentPos[3*i+j] = _data->_legController->data[i].q(j); 
                 _data->_legController->commands[i].qDes(j) = currentPos[3*i+j]*(1-percent) + init_Pos[3*i+j]*percent; 
               } 
            } 
          
            //runQP=false;
          
        }
        
    // Send command to low-level controller
    _data->_legController->updateCommand_Jump2D();
    
    rate.sleep();

    counter++;
    std::cout<<"counter2: " << counter << std::endl;
    std::cout<<"------------------------------------"<< std::endl;
    //std::cout << "force" << std::endl;
    
}
}

void FSM_State::QPstand(){
    ofstream myfile;
    ofstream QP;
    ofstream z_pos;
    ofstream b_des;
    myfile.open ("ori.txt");
    QP.open("QPsolution.txt");
    z_pos.open("zPos.txt");
    b_des.open("b_des_z.txt");
    ros::Rate rate(1000);
    // reset forces and steps to 0
    footFeedForwardForces = Mat34<double>::Zero();
    footstepLocations = Mat34<double>::Zero();
    double minForce = 5;
    double maxForce = 500;
    double contactStateScheduled[4] = {1, 1, 1, 1};

    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

    Vec4<double> contactphase(0.5,0.5,0.5,0.5); 
    _data->_stateEstimator->setContactPhase(contactphase);
    //Timer time;
    //time.start();
    //double COM_weights_stance[3] = {50, 50, 50};
    //double Base_weights_stance[3] = {500, 150, 30};// for QP locomotion
    double COM_weights_stance[3] = {5, 5, 10};
    double Base_weights_stance[3] = {10, 10, 20};
    double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
      omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
    double b_control[6];
    int counter = 0;
       // ros::spinOnce();
       _data->_legController->updateData();
       _data->_stateEstimator->run();
    // se_xfb[3] = 1.0; 
    //  int during = 10;
    // position & rpy desired
    for(int i = 0; i < 3; i++){
      //p_des[i] = _data->_stateEstimator->getResult().position(i); 
      v_des[i] = 0;
      omegaDes[i] = 0;
      rpy[i] = 0;
    }
       //rpy[2] = _data->_stateEstimator->getResult().rpy(2);
      //  p_des[2] = 0.4; // standup height for AlienGo
      p_des[2] = 0.3; // standup height for A1
     //bool firstLocoRun = true;
     bool runMPC = true;
    //double v_des[3] = {0, 0, 0};
    while(ros::ok()) {
     //std::cout << ros::Time::now() << std::endl;
      // rate.reset();
       _data->_legController->updateData();
       _data->_stateEstimator->run();

    /*======================== MPC locomotion ====================*/
   if(counter>10000){
     z_pos << _data->_stateEstimator->getResult().position[2] << " " <<  _data->_stateEstimator->getResult().position[1] << std::endl;
     //std::cout << "contact " << _data->_stateEstimator->getResult().contactEstimate << std::endl;
     //ROS_INFO("a");
     //std::cout << "start motion" << std::endl;
      //_data->_legController->updateData();
      //_data->_stateEstimator->run();
      Vec3<double> v_des(0, 0, 0); // v_body
      double yaw_rate = 0;
      double roll = 0;
      double pitch = 0;
      Cmpc.setGaitNum(1); 
      // Cmpc.climb = 1;
    //   std::cout << "state estimate rpy ";
    //   for(int i = 0; i < 3; i++){
    //   std::cout << _data->_stateEstimator->getResult().rpy(i) << " " ; 
    //  }

      if(counter > 4000){ 
        v_des[0] = 0.5;
      }

      if(counter > 10000){
       v_des[0] = 1;
      //  pitch = -0.3;
      //  v_des[1] = 0.3;
      //  yaw_rate = 2;
       Cmpc.setGaitNum(1);
      }

      if(counter > 15000){
        Cmpc.setGaitNum(4);
        v_des[0] = 0.5;
      //yaw_rate = 0;
      // pitch = 0.3;
      }


	//  yaw_rate = 0.39;
  //     }

  //     if (counter > 23250 && counter < 24000) {
	//  std::cout<< "Rest4" << std::endl;
	//  yaw_rate = 0;
  //     }
  //     if (counter > 24000 && counter < 26000) {
	//  std::cout<< "Fifth leg" << std::endl;
  //        //pitch = -0.5;
  //        v_des[0] = sqrt(3);
  //        v_des[1] = -0.08;
	//  yaw_rate = 0;
  //     }

  //     if(counter > 26000 && counter < 27250){
  //       std::cout<< "Rotating" << std::endl;
  //       //pitch = -0.6;
  //       v_des[0] = 0;
	// v_des[1] = 0;
  //       yaw_rate = 0.52;
  //     }
      
  //     if (counter > 27250 && counter < 27750) {
	//  std::cout<< "Dec5" << std::endl;
  //        //pitch = -0.5;
    
	//  yaw_rate = 0.39;
  //     }

  //     if (counter > 27750 && counter < 30500) {
	//  std::cout<< "Rest5" << std::endl;
	//  yaw_rate = 0;
  //     }
  //     if (counter > 30500 && counter < 32500) {
	//  std::cout<< "sixth leg" << std::endl;
  //        //pitch = -0.5;
  //        v_des[0] = sqrt(3);
  //        v_des[1] = -0.08;
	//  yaw_rate = 0;
  //     }

      _data->_desiredStateCommand->setStateCommands(roll, pitch, v_des, yaw_rate);
    
      Cmpc.run(*_data);
    
    //  for(int i = 0; i < 4; i++){
    //     std::cout << " leg " << i << ":  " << _data->_legController->commands[i].feedforwardForce[2] << std::endl;
    //   }
      for(int i = 0; i < 4; i++){
        QP << _data->_legController->commands[i].feedforwardForce[2] << " ";
      }
      QP << "\n";
      runQP = false;
    }

  
        
    /* =================================================================*/
    if(runQP){
      for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
      }
       // v_des[2] = 0.1;
      for (int i = 0; i < 3; i++) {
        //rpy[i] = 0;
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        
        v_act[i] = _data->_stateEstimator->getResult().vWorld(i);
        // v_des[i] = 0;
        //v_des[2] = 0.005;

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaWorld(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vWorld(i);
        
    // Set the translational and orientation gains
        kpCOM[i] = 30;   //_data->controlParameters->kpCOM(i);
        kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
        kpBase[i] = 80;     //_data->controlParameters->kpBase(i);
        kdBase[i] = 20; //  _data->controlParameters->kdBase(i);
        }

      kpCOM[2] = 50;
      kpBase[0] = 300;
      kpBase[1] = 200;

    //Vec3<double> pFeetVec;
    Vec3<double> pFeetVecCOM;

    // Get the foot locations relative to COM
      for (int leg = 0; leg < 4; leg++) {
        // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
        //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
        //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);

        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
        //std::cout << "pFeet" << leg << std::endl;
      }
     _data->_stateEstimator->setContactPhase(contactphase);

    p_des[0] = p_act[0] + (pFeet[0] + pFeet[3] + pFeet[6] + pFeet[9]) / 4.0;
    p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10]) / 4.0;
    //  myfile << "\n";
     // std::cout << j << std::endl;
     //std::cout << "run QP" << std::endl;
    balanceController.set_alpha_control(0.01);
    balanceController.set_friction(0.2);
    balanceController.set_mass(_data->_quadruped->mass);
    balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
    balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
    balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _data->_stateEstimator->getResult().rpy(2));
   // balanceController.print_QPData();
    double fOpt[12];
    balanceController.solveQP_nonThreaded(fOpt);

    std::cout << "zForce leg 0: " << fOpt[2] << std::endl;
    //balanceController.get_b_matrix(b_control);
    //b_des << b_control[2] << "\n";

  // Publish the results over ROS
  // balanceController.publish_data_lcm();

  // Copy the results to the feed forward forces
    
     //_data->_stateEstimator->run();


    for (int leg = 0; leg < 4; leg++) {
        footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
        fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame
        
        _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
        //_data->_stateEstimator->getResult().rBody.transpose() * footFeedForwardForces.col(leg); 
        // std::cout << _data->_legController->commands[leg].feedforwardForce[2]<< "\n";
    }
  
    //std::cout << j << std::endl;
    //QP << "\n";
    
    
    //_data->_legController->commands[3].feedforwardForce = _data->_legController->commands[2].feedforwardForce;
    //if(p_act[2] < 0.25){
     //  std::cout << "force" << std::endl;
    //std::cout << footFeedForwardForces << std::endl;
    //_data->_legController->updateCommand();

    // plots
    // if(counter>500){
    //  for(int i = 0; i < 4; i++){
    //    QP << _data->_legController->commands[i].feedforwardForce[2] << " ";
    //    //b_des << _data->_legController->commands[i].pDes[2] << " "; // foot pos
       
    //   }
    //   b_des << _data->_legController->commands[1].pDes[2] << " ";
    //   b_des << _data->_legController->data[1].p[2] << "\n ";
      
    //   for(int i =0; i<3; i++){
    //     //myfile << _data->_stateEstimator->getResult().rpy(i);
    //     //myfile << " " ;
    //      myfile << _data->_legController->commands[1].pDes[i] - _data->_legController->data[1].p[i] << " ";
    //   }
    //  myfile << "\n";
    //   z_pos << _data->_stateEstimator->getResult().position(2) << "\n";
    //  QP << "\n";
    //  //b_des << "\n";
    // }
    }

    _data->_legController->updateCommand();
    rate.sleep();
  
    counter++;
 
}
 
 b_des.close();
 z_pos.close();
 myfile.close();
 QP.close();
 std::cout << "stand up finished" << std::endl;
}


void FSM_State::PDstand(){
  ros::Rate rate(1000);
  int counter = 0;
  
  // initial foot possition
  Mat34<double> init_foot_pos;
  _data->_legController->updateData();
  for(int i = 0; i < 4; i++){
    init_foot_pos.col(i) = _data->_legController->data[i].p;
  }
  double h = 0.4; // standup height
  double side_sign[4] = {-1, 1, -1, 1};
  Vec3<double> ff_force_world(0.5, 0, 0);

  while(ros::ok()){
     double progress = counter * 0.001;
     if(progress > 1.)  {progress = 1.;}

    _data->_legController->updateData();  
    _data->_stateEstimator->run();    

      for(int i = 0; i < 4; i++){
        _data->_legController->commands[i].kpCartesian = Vec3<double>(400,400,900).asDiagonal();
        _data->_legController->commands[i].kdCartesian = Vec3<double>(20,20,20).asDiagonal();

        _data->_legController->commands[i].pDes << 0, side_sign[i] * 0.083, 0;
        
        _data->_legController->commands[i].pDes[2] = -h*progress + (1. - progress) * init_foot_pos(2, i);
       // _data->_legController->commands[i].feedforwardForce = _data->_stateEstimator->getResult().rBody.transpose() * ff_force_world;
       // std::cout << "leg  " << i << "  " << _data->_legController->data[i].p << std::endl;
      }

      _data->_legController->updateCommand();
      counter++;

      rate.sleep();
  }
}