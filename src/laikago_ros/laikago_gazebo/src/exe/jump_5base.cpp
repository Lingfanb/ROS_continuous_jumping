#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "laikago_msgs/LowCmd.h"
#include "laikago_msgs/LowState.h"
#include "laikago_msgs/MotorCmd.h"
#include "laikago_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "../../include/body.h"
#include <sstream>
#include <fstream>
#include "../../include/Utilities/Timer.h"

///---- Chuong add----------------------------------
#include "laikago_msgs/CheaterState.h" //
#include "../../include/CurrentState.h"// 

#include "../../include/Quadruped.h"// for robot model. use: Quadruped quad
#include "../../include/OrientationEstimator.h"// use stateEstimator->addEstimator<CheaterOrientationEstimator>();
#include "../../include/ContactEstimator.h" // use: stateEstimator->addEstimator<ContactEstimator>();
#include "../../include/PositionVelocityEstimator.h" //use: stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();

#include "../../include/DesiredCommand.h" // use: DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(_tele, &stateEstimate, dt);
//#include "../../include/teleCmd.h" // already included in DesiredCommand.h

#include "../../include/ControlFSMData.h" // 

// #include "../../include/FSM.h"
// #include "../../include/Utilities/Timer.h"

using namespace std;
using namespace laikago_model;

bool start_up = true;



class multiThread
{
public:
    multiThread(){
        imu_sub = nm.subscribe("/trunk_imu", 1, &multiThread::imuCallback, this);
        state_sub=nm.subscribe("/gazebo/model_states", 1, &multiThread::cheaterCallback,this);// chuong add
        //state_sub = nm.subscribe("/gazebo/model_states", 1, &multiThread::cheaterCallback, this);// chuong add
        footForce_sub[0] = nm.subscribe("/visual/FR_foot_contact/the_force", 1, &multiThread::FRfootCallback, this);
        footForce_sub[1] = nm.subscribe("/visual/FL_foot_contact/the_force", 1, &multiThread::FLfootCallback, this);
        footForce_sub[2] = nm.subscribe("/visual/RR_foot_contact/the_force", 1, &multiThread::RRfootCallback, this);
        footForce_sub[3] = nm.subscribe("/visual/RL_foot_contact/the_force", 1, &multiThread::RLfootCallback, this);
        servo_sub[0] = nm.subscribe("/laikago_gazebo/FR_hip_controller/state", 1, &multiThread::FRhipCallback, this);
        servo_sub[1] = nm.subscribe("/laikago_gazebo/FR_thigh_controller/state", 1, &multiThread::FRthighCallback, this);
        servo_sub[2] = nm.subscribe("/laikago_gazebo/FR_calf_controller/state", 1, &multiThread::FRcalfCallback, this);
        servo_sub[3] = nm.subscribe("/laikago_gazebo/FL_hip_controller/state", 1, &multiThread::FLhipCallback, this);
        servo_sub[4] = nm.subscribe("/laikago_gazebo/FL_thigh_controller/state", 1, &multiThread::FLthighCallback, this);
        servo_sub[5] = nm.subscribe("/laikago_gazebo/FL_calf_controller/state", 1, &multiThread::FLcalfCallback, this);
        servo_sub[6] = nm.subscribe("/laikago_gazebo/RR_hip_controller/state", 1, &multiThread::RRhipCallback, this);
        servo_sub[7] = nm.subscribe("/laikago_gazebo/RR_thigh_controller/state", 1, &multiThread::RRthighCallback, this);
        servo_sub[8] = nm.subscribe("/laikago_gazebo/RR_calf_controller/state", 1, &multiThread::RRcalfCallback, this);
        servo_sub[9] = nm.subscribe("/laikago_gazebo/RL_hip_controller/state", 1, &multiThread::RLhipCallback, this);
        servo_sub[10] = nm.subscribe("/laikago_gazebo/RL_thigh_controller/state", 1, &multiThread::RLthighCallback, this);
        servo_sub[11] = nm.subscribe("/laikago_gazebo/RL_calf_controller/state", 1, &multiThread::RLcalfCallback, this);
    }

    void imuCallback(const sensor_msgs::Imu & msg)
    { 
        lowState.imu.quaternion[0] = msg.orientation.w;
        lowState.imu.quaternion[1] = msg.orientation.x;
        lowState.imu.quaternion[2] = msg.orientation.y;
        lowState.imu.quaternion[3] = msg.orientation.z;
    }

    void cheaterCallback(const gazebo_msgs::ModelStates& msg)
    {
       lowState.cheat.orientation[0] = msg.pose[2].orientation.w;
       lowState.cheat.orientation[1] = msg.pose[2].orientation.x;
       lowState.cheat.orientation[2] = msg.pose[2].orientation.y;
       lowState.cheat.orientation[3] = msg.pose[2].orientation.z;
        
       lowState.cheat.position[0] = msg.pose[2].position.x;
       lowState.cheat.position[1] = msg.pose[2].position.y;
       lowState.cheat.position[2] = msg.pose[2].position.z;

       lowState.cheat.vWorld[0] = msg.twist[2].linear.x;
       lowState.cheat.vWorld[1] = msg.twist[2].linear.y;
       lowState.cheat.vWorld[2] = msg.twist[2].linear.z;

       lowState.cheat.omegaBody[0] = msg.twist[2].angular.x;
       lowState.cheat.omegaBody[1] = msg.twist[2].angular.y;
       lowState.cheat.omegaBody[2] = msg.twist[2].angular.z;
    }

    void FRhipCallback(const laikago_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[0].mode = msg.mode;
        lowState.motorState[0].position = msg.position;
        lowState.motorState[0].velocity = msg.velocity;
        lowState.motorState[0].torque = msg.torque;
    }

    void FRthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[1].mode = msg.mode;
        lowState.motorState[1].position = msg.position;
        lowState.motorState[1].velocity = msg.velocity;
        lowState.motorState[1].torque = msg.torque;
    }

    void FRcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[2].mode = msg.mode;
        lowState.motorState[2].position = msg.position;
        lowState.motorState[2].velocity = msg.velocity;
        lowState.motorState[2].torque = msg.torque;
    }

    void FLhipCallback(const laikago_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[3].mode = msg.mode;
        lowState.motorState[3].position = msg.position;
        lowState.motorState[3].velocity = msg.velocity;
        lowState.motorState[3].torque = msg.torque;
    }

    void FLthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[4].mode = msg.mode;
        lowState.motorState[4].position = msg.position;
        lowState.motorState[4].velocity = msg.velocity;
        lowState.motorState[4].torque = msg.torque;
    }

    void FLcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[5].mode = msg.mode;
        lowState.motorState[5].position = msg.position;
        lowState.motorState[5].velocity = msg.velocity;
        lowState.motorState[5].torque = msg.torque;
    }

    void RRhipCallback(const laikago_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[6].mode = msg.mode;
        lowState.motorState[6].position = msg.position;
        lowState.motorState[6].velocity = msg.velocity;
        lowState.motorState[6].torque = msg.torque;
    }

    void RRthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[7].mode = msg.mode;
        lowState.motorState[7].position = msg.position;
        lowState.motorState[7].velocity = msg.velocity;
        lowState.motorState[7].torque = msg.torque;
    }

    void RRcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[8].mode = msg.mode;
        lowState.motorState[8].position = msg.position;
        lowState.motorState[8].velocity = msg.velocity;
        lowState.motorState[8].torque = msg.torque;
    }

    void RLhipCallback(const laikago_msgs::MotorState& msg)
    {
        start_up = false;
        lowState.motorState[9].mode = msg.mode;
        lowState.motorState[9].position = msg.position;
        lowState.motorState[9].velocity = msg.velocity;
        lowState.motorState[9].torque = msg.torque;
    }

    void RLthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[10].mode = msg.mode;
        lowState.motorState[10].position = msg.position;
        lowState.motorState[10].velocity = msg.velocity;
        lowState.motorState[10].torque = msg.torque;
    }

    void RLcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[11].mode = msg.mode;
        lowState.motorState[11].position = msg.position;
        lowState.motorState[11].velocity = msg.velocity;
        lowState.motorState[11].torque = msg.torque;
    }

    void FRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[0].x = msg.wrench.force.x;
        lowState.eeForce[0].y = msg.wrench.force.y;
        lowState.eeForce[0].z = msg.wrench.force.z;
        lowState.footForce[0] = msg.wrench.force.z;
    }

    void FLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[1].x = msg.wrench.force.x;
        lowState.eeForce[1].y = msg.wrench.force.y;
        lowState.eeForce[1].z = msg.wrench.force.z;
        lowState.footForce[1] = msg.wrench.force.z;
    }

    void RRfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[2].x = msg.wrench.force.x;
        lowState.eeForce[2].y = msg.wrench.force.y;
        lowState.eeForce[2].z = msg.wrench.force.z;
        lowState.footForce[2] = msg.wrench.force.z;
    }

    void RLfootCallback(const geometry_msgs::WrenchStamped& msg)
    {
        lowState.eeForce[3].x = msg.wrench.force.x;
        lowState.eeForce[3].y = msg.wrench.force.y;
        lowState.eeForce[3].z = msg.wrench.force.z;
        lowState.footForce[3] = msg.wrench.force.z;
    }

private:
    ros::NodeHandle nm;
    ros::Subscriber servo_sub[8], footForce_sub[4], imu_sub, state_sub;
};


class JumpTraJectory 
{
    private:
        vector<vector<double>> QDes;
        vector<vector<double>> tauDes;
        vector<vector<double>> pfDes;
        vector<vector<double>> vfDes;
        string InputPath;
        string OutputPath;
        std::ofstream file_ofstream[36];
        vector<string> file_name;
    public:
        JumpTraJectory (const string& path1, const string& path2) : InputPath(path1), OutputPath(path2)
        {
            file_name.push_back("/Hipposition_act.txt");//0
            file_name.push_back("/Hipvelocity_act.txt");//1
            file_name.push_back("/Hiptorque_act.txt");//2
            file_name.push_back("/Thighposition_act.txt");//3
            file_name.push_back("/Thighvelocity_act.txt");//4
            file_name.push_back("/Thightorque_act.txt");//5
            file_name.push_back("/Calfposition_act.txt");//6
            file_name.push_back("/Calfvelocity_act.txt");//7
            file_name.push_back("/Calftorque_act.txt");//8     
            file_name.push_back("/COM_position_act.txt");//9

            file_name.push_back("/Hipposition_des.txt");//10
            file_name.push_back("/Hipvelocity_des.txt");//11
            file_name.push_back("/Hiptorque_des.txt");//12
            file_name.push_back("/Thighposition_des.txt");//13
            file_name.push_back("/Thighvelocity_des.txt");//14
            file_name.push_back("/Thightorque_des.txt");//15
            file_name.push_back("/Calfposition_des.txt");//16
            file_name.push_back("/Calfvelocity_des.txt");//17
            file_name.push_back("/Calftorque_des.txt");//18       
            file_name.push_back("/COM_position_des.txt");//19

            file_name.push_back("/p_FR_act.txt");//20
            file_name.push_back("/p_FL_act.txt");//21
            file_name.push_back("/p_RR_act.txt");//22
            file_name.push_back("/p_RL_act.txt");//23
            file_name.push_back("/v_FR_act.txt");//24
            file_name.push_back("/v_FL_act.txt");//25
            file_name.push_back("/v_RR_act.txt");//26
            file_name.push_back("/v_RL_act.txt");//27

            file_name.push_back("/p_FR_des.txt");//28
            file_name.push_back("/p_FL_des.txt");//29
            file_name.push_back("/p_RR_des.txt");//30
            file_name.push_back("/p_RL_des.txt");//31
            file_name.push_back("/v_FR_des.txt");//32
            file_name.push_back("/v_FL_des.txt");//33
            file_name.push_back("/v_RR_des.txt");//34
            file_name.push_back("/v_RL_des.txt");//35               
        }
        void ReadCsvData(const string& data_path,vector<vector<double>>& Vec2D)
        {
            std::ifstream myFile;  
            myFile.open(data_path.c_str());
            if(!myFile.is_open())
                {
                    std::cout << "Could not open file" + data_path << std::endl;
                }
            cout << "Reading: " + data_path << endl;
            string line; // the number of line is 34 (size of Q)
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
                Vec2D.push_back(Q);
            }
            myFile.close();
        }
        void GetInput()
        {
            
            const string QDes_path = this->InputPath + "/data_Q.csv";
            const string tauDes_path = this->InputPath + "/data_tau.csv";
            const string pfDes_path = this->InputPath + "/data_pf.csv";
            const string vfDes_path = this->InputPath + "/data_vf.csv";
            this->ReadCsvData(QDes_path,QDes);
            this->ReadCsvData(tauDes_path,tauDes);
            this->ReadCsvData(pfDes_path,pfDes);
            this->ReadCsvData(vfDes_path,vfDes);
            // this->QDes = QDes;
            // this->tauDes = tauDes;
            // this->pfDes = pfDes;
            // this->vfDes = vfDes;
        }

        void JumpAndWrite (const double Kp, const double Kd, const double Hip_Kp, double Hip_Kd, 
                           const double KpCartesian,const double KdCartesian,const double Hip_KpCartesian,const double Hip_KdCartesian)
        {
            static int count = 0;
            if(count ==0)
            {
                for(int i=0; i<36; i++)
                {
                    file_ofstream[i].open(OutputPath + file_name[i],std::fstream::trunc);
                }

            }
            else 
            {
                for(int i=0; i<36; i++)
                {
                    file_ofstream[i].open(OutputPath + file_name[i],std::fstream::app);
                }
                

            }
            std::cout << "init Joint Parameters and Stand Up" << std::endl;
            std::cout << ros::Time::now() << std::endl;
            Jump_Init();
            std::cout << ros::Time::now() << std::endl;
            Quadruped quad;
            quad.setQuadruped(2);
            LegController* legController = new LegController(quad);
            ros::Rate r(1000); 
            legController->updateData();
            
            if(ros::ok()){
                usleep(2000000); //2s

                std::cout << "Start jumping" << std::endl;
                Timer t1;
                t1.start();


                for(int i = 0; i < QDes[0].size();i++)
                {
                    legController->updateData();
                    
                    // Using for Debug
                    // ORDER: FR, FL, RR, RL
                    /*
                    file_ofstream[20]<< legController ->data[0].p(0)<< " " << legController ->data[0].p(1)<<" "<< legController ->data[0].p(2)<<endl; //FR
                    file_ofstream[21]<< legController ->data[1].p(0)<< " " << legController ->data[1].p(1)<<" "<< legController ->data[1].p(2)<<endl; //FL
                    file_ofstream[22]<< legController ->data[2].p(0)<< " " << legController ->data[2].p(1)<<" "<< legController ->data[2].p(2)<<endl; //RR
                    file_ofstream[23]<< legController ->data[3].p(0)<< " " << legController ->data[3].p(1)<<" "<< legController ->data[3].p(2)<<endl; //RL

                    file_ofstream[24]<< legController ->data[0].v(0)<< " " << legController ->data[0].v(1)<<" "<< legController ->data[0].v(2)<<endl; //FR
                    file_ofstream[25]<< legController ->data[1].v(0)<< " " << legController ->data[1].v(1)<<" "<< legController ->data[1].v(2)<<endl; //FL
                    file_ofstream[26]<< legController ->data[2].v(0)<< " " << legController ->data[2].v(1)<<" "<< legController ->data[2].v(2)<<endl; //RR
                    file_ofstream[27]<< legController ->data[3].v(0)<< " " << legController ->data[3].v(1)<<" "<< legController ->data[3].v(2)<<endl; //RL

                    

                    file_ofstream[10] << QDes[8][i] << " " << QDes[5][i] << " " << QDes[14][i] <<  " " << QDes[11][i] << endl;           
                    file_ofstream[11] << QDes[25][i] << " " << QDes[22][i] << " " <<QDes[31][i] <<  " " << QDes[28][i]  << endl;
                    file_ofstream[12] << tauDes[3][i] << " " << tauDes[0][i] << " " << tauDes[9][i] <<  " " << tauDes[6][i]  << endl;
                    file_ofstream[13] << QDes[9][i] << " " << QDes[6][i] << " " << QDes[15][i] <<  " " << QDes[12][i] << endl;           
                    file_ofstream[14] << QDes[26][i] << " " << QDes[23][i] << " " <<QDes[32][i] <<  " " << QDes[29][i]  << endl;
                    file_ofstream[15] << tauDes[4][i] << " " << tauDes[1][i] << " " << tauDes[10][i] <<  " " << tauDes[7][i]  << endl;
                    file_ofstream[16] << QDes[10][i] << " " << QDes[7][i] << " " <<QDes[16][i] <<  " " << QDes[13][i] << endl;
                    file_ofstream[17] << QDes[27][i] << " " << QDes[24][i] << " " <<QDes[33][i] <<  " " << QDes[30][i] << endl;
                    file_ofstream[18] << tauDes[5][i] << " " << tauDes[2][i] << " " << tauDes[11][i] <<  " " << tauDes[8][i] << endl;
                    file_ofstream[19] << QDes[0][i] <<" " << QDes[1][i] <<" "<< QDes[2][i]<< endl;

                    file_ofstream[0] << lowState.motorState[0].position << " " << lowState.motorState[3].position << " " <<lowState.motorState[6].position <<  " " << lowState.motorState[9].position << endl;
                    file_ofstream[1] << lowState.motorState[0].velocity << " " << lowState.motorState[3].velocity << " " <<lowState.motorState[6].velocity <<  " " << lowState.motorState[9].velocity << endl;
                    file_ofstream[2] << lowState.motorState[0].torque << " " << lowState.motorState[3].torque << " " <<lowState.motorState[6].torque <<  " " << lowState.motorState[9].torque << endl;
                    file_ofstream[3] << lowState.motorState[1].position << " " << lowState.motorState[4].position << " " <<lowState.motorState[7].position <<  " " << lowState.motorState[10].position << endl;           
                    file_ofstream[4] << lowState.motorState[1].velocity << " " << lowState.motorState[4].velocity << " " <<lowState.motorState[7].velocity <<  " " << lowState.motorState[10].velocity << endl;
                    file_ofstream[5] << lowState.motorState[1].torque << " " << lowState.motorState[4].torque << " " <<lowState.motorState[7].torque <<  " " << lowState.motorState[10].torque << endl;
                    file_ofstream[6] << lowState.motorState[2].position << " " << lowState.motorState[5].position << " " <<lowState.motorState[8].position <<  " " << lowState.motorState[11].position << endl;
                    file_ofstream[7] << lowState.motorState[2].velocity << " " << lowState.motorState[5].velocity << " " <<lowState.motorState[8].velocity <<  " " << lowState.motorState[11].velocity << endl;
                    file_ofstream[8] << lowState.motorState[2].torque << " " << lowState.motorState[5].torque << " " <<lowState.motorState[8].torque <<  " " << lowState.motorState[11].torque << endl;
                    file_ofstream[9] << lowState.cheat.position[0] <<" " << lowState.cheat.position[1] << " " << lowState.cheat.position[2] << endl;
                    */
                    // Joint PD controller via LegController.cpp
                    
                    /*
                    for(int i = 0; i <4; i++){
                        legController->commands[i].kpJoint << Hip_Kp, 0, 0,
                                                            0, Kp, 0,
                                                            0, 0, Kp;

                        legController->commands[i].kdJoint << Hip_Kd, 0, 0,
                                                            0, Kd, 0,
                                                            0, 0, Kd;
                        
                        legController ->commands[i].kpCartesian << Hip_KpCartesian, 0, 0,
                                                                    0 ,KpCartesian, 0,
                                                                    0, 0, KpCartesian;
                        legController ->commands[i].kdCartesian << Hip_KdCartesian, 0, 0,
                                                                    0 , KdCartesian, 0,
                                                                    0, 0, KdCartesian;
                        
                    }
                    */
                    
                    // motor: ROS definition
                    // FR (motor 0,1,2); FL(motor 3,4,5); RR(motor 6,7,8); RL(motor 9,10,11)
                    // motor 0 -> hip
                    // Commands: torque, qDes and qdDes
                    legController->commands[0].tau << tauDes[3][i], tauDes[4][i], tauDes[5][i];// front right leg
                    legController->commands[1].tau << tauDes[0][i], tauDes[1][i], tauDes[2][i]; // front left leg
                    legController->commands[2].tau << tauDes[9][i], tauDes[10][i], tauDes[11][i];// rear right leg
                    legController->commands[3].tau << tauDes[6][i], tauDes[7][i], tauDes[8][i]; // rear left leg

                    legController->commands[0].qDes << QDes[8][i], QDes[9][i], QDes[10][i]; // front right leg
                    legController->commands[1].qDes << QDes[5][i], QDes[6][i], QDes[7][i]; // front left leg
                    legController->commands[2].qDes << QDes[14][i], QDes[15][i], QDes[16][i];// rear right leg
                    legController->commands[3].qDes << QDes[11][i],QDes[12][i], QDes[13][i]; // rear left leg

                    legController->commands[0].qdDes << QDes[25][i], QDes[26][i], QDes[27][i]; // front right leg
                    legController->commands[1].qdDes << QDes[22][i], QDes[23][i], QDes[24][i]; // front left leg
                    legController->commands[2].qdDes << QDes[31][i], QDes[32][i], QDes[33][i];// rear right leg
                    legController->commands[3].qdDes << QDes[28][i], QDes[29][i], QDes[30][i]; // rear left leg

                    // Commands: position and velocity of foot
                    /*
                    legController->commands[0].pDes << pfDes[3][i], pfDes[4][i], pfDes[5][i];// front right leg
                    legController->commands[1].pDes << pfDes[0][i], pfDes[1][i], pfDes[2][i];// front left leg
                    legController->commands[2].pDes << pfDes[9][i], pfDes[10][i],pfDes[11][i];// rear right leg
                    legController->commands[3].pDes << pfDes[6][i], pfDes[7][i], pfDes[8][i];// rear left leg

                    legController->commands[0].vDes << vfDes[3][i], vfDes[4][i], vfDes[5][i];// front right leg
                    legController->commands[1].vDes << vfDes[0][i], vfDes[1][i], vfDes[2][i];// front left leg
                    legController->commands[2].vDes << vfDes[9][i], vfDes[10][i], vfDes[11][i];// rear right leg
                    legController->commands[3].vDes << vfDes[6][i], vfDes[7][i], vfDes[8][i];// rear left leg
                    */
                    
                    /*
                    file_ofstream[28] << pfDes[3][i] <<" "<< pfDes[4][i]<< " "<< pfDes[5][i] <<endl;
                    file_ofstream[29] << pfDes[0][i] <<" "<< pfDes[1][i]<< " "<< pfDes[2][i] <<endl;
                    file_ofstream[30] << pfDes[9][i] <<" "<< pfDes[10][i]<< " "<< pfDes[11][i] <<endl;
                    file_ofstream[31] << pfDes[6][i] <<" "<< pfDes[7][i]<< " "<< pfDes[8][i] <<endl;
                    file_ofstream[32] << vfDes[3][i] <<" "<< vfDes[4][i]<< " "<< vfDes[5][i] <<endl;
                    file_ofstream[33] << vfDes[0][i] <<" "<< vfDes[1][i]<< " "<< vfDes[2][i] <<endl;
                    file_ofstream[34] << vfDes[9][i] <<" "<< vfDes[10][i]<< " "<< vfDes[11][i] <<endl;
                    file_ofstream[35] << vfDes[6][i] <<" "<< vfDes[7][i]<< " "<< vfDes[8][i] <<endl;
                    */
                    
                    legController->updateCommand_jump3D();
                    r.sleep();
                } 
                printf("Jumping time %f ms\n", t1.getMs());
                std::cout << "End jumping" << std::endl;
            }
            

            for(int i=0; i<36; i++)
            {
                file_ofstream[i].close();
            }
            delete legController;
            Jump_Init();
            count++;
        }

};

int main(int argc, char **argv)
{
    // JumpTraJectory yaw_90("src/laikago_ros/optimization_data/jump_yaw/jump_yaw_base_ry_rz","src/laikago_ros/result/jump_yaw/jump_yaw_base_ry_rz/yaw_90d");
    // JumpTraJectory yaw_30d("src/laikago_ros/optimization_data/jump_up/jump_up_ry_rz","src/laikago_ros/result/jump_up/jump_up_ry_rz");
    // yaw_30d.GetInput();
     JumpTraJectory* diagonal = new JumpTraJectory("src/laikago_ros/optimization_data/jump_diagonal/chuong_may2021","src/laikago_ros/result/jump_diagonal/chuong_may2021");
    //JumpTraJectory* backflip = new JumpTraJectory("src/laikago_ros/optimization_data/jump_backflip","src/laikago_ros/result/jump_backflip");

    //backflip->GetInput();
     diagonal->GetInput();


    
    ros::init(argc, argv, "laikago_gazebo_jump");

    multiThread listen_publish_obj;
    ros::AsyncSpinner spinner(1); // one threads
    spinner.start();
    usleep(2000000); // must wait 2s, to get first state
    ros::NodeHandle n;
    //ros::Publisher lowState_pub; //for rviz visualization

    // Publish the command every 1ms (1000Hz)
    ros::Rate r(1000); 
    //lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
    servo_pub[0] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_calf_controller/command", 1);

          /////////////// CHUONG ADD on Dec 2 --------------------------------------------------------------
    std::cout << "initialize" << std::endl;
    double dt = 0.001; 

    // robot model
    Quadruped quad;
    quad.setQuadruped(2); // 1 for Aliengo, 2 for A1
    
       // initialize new leg controller and state estimate object
    StateEstimate stateEstimate;
    // yaw_30d.JumpAndWrite(250,2.5,250,2.5,0,0,0,0);
    diagonal->JumpAndWrite(250,10,250,10,0,0,0,0);
    //backflip->JumpAndWrite(300,6,300,6,0,0,0,0);
    //delete backflip;
    delete diagonal;

    return 0;
}
