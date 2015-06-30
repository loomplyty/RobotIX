/*
 * Gait.cpp
 *
 *  Created on: Nov 28, 2014
 *      Author: hex
 */
#include"Gait.h"
#include <fstream>
#include <iostream>
#include <lapacke.h>
using namespace Hexapod_Robot;



bool CGait::isReadytoSetGait[AXIS_NUMBER];
EGAIT CGait::m_currentGait[AXIS_NUMBER];
long long int CGait::m_gaitStartTime[AXIS_NUMBER];
int CGait::m_gaitCurrentIndex[AXIS_NUMBER];
int CGait::m_gaitCurrentIndexForPID[AXIS_NUMBER];
EGaitState CGait::m_gaitState[AXIS_NUMBER];
Aris::RT_CONTROL::CMotorData CGait::m_standStillData[AXIS_NUMBER];
Aris::RT_CONTROL::CMotorData CGait::m_commandDataMapped[AXIS_NUMBER];
Aris::RT_CONTROL::CMotorData CGait::m_feedbackDataMapped[AXIS_NUMBER];
 CTrotGait CGait::Trot;
 int CGait::Gait_iter[AXIS_NUMBER];
int CGait::Gait_iter_count[AXIS_NUMBER];
bool CGait::IsConsFinished[AXIS_NUMBER];
bool CGait::IsHomeStarted[AXIS_NUMBER];

double CGait::online_ideal_foot_pos[18] ;
double CGait::online_ideal_body_pos[6] ;

double CGait::online_static_foot_pos[18];
double CGait::online_static_body_pos[6];
double CGait::online_static_screw_pos[18];

double CGait::online_ideal_screw_vel[18];
double CGait::online_last_ideal_screw_vel[18];

double CGait::online_ideal_screw_pos[18] ;
double CGait::online_last_ideal_screw_pos[18];
double CGait::online_second_last_ideal_screw_pos[18];

//test
double CGait::online_ideal_screw_pos_before_PID[18];
double CGait::online_ideal_foot_pos_before_PID[18];


//double CGait::online_actual_screw_pos[18] ;
//double CGait::online_actual_screw_vel[18] ;

double CGait::online_angle[3];
double CGait::online_angleVel[3];
double CGait::online_linearAcc[3];
double CGait::online_IMU_313[6];
ROBOT CGait::robot;
double CGait::online_body_height;
double CGait::online_stance_leg_height[2];
double CGait::online_stance_leg_pos[18];
//double CGait::online_actual_foot_pos[18];

int Gait_online_Acc_Length;
int Gait_online_Cons_Length;
int Gait_online_Dec_Length;


///////////////////////////////////////////////
int GaitMove[GAIT_MOVE_LEN][GAIT_WIDTH];
int GaitMoveBack[GAIT_MOVEBACK_LEN][GAIT_WIDTH];

int GaitHome2Start[GAIT_HOME2START_LEN][GAIT_WIDTH];

///////////////////////////////////////////////
int GaitMove_Acc[GAIT_ACC_LEN][GAIT_WIDTH];
int GaitMove_Cons[GAIT_CON_LEN][GAIT_WIDTH];
int GaitMove_Dec[GAIT_DEC_LEN][GAIT_WIDTH];

int GaitMoveBack_Acc[GAIT_ACC_LEN][GAIT_WIDTH];
int GaitMoveBack_Cons[GAIT_CON_LEN][GAIT_WIDTH];
int GaitMoveBack_Dec[GAIT_DEC_LEN][GAIT_WIDTH];

int GaitFastMove_Acc[GAIT_FAST_ACC_LEN][GAIT_WIDTH];
int GaitFastMove_Cons[GAIT_FAST_CON_LEN][GAIT_WIDTH];
int GaitFastMove_Dec[GAIT_FAST_DEC_LEN][GAIT_WIDTH];

int GaitFastMoveBack_Acc[GAIT_FAST_ACC_LEN][GAIT_WIDTH];
int GaitFastMoveBack_Cons[GAIT_FAST_CON_LEN][GAIT_WIDTH];
int GaitFastMoveBack_Dec[GAIT_FAST_DEC_LEN][GAIT_WIDTH];

int GaitTrot_Acc[GAIT_TROT_ACC_LEN][GAIT_WIDTH];
int GaitTrot_Cons[GAIT_TROT_CON_LEN][GAIT_WIDTH];
int GaitTrot_Dec[GAIT_TROT_DEC_LEN][GAIT_WIDTH];

int GaitLegUp[GAIT_LEGUP_LEN][GAIT_WIDTH];

int GaitTurnLeft[GAIT_TURN_LEN][GAIT_WIDTH];

int GaitTurnRight[GAIT_TURN_LEN][GAIT_WIDTH];


const int MapAbsToPhy[18]=
{
		10,	11,	9,
		12,	14,	13,
		17,	15,	16,
		6,	8,	7,
		3,	5,	4,
		0,	2,	1
};
const int MapPhyToAbs[18]=
{
		15,	17,	16,
		12,	14,	13,
		9,	11,	10,
		2,	0,	1,
		3,	5,	4,
		7,	8,	6
};

void CGait::MapFeedbackDataIn(Aris::RT_CONTROL::CMachineData& p_data )
{
	for(int i=0;i<MOTOR_NUM;i++)
	{

		m_feedbackDataMapped[i].Position=p_data.feedbackData[MapAbsToPhy[i]].Position;
		m_feedbackDataMapped[i].Velocity=p_data.feedbackData[MapAbsToPhy[i]].Velocity;
		m_feedbackDataMapped[i].Torque=p_data.feedbackData[MapAbsToPhy[i]].Torque;
	}
};
void CGait::MapCommandDataOut(Aris::RT_CONTROL::CMachineData& p_data )
{
	for(int i=0;i<MOTOR_NUM;i++)
	{
		p_data.commandData[i].Position=m_commandDataMapped[MapPhyToAbs[i]].Position;
		p_data.commandData[i].Velocity=m_commandDataMapped[MapPhyToAbs[i]].Velocity;
		p_data.commandData[i].Torque=m_commandDataMapped[MapPhyToAbs[i]].Torque;
	}
};

CGait::CGait()
{
	for(int i=1;i<AXIS_NUMBER;i++)
	{
		CGait::m_currentGait[i]=EGAIT::GAIT_NULL;
		CGait::m_gaitState[i]=EGaitState::NONE;
		CGait::IsHomeStarted[i]=false;
		CGait::IsConsFinished[i]=false;
		CGait::Gait_iter[i]=1;
		CGait::Gait_iter_count[i]=0;
	}
};

CGait::~CGait()
{
};

int CGait::GetIMUData(double * angle, double* angleVel, double* linearAcc)
{
	memcpy(online_angle,angle,sizeof(double)*3);
	memcpy(online_angleVel,angleVel,sizeof(double)*3);
    memcpy(online_linearAcc,linearAcc,sizeof(double)*3);
}

void CGait::online_DoPID(int N,Aris::RT_CONTROL::CMachineData& p_data)//double screw_pos,double screw_vel,
{
//**********************************CALCULATON OF EP_ROBOT_313***************************//
    double IMU_body_pos[6];
    double pm_IMU2G0[4][4];
    double ep_IMU_321[6];
    double pm_ROBOT[4][4];
    double ep_ROBOT_313[6];
    static double angle_trans[3]={0,0,0};


   // if(IMU_mutex.try_lock())
   // {
    	for (int i=0;i<3;i++)
    	    {
    	    	angle_trans[2-i]=online_angle[i];
    	    }
    	angle_trans[0]=PI;

    	//rt_printf("suceesfully get IMU data");
    	//IMU_mutex.unlock();
   // }
   // else
   // {
    	//rt_printf("Data is locked");
  //  }
    //std::u<std::mutex> lck(IMU_mutex);


  //  memcpy(online_angle,angle_trans,sizeof(double)*3);
    memcpy(ep_IMU_321,angle_trans,sizeof(double)*3);

    double pm_G02G[4][4]=
    {
    		{-1,0,0,0},
			{0,0,1,0},
			{0,1,0,0},
			{0,0,0,1},
    };
 //   ep_IMU_321[0]=PI;
    Aris::DynKer::s_ep2pm(ep_IMU_321,*pm_IMU2G0,"321");

    double pm_R2IMU[4][4]=
    {
    		{1,0,0,0},
    		{0,0,1,0},
			{0,-1,0,0},
			{0,0,0,1},
    };

    double pm_R2G0[4][4];
    double pm_R2G[4][4];
    Aris::DynKer::s_pm_dot_pm(*pm_IMU2G0,*pm_R2IMU,*pm_R2G0);
    Aris::DynKer::s_pm_dot_pm(*pm_G02G,*pm_R2G0,*pm_R2G);
    Aris::DynKer::s_pm2ep(*pm_R2G,ep_ROBOT_313,"313");
    memcpy(online_IMU_313, ep_ROBOT_313, sizeof (double)*6);

    //*************************CALCULATION OF IDEAL SCREW INPUT*****************************************************************///
	//screw_pos count
	//screw_vel count/s
	double VelMax=0.23; // m/s
	double AccMax=0.8;  // m/s^2;
	double SafetyDistance = 0.5*(VelMax*VelMax)/AccMax;
     //rt_printf("N=%d\n",N);

    Trot.CalPee(N,online_static_foot_pos,online_static_body_pos);// foot_pos static
    Trot.CalPin(online_static_screw_pos);
   // rt_printf("leg phase %d %d %d %d %d %d\n",Trot.m_leg_phase[0],Trot.m_leg_phase[1],Trot.m_leg_phase[2],Trot.m_leg_phase[3],Trot.m_leg_phase[4],Trot.m_leg_phase[5]);

/*    rt_printf("online static screw pos\n");
        rt_printf("%f,%f,%f\n",online_static_screw_pos[0],online_static_screw_pos[1],online_static_screw_pos[2]);
        rt_printf("%f,%f,%f\n",online_static_screw_pos[3],online_static_screw_pos[4],online_static_screw_pos[5]);
        rt_printf("%f,%f,%f\n",online_static_screw_pos[6],online_static_screw_pos[7],online_static_screw_pos[8]);
        rt_printf("%f,%f,%f\n",online_static_screw_pos[9],online_static_screw_pos[10],online_static_screw_pos[11]);
        rt_printf("%f,%f,%f\n",online_static_screw_pos[12],online_static_screw_pos[13],online_static_screw_pos[14]);
        rt_printf("%f,%f,%f\n",online_static_screw_pos[15],online_static_screw_pos[16],online_static_screw_pos[17]);
*/

/*  rt_printf("body pos\n");
    rt_printf("%f,%f,%f\n",online_ideal_foot_pos[0],online_ideal_foot_pos[1],online_ideal_foot_pos[2]);
    rt_printf("%f,%f,%f\n",online_ideal_foot_pos[3],online_ideal_foot_pos[4],online_ideal_foot_pos[5]);
    rt_printf("%f,%f,%f\n",online_ideal_foot_pos[6],online_ideal_foot_pos[7],online_ideal_foot_pos[8]);
*/
    double TargetXZ[2];
    double H0=0.85;
    double delta_height;

    double Kp_stance=10;
    double Kp_swing=26;
    double Ki=0;
    double Kd=0;//minus

    ETrotGaitPhase LegPhase[6];

    double body_zero[6]={0, 0, 0, 0, 0, 0};
    double IMU_angleVel[3]={0.2,0.2,0};
    CGait::robot.SetPin(online_static_screw_pos, ep_ROBOT_313);//Set body euler & pos//,ep_ROBOT_313
    //CGait::robot.SetPin(online_static_screw_pos,body_zero);

    static int count_angle;
    if(count_angle%10==0)
    {
        // rt_printf("old %f %f %f,new  %f %f %f\n",online_angle[0],online_angle[1],online_angle[2],ep_ROBOT_313[0],ep_ROBOT_313[1],ep_ROBOT_313[2]);

    }
    count_angle++;

     for(int i=0;i<6;i++)
    {

    	if(Trot.m_leg_phase[i]==GaitStance)
    	{
    		CGait::robot.pLegs[i]->SetPin(online_static_screw_pos+3*i);
        	CGait::robot.pLegs[i]->GetPee(online_ideal_foot_pos+3*i);
        	if(i<=2)
        		online_stance_leg_height[0]=-online_ideal_foot_pos[3*i+1];//destination of y for swing leg
        	else
        		online_stance_leg_height[1]=-online_ideal_foot_pos[3*i+1];//destination of y for swing leg

    	}
    }
    online_body_height=(online_stance_leg_height[0]+online_stance_leg_height[1])/2;
    delta_height=H0-online_body_height;

    //rt_printf("delta height:%f\n",delta_height);

    if(N==0)
    {
    	memcpy(online_last_ideal_screw_pos,online_static_screw_pos,sizeof(double)*18);

    	memcpy(online_second_last_ideal_screw_pos,online_static_screw_pos,sizeof(double)*18);

    	memcpy(online_stance_leg_pos,online_static_foot_pos,sizeof(double)*18);
    }
   /* rt_printf("online ideal last screw pos\n");
        rt_printf("%f,%f,%f\n",online_last_ideal_screw_pos[0],online_last_ideal_screw_pos[1],online_last_ideal_screw_pos[2]);
        rt_printf("%f,%f,%f\n",online_last_ideal_screw_pos[3],online_last_ideal_screw_pos[4],online_last_ideal_screw_pos[5]);
        rt_printf("%f,%f,%f\n",online_last_ideal_screw_pos[6],online_last_ideal_screw_pos[7],online_last_ideal_screw_pos[8]);
        rt_printf("%f,%f,%f\n",online_last_ideal_screw_pos[9],online_last_ideal_screw_pos[10],online_last_ideal_screw_pos[11]);
        rt_printf("%f,%f,%f\n",online_last_ideal_screw_pos[12],online_last_ideal_screw_pos[13],online_last_ideal_screw_pos[14]);
        rt_printf("%f,%f,%f\n",online_last_ideal_screw_pos[15],online_last_ideal_screw_pos[16],online_last_ideal_screw_pos[17]);
*/
  //   rt_printf("leg phase: %d %d %d %d %d %d\n",Trot.m_leg_phase[0],Trot.m_leg_phase[1],Trot.m_leg_phase[2],Trot.m_leg_phase[3],Trot.m_leg_phase[4],Trot.m_leg_phase[5]);

    for(int i=0;i<6;i++)
     {
    	switch(Trot.m_leg_phase[i])
        {
        case GaitStance:
           // rt_printf("leg ID: %d\n",i);
        	memcpy(online_ideal_foot_pos+3*i,online_stance_leg_pos+3*i,sizeof(double)*3);
       //rt_printf("foot_pos before PID, %f %f %f\n",online_ideal_foot_pos[3*i],online_ideal_foot_pos[3*i+1],online_ideal_foot_pos[3*i+2]);
            rt_printf("foot y %f\n",online_ideal_foot_pos[3*i+1]);
        	online_ideal_foot_pos[3*i+1]=-H0;

            memcpy(online_ideal_foot_pos_before_PID+3*i,online_ideal_foot_pos+3*i,sizeof(double)*3);

            // rt_printf("online_body_height:%f\n",online_body_height);
            CGait::robot.SetPee(online_ideal_foot_pos,body_zero);
            CGait::robot.pLegs[i]->SetPee(online_ideal_foot_pos+3*i);
            CGait::robot.pLegs[i]->GetPin(online_ideal_screw_pos+3*i);

        	memcpy(online_ideal_screw_pos+3*i,online_static_screw_pos+3*i,sizeof(double)*3);
            memcpy(online_ideal_screw_pos_before_PID+3*i,online_ideal_screw_pos+3*i,sizeof(double)*3);

            for(int j=0;j<3;j++)
            {
            	//rt_printf("leg %d, static screw_pos %f\n",i,online_static_screw_pos[3*i+j]);
                //rt_printf("screw_input:%f\n",online_ideal_screw_pos[3*i+j]);
                //rt_printf("screw_last input:%f\n",online_last_ideal_screw_pos[3*i+j]);

              	online_ideal_screw_vel[3*i+j]=Kp_stance*(online_ideal_screw_pos[3*i+j]-online_last_ideal_screw_pos[3*i+j])+
            	 		                           Kd/0.001*(online_ideal_screw_pos[3*i+j]+online_second_last_ideal_screw_pos[3*i+j]-2*online_last_ideal_screw_pos[3*i+j])+
				 								   Ki*0.001*online_ideal_screw_pos[3*i+j];
               // rt_printf("1.screw_vel_input:%f\n",online_ideal_screw_vel[3*i+j]);

            	if(online_ideal_screw_vel[3*i+j]-online_last_ideal_screw_vel[3*i+j]>AccMax*0.001)
            		online_ideal_screw_vel[3*i+j]=online_last_ideal_screw_vel[3*i+j]+AccMax*0.001;
            	else if(online_ideal_screw_vel[3*i+j]-online_last_ideal_screw_vel[3*i+j]<-AccMax*0.001)
            		online_ideal_screw_vel[3*i+j]=online_last_ideal_screw_vel[3*i+j]-AccMax*0.001;
               // rt_printf("2.screw_vel_input:%f, last vel: %f \n",online_ideal_screw_vel[3*i+j],online_last_ideal_screw_vel[3*i+j]);

                if(online_ideal_screw_vel[3*i+j]>VelMax)
            		online_ideal_screw_vel[3*i+j]=VelMax;
            	else if(online_ideal_screw_vel[3*i+j]<-VelMax)
            		online_ideal_screw_vel[3*i+j]=-VelMax;
               // rt_printf("3.screw_vel_input:%f\n",online_ideal_screw_vel[3*i+j]);

            	online_ideal_screw_pos[3*i+j]=online_last_ideal_screw_pos[3*i+j]+online_ideal_screw_vel[3*i+j]/1000;
               //rt_printf("screw_input final:%f\n",online_ideal_screw_pos[3*i+j]);
            }
            CGait::robot.SetPee(online_ideal_foot_pos,ep_ROBOT_313);
            CGait::robot.pLegs[i]->SetPin(online_ideal_screw_pos+3*i);
            CGait::robot.pLegs[i]->GetPee(online_ideal_foot_pos+3*i);
        	//rt_printf("foot_pos after PID, %f %f %f\n",online_ideal_foot_pos[3*i],online_ideal_foot_pos[3*i+1],online_ideal_foot_pos[3*i+2]);

        	break;

        case GaitSwing:

          //  rt_printf("leg ID: %d\n",i);
        	//CGait::robot.pLegs[i]->SetPin(online_static_screw_pos+3*i);
           // CGait::robot.pLegs[i]->GetPee(online_ideal_foot_pos+3*i);

        	memcpy(online_ideal_foot_pos+3*i,online_static_foot_pos+3*i,sizeof(double)*3);
            online_ideal_foot_pos[3*i+1]=online_static_foot_pos[3*i+1]+delta_height;//x,z stay the same with static traj
/*
            //CGait::online_trot_TargetXZ(IMU_angleVel,online_body_height,i,TargetXZ);
            CGait::online_trot_TargetXZ(online_angleVel,online_body_height,i,TargetXZ);
            online_ideal_foot_pos[3*i]=online_static_foot_pos[3*i]+TargetXZ[0];
            online_ideal_foot_pos[3*i+2]=online_static_foot_pos[3*i+2]+TargetXZ[1];
*/
        	//rt_printf("foot_pos before PID, %f %f %f\n",online_ideal_foot_pos[3*i],online_ideal_foot_pos[3*i+1],online_ideal_foot_pos[3*i+2]);

            memcpy(online_ideal_foot_pos_before_PID+3*i,online_ideal_foot_pos+3*i,sizeof(double)*3);

            // rt_printf("online_body_height:%f\n",online_body_height);

           // CGait::robot.SetPee(online_ideal_foot_pos,ep_ROBOT_313);
            CGait::robot.pLegs[i]->SetPee(online_ideal_foot_pos+3*i);
            CGait::robot.pLegs[i]->GetPin(online_ideal_screw_pos+3*i);

            memcpy(online_ideal_screw_pos_before_PID+3*i,online_ideal_screw_pos+3*i,sizeof(double)*3);

            for(int j=0;j<3;j++)
            {
            	//rt_printf("leg %d, static screw_pos %f\n",i,online_static_screw_pos[3*i+j]*350*65536);

               // rt_printf("screw_input:%f\n",online_ideal_screw_pos[3*i+j]);
                //rt_printf("screw_last input:%f\n",online_last_ideal_screw_pos[3*i+j]);

              	online_ideal_screw_vel[3*i+j]=Kp_swing*(online_ideal_screw_pos[3*i+j]-online_last_ideal_screw_pos[3*i+j])+
            	 		                           Kd/0.001*(online_ideal_screw_pos[3*i+j]+online_second_last_ideal_screw_pos[3*i+j]-2*online_last_ideal_screw_pos[3*i+j])+
				 								   Ki*0.001*online_ideal_screw_pos[3*i+j];
                //rt_printf("1.screw_vel_input:%f\n",online_ideal_screw_vel[3*i+j]);

            	if(online_ideal_screw_vel[3*i+j]-online_last_ideal_screw_vel[3*i+j]>AccMax*0.001)
            		online_ideal_screw_vel[3*i+j]=online_last_ideal_screw_vel[3*i+j]+AccMax*0.001;
            	else if(online_ideal_screw_vel[3*i+j]-online_last_ideal_screw_vel[3*i+j]<-AccMax*0.001)
            		online_ideal_screw_vel[3*i+j]=online_last_ideal_screw_vel[3*i+j]-AccMax*0.001;
               // rt_printf("2.screw_vel_input:%f, last vel: %f \n",online_ideal_screw_vel[3*i+j],online_last_ideal_screw_vel[3*i+j]);

                if(online_ideal_screw_vel[3*i+j]>VelMax)
            		online_ideal_screw_vel[3*i+j]=VelMax;
            	else if(online_ideal_screw_vel[3*i+j]<-VelMax)
            		online_ideal_screw_vel[3*i+j]=-VelMax;
               // rt_printf("3.screw_vel_input:%f\n",online_ideal_screw_vel[3*i+j]);

            	online_ideal_screw_pos[3*i+j]=online_last_ideal_screw_pos[3*i+j]+online_ideal_screw_vel[3*i+j]/1000;
              // rt_printf("screw_input final:%f\n",online_ideal_screw_pos[3*i+j]);
            }
            CGait::robot.pLegs[i]->SetPin(online_ideal_screw_pos+3*i);
            CGait::robot.pLegs[i]->GetPee(online_ideal_foot_pos+3*i);

            memcpy(online_stance_leg_pos+3*i,online_ideal_foot_pos+3*i,sizeof(double)*3);
        	//rt_printf("foot_pos after PID, %f %f %f\n",online_ideal_foot_pos[3*i],online_ideal_foot_pos[3*i+1],online_ideal_foot_pos[3*i+2]);

        	break;
        case Invalid:
            //rt_printf("leg ID: %d\n",i);

        	memcpy(online_ideal_screw_pos+3*i,online_static_screw_pos+3*i,sizeof(double)*3);

            CGait::robot.pLegs[i]->SetPin(online_ideal_screw_pos+3*i);
            CGait::robot.pLegs[i]->GetPee(online_ideal_foot_pos+3*i);
        	//rt_printf("foot_pos before PID, %f %f %f\n",online_ideal_foot_pos[3*i],online_ideal_foot_pos[3*i+1],online_ideal_foot_pos[3*i+2]);

            //just to keep accordance with other cases
            memcpy(online_ideal_screw_pos_before_PID+3*i,online_ideal_screw_pos+3*i,sizeof(double)*3);
            memcpy(online_ideal_foot_pos_before_PID+3*i,online_ideal_foot_pos+3*i,sizeof(double)*3);
        	//rt_printf("foot_pos after PID, %f %f %f\n",online_ideal_foot_pos[3*i],online_ideal_foot_pos[3*i+1],online_ideal_foot_pos[3*i+2]);

        	break;

        default:
        	break;
        }


    	//decelerate at AccMax when getting screwpos limit
    	if (online_ideal_screw_pos[3*i] == 1.091 - SafetyDistance)
    	{
    		online_ideal_screw_pos[3*i]=online_last_ideal_screw_pos[3*i]+online_last_ideal_screw_vel[3*i]*0.001-AccMax/2*0.001*0.001;
    		online_ideal_screw_vel[3*i]=online_last_ideal_screw_vel[3*i]-AccMax*0.001;

    	}
    	else if(online_ideal_screw_pos[3*i] == 0.68 + SafetyDistance)
    	{
    		online_ideal_screw_pos[3*i]=online_last_ideal_screw_pos[3*i]+online_last_ideal_screw_vel[3*i]*0.001+AccMax/2*0.001*0.001;
    		online_ideal_screw_vel[3*i]=online_last_ideal_screw_vel[3*i]+AccMax*0.001;

    	}

    	if (online_ideal_screw_pos[3*i+1] == 1.112 - SafetyDistance)
		{
			online_ideal_screw_pos[3*i+1]=online_last_ideal_screw_pos[3*i+1]+online_last_ideal_screw_vel[3*i+1]*0.001-AccMax/2*0.001*0.001;
    		online_ideal_screw_vel[3*i+1]=online_last_ideal_screw_vel[3*i+1]-AccMax*0.001;

		}
    	else if(online_ideal_screw_pos[3*i+1] == 0.702 + SafetyDistance)
    	{
    		online_ideal_screw_pos[3*i+1]=online_last_ideal_screw_pos[3*i+1]+online_last_ideal_screw_vel[3*i+1]*0.001+AccMax/2*0.001*0.001;
    		online_ideal_screw_vel[3*i+1]=online_last_ideal_screw_vel[3*i+1]+AccMax*0.001;

    	}

    	if (online_ideal_screw_pos[3*i+2] == 1.112 - SafetyDistance)
		{
			online_ideal_screw_pos[3*i+2]=online_last_ideal_screw_pos[3*i+2]+online_last_ideal_screw_vel[3*i+2]*0.001-AccMax/2*0.001*0.001;
    		online_ideal_screw_vel[3*i+2]=online_last_ideal_screw_vel[3*i+2]-AccMax*0.001;

		}
    	else if(online_ideal_screw_pos[3*i+2] == 0.702 - SafetyDistance)
		{

    		online_ideal_screw_pos[3*i+2]=online_last_ideal_screw_pos[3*i+2]+online_last_ideal_screw_vel[3*i+2]*0.001+AccMax/2*0.001*0.001;
    		online_ideal_screw_vel[3*i+2]=online_last_ideal_screw_vel[3*i+2]+AccMax*0.001;

		}

    }

 	memcpy(online_second_last_ideal_screw_pos,online_last_ideal_screw_pos,sizeof(double)*18);
 	memcpy(online_last_ideal_screw_pos,online_ideal_screw_pos,sizeof(double)*18);


 	memcpy(online_last_ideal_screw_vel,online_ideal_screw_vel,sizeof(double)*18);

  /*  rt_printf("online ideal last screw pos\n");
        rt_printf("%f,%f,%f\n",online_last_ideal_screw_pos[0],online_last_ideal_screw_pos[1],online_last_ideal_screw_pos[2]);
        rt_printf("%f,%f,%f\n",online_last_ideal_screw_pos[3],online_last_ideal_screw_pos[4],online_last_ideal_screw_pos[5]);
        rt_printf("%f,%f,%f\n",online_last_ideal_screw_pos[6],online_last_ideal_screw_pos[7],online_last_ideal_screw_pos[8]);
        rt_printf("%f,%f,%f\n",online_last_ideal_screw_pos[9],online_last_ideal_screw_pos[10],online_last_ideal_screw_pos[11]);
        rt_printf("%f,%f,%f\n",online_last_ideal_screw_pos[12],online_last_ideal_screw_pos[13],online_last_ideal_screw_pos[14]);
        rt_printf("%f,%f,%f\n",online_last_ideal_screw_pos[15],online_last_ideal_screw_pos[16],online_last_ideal_screw_pos[17]);
*/
 	  //  double screw_pos_ideal[18];
 	 //   for (int k=0;k<18;k++)
 	  //  {
 	 //   	screw_pos_ideal[k]=online_ideal_screw_pos[k]/350/65536;
 	 //   }
     //   CGait::robot.SetPin(screw_pos_ideal,ep_ROBOT_313);

    //    CGait::robot.GetPee(online_actual_foot_pos);

      //  CGait::robot.SetPee(online_actual_foot_pos,ep_ROBOT_313);

      //  CGait::robot.GetPin(online_screw_pos);
    // rt_printf(" \n");

}


bool CGait::IsGaitFinished()
{
	for(int i=0;i<AXIS_NUMBER;i++)
	{
		if(m_gaitState[i]!=EGaitState::GAIT_STOP)
			return false;
	}
		return true;
};

static std::ifstream fin;
//read file
int CGait::InitGait(Aris::RT_CONTROL::CSysInitParameters& param)
{
    CGait::robot.LoadXML("/usr/Robots/resource/HexapodIII/HexapodIII.xml");

   	 	Trot.LoadRobot();
	    double timemidleg=1;
	    double period=4;
	    double stepsize=0.0;
	    double stepheight=0.04;
	    double alpha=0.65;
		Trot.SetGaitParas(timemidleg,period,stepsize,stepheight,alpha);

		Gait_online_Acc_Length=int((CGait::Trot.m_raiseMidLegsTime+CGait::Trot.m_period*0.5)*1000);
		Gait_online_Cons_Length=int(CGait::Trot.m_period*1000);
		Gait_online_Dec_Length=int((CGait::Trot.m_raiseMidLegsTime+CGait::Trot.m_period*0.5)*1000);

		//printf("acc time %d, cons time %d, dec time %d\n",Gait_online_Acc_Length,Gait_online_Cons_Length,Gait_online_Dec_Length);

	    int Line_Num;
		int ret;

		double temp;
 		std::cout<<"Reading Static Trajectories..."<<std::endl;

		fin.open("../../resource/gait/TL.txt");

		for(int i=0;i<GAIT_TURN_LEN;i++)
		{
			fin>>Line_Num;
			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitTurnLeft[i][j]=-(int)temp;
				//cout<<GaitTurnLeft[i][j]<<endl;

			}
		}


	    fin.close();

	     fin.open("../../resource/gait/TR.txt");

			for(int i=0;i<GAIT_TURN_LEN;i++)
			{
				fin>>Line_Num;


				for(int j=0;j<GAIT_WIDTH;j++)
				{
					fin>>temp;
					GaitTurnRight[i][j]=-(int)temp;

				}
			}


		fin.close();

		fin.open("../../resource/gait/leg_up.txt");

		for(int i=0;i<GAIT_LEGUP_LEN;i++)
		{
			fin>>Line_Num;


			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitLegUp[i][j]=-(int)temp;

			}
		}

		fin.close();

		fin.open("../../resource/gait/start.txt");

		for(int i=0;i<GAIT_HOME2START_LEN;i++)
		{
			fin>>Line_Num;


			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitHome2Start[i][j]=-(int)temp;
			 	//cout<<"gait start "<<GaitHome2Start[i][j]<<endl;

			}

		}

		fin.close();
		/*FOWARD*/
		fin.open("../../resource/gait/acc.txt");
		for(int i=0; i<GAIT_ACC_LEN;i++)
		{
			fin>>Line_Num;
			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitMove_Acc[i][j]=-(int)temp;
			}

		}
		fin.close();


		fin.open("../../resource/gait/cons.txt");
		for(int i=0+GAIT_ACC_LEN; i<GAIT_ACC_LEN+GAIT_CON_LEN;i++)
		{
			fin>>Line_Num;
			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitMove_Cons[i-GAIT_ACC_LEN][j]=-(int)temp;
			}

		}
		fin.close();
		fin.open("../../resource/gait/dec.txt");
		for(int i=0+GAIT_ACC_LEN+GAIT_CON_LEN; i<GAIT_ACC_LEN+GAIT_CON_LEN+GAIT_DEC_LEN;i++)
		{
			fin>>Line_Num;
			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitMove_Dec[i-GAIT_ACC_LEN-GAIT_CON_LEN][j]=-(int)temp;
			}

		}
		fin.close();

		/*backward*/
		fin.open("../../resource/gait/back_acc.txt");
		for(int i=0; i<GAIT_ACC_LEN;i++)
		{
			fin>>Line_Num;
			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitMoveBack_Acc[i][j]=-(int)temp;
			}

		}
		fin.close();


		fin.open("../../resource/gait/back_cons.txt");
		for(int i=0+GAIT_ACC_LEN; i<GAIT_ACC_LEN+GAIT_CON_LEN;i++)
		{
			fin>>Line_Num;
			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitMoveBack_Cons[i-GAIT_ACC_LEN][j]=-(int)temp;
			}

		}
		fin.close();
		fin.open("../../resource/gait/back_dec.txt");
		for(int i=0+GAIT_ACC_LEN+GAIT_CON_LEN; i<GAIT_ACC_LEN+GAIT_CON_LEN+GAIT_DEC_LEN;i++)
		{
			fin>>Line_Num;
			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitMoveBack_Dec[i-GAIT_ACC_LEN-GAIT_CON_LEN][j]=-(int)temp;
			}

		}
		fin.close();

		/*fast FOWARD*/

		fin.open("../../resource/gait/fast_acc.txt");
		for(int i=0; i<GAIT_FAST_ACC_LEN;i++)
		{
			fin>>Line_Num;
			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitFastMove_Acc[i][j]=-(int)temp;
			}
		}
		fin.close();


		fin.open("../../resource/gait/fast_cons.txt");
		for(int i=0+GAIT_FAST_ACC_LEN; i<GAIT_FAST_ACC_LEN+GAIT_FAST_CON_LEN;i++)
		{
			fin>>Line_Num;
			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitFastMove_Cons[i-GAIT_FAST_ACC_LEN][j]=-(int)temp;
			}
		}

		fin.close();
		fin.open("../../resource/gait/fast_dec.txt");
		for(int i=0+GAIT_FAST_ACC_LEN+GAIT_FAST_CON_LEN; i<GAIT_FAST_ACC_LEN+GAIT_FAST_CON_LEN+GAIT_FAST_DEC_LEN;i++)
		{
			fin>>Line_Num;
			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitFastMove_Dec[i-GAIT_FAST_ACC_LEN-GAIT_FAST_CON_LEN][j]=-(int)temp;
			}
		}
		fin.close();
		/*fast backward*/

		fin.open("../../resource/gait/fast_back_acc.txt");
		for(int i=0; i<GAIT_FAST_ACC_LEN;i++)
		{
			fin>>Line_Num;
			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitFastMoveBack_Acc[i][j]=-(int)temp;
			}
		}
		fin.close();


		fin.open("../../resource/gait/fast_back_const.txt");
		for(int i=0+GAIT_FAST_ACC_LEN; i<GAIT_FAST_ACC_LEN+GAIT_FAST_CON_LEN;i++)
		{
			fin>>Line_Num;
			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitFastMoveBack_Cons[i-GAIT_FAST_ACC_LEN][j]=-(int)temp;
			}
		}

		fin.close();
		fin.open("../../resource/gait/fast_back_dec.txt");
		for(int i=0+GAIT_FAST_ACC_LEN+GAIT_FAST_CON_LEN; i<GAIT_FAST_ACC_LEN+GAIT_FAST_CON_LEN+GAIT_FAST_DEC_LEN;i++)
		{
			fin>>Line_Num;
			for(int j=0;j<GAIT_WIDTH;j++)
			{
				fin>>temp;
				GaitFastMoveBack_Dec[i-GAIT_FAST_ACC_LEN-GAIT_FAST_CON_LEN][j]=-(int)temp;
			}
		}
		fin.close();

	//// trot gait
		fin.open("../../resource/gait/trot_three_acc_0.55_2.3_0.18.txt");
			for(int i=0; i<GAIT_TROT_ACC_LEN;i++)
			{
				fin>>Line_Num;
				for(int j=0;j<GAIT_WIDTH;j++)
				{
					fin>>temp;
					GaitTrot_Acc[i][j]=-(int)temp;
				}
			}
			fin.close();


			fin.open("../../resource/gait/trot_three_const_0.55_2.3_0.18.txt");
			for(int i=0+GAIT_TROT_ACC_LEN; i<GAIT_TROT_ACC_LEN+GAIT_TROT_CON_LEN;i++)
			{
				fin>>Line_Num;
				for(int j=0;j<GAIT_WIDTH;j++)
				{
					fin>>temp;
					GaitTrot_Cons[i-GAIT_TROT_ACC_LEN][j]=-(int)temp;
				}
			}

			fin.close();
			fin.open("../../resource/gait/trot_three_dec_0.55_2.3_0.18.txt");
			for(int i=0+GAIT_TROT_ACC_LEN+GAIT_TROT_CON_LEN; i<GAIT_TROT_ACC_LEN+GAIT_TROT_CON_LEN+GAIT_TROT_DEC_LEN;i++)
			{
				fin>>Line_Num;
				for(int j=0;j<GAIT_WIDTH;j++)
				{
					fin>>temp;
					GaitTrot_Dec[i-GAIT_TROT_ACC_LEN-GAIT_TROT_CON_LEN][j]=-(int)temp;
				}
			}
			fin.close();

		return 0;
};
void CGait::IfReadytoSetGait(bool b,int driverID)
{
   CGait::isReadytoSetGait[driverID]=b;
}

int CGait::RunGait(EGAIT* p_gait,Aris::RT_CONTROL::CMachineData& p_data)
{
	//rt_printf("operation mode %d\n",p_data.motorsModes[0]);
    MapFeedbackDataIn(p_data);


    if(p_gait[0]==GAIT_ONLINE)
    {
    	if(p_gait[0]!=m_currentGait[0])
    	{
        	//rt_printf("1. PID index %d\n",1);

    		online_DoPID(0,p_data);

    	}
    	else if(m_gaitCurrentIndex[0]+1<Gait_online_Acc_Length)
		{
        	//rt_printf("1. PID index %d\n",1+m_gaitCurrentIndex[0]);

	    	online_DoPID(1+m_gaitCurrentIndex[0],p_data);
		}
		else
		{
			if(Gait_iter[0]>=1&&IsConsFinished[0]==false)
			{
	        	//rt_printf("2. PID index %d\n",(m_gaitCurrentIndex[0]+1-Gait_online_Acc_Length)%Gait_online_Cons_Length+Gait_online_Acc_Length);

				online_DoPID((m_gaitCurrentIndex[0]+1-Gait_online_Acc_Length)%Gait_online_Cons_Length+Gait_online_Acc_Length,p_data);

			}
			else
			{
	        	//rt_printf("3. PID index %d\n",m_gaitCurrentIndex[0]+1-Gait_online_Cons_Length*(Gait_iter_count[0]-1));

		    	online_DoPID(m_gaitCurrentIndex[0]+1-Gait_online_Cons_Length*(Gait_iter_count[0]-1),p_data);

			}
		}

    	//rt_printf("is cons finished %d, current index %d\n",IsConsFinished[0],m_gaitCurrentIndex[0]);
    }
    if(p_gait[0]==GAIT_TOSTANDSTILL)
    {
    	double screw_pos[18];

    	if(p_gait[0]!=m_currentGait[0])
    	{
			 online_ToStandstill(0,online_ideal_screw_pos,screw_pos);
    	}
    	else
    	{
			 online_ToStandstill(m_gaitCurrentIndex[0]+1,online_ideal_screw_pos,screw_pos);
    	}
    }

    //IF ONLINE-->PID-->screw[18]
  /*  if(p_gait[0]==GAIT_ONLINE)
    {
    	online_DoPID(1+m_gaitCurrentIndex[0],p_data);
    }*/

	for(int i=0;i<AXIS_NUMBER;i++)
	{
		if(isReadytoSetGait[i]==true)
		{
			int motorID =MapPhyToAbs[i];
			/* if(i==0)
				{
					rt_printf("feedbackdata all:%d\n",m_feedbackDataMapped[motorID].Position);
				}*/
			switch(p_gait[i])
			{
			case GAIT_TOSTANDSTILL:

				double screw_pos[18];

				if(p_gait[i]!=m_currentGait[i])
				{
					rt_printf("driver %d: GAIT_TOSTANDSTILL begin\n",i);
					m_gaitState[i]=EGaitState::GAIT_RUN;
					m_currentGait[i]=p_gait[i];
					m_gaitStartTime[i]=p_data.time;
					m_gaitCurrentIndex[i]=0;

 					//rt_printf("online ideal screw pos before:%f\n",online_ideal_screw_pos[motorID]);
 				    m_commandDataMapped[motorID].Position=int(350*65536*screw_pos[motorID]);
				}
				else
				{
					m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);


				    m_commandDataMapped[motorID].Position=int(350*65536*screw_pos[motorID]);


					if(m_gaitCurrentIndex[i]==6000-1)
					{

						rt_printf("driver %d:GAIT_HOME2START will transfer to GAIT_STANDSTILL...\n",i);
						p_gait[i]=GAIT_STANDSTILL;
						//online_ToStandstill(m_gaitCurrentIndex[i],online_ideal_screw_pos,screw_pos);

						m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
						m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
						m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;
						m_gaitState[i]=EGaitState::GAIT_STOP;
					}

				}
				break;

			 case GAIT_ONLINE:

 				  				 	if(p_gait[i]!=m_currentGait[i])
				  					{
				  						rt_printf("driver %d: GAIT_MOVE begin\n",i);
				  						m_gaitState[i]=EGaitState::GAIT_RUN;
				  						m_currentGait[i]=p_gait[i];
				  						m_gaitStartTime[i]=p_data.time;
				  						//rt_printf("time is: %d \n", p_data.time);
				  						m_commandDataMapped[motorID].Position=int(350*65536*online_ideal_screw_pos[motorID]);
				  						//m_gaitCurrentIndexForPID[i]=1;
				  						m_gaitCurrentIndex[i]=0;
				  						rt_printf("driver %d:Begin Online Trotting...\n",i);
				  					}
				  					else
				  					{
				  						m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);
				  						//m_gaitCurrentIndexForPID[i] = m_gaitCurrentIndex[i]+1;
  				  						//rt_printf("index%d and Gait_iter_count%d and Gait_iter%d\n",m_gaitCurrentIndex,Gait_iter_count,Gait_iter);

				  						//rt_printf("MOVE order:%d\n",GaitMove_Acc[m_gaitCurrentIndex][0]);
				  						if(m_gaitCurrentIndex[i]<Gait_online_Acc_Length)
				  						{
				  							m_commandDataMapped[motorID].Position=int(350*65536*online_ideal_screw_pos[motorID]);
				  						}

				  						else //if(m_gaitCurrentIndex>=GAIT_ACC_LEN)
				  						{
				  							if(Gait_iter[i]>=1&&IsConsFinished[i]==false)
				  							{


				  								m_commandDataMapped[motorID].Position=int(350*65536*online_ideal_screw_pos[motorID]);

				  								if(((m_gaitCurrentIndex[i]-Gait_online_Acc_Length)%Gait_online_Cons_Length)==0)
				  								{
				  									Gait_iter_count[i]++;
				  								}

				  								if(((m_gaitCurrentIndex[i]-Gait_online_Acc_Length)%Gait_online_Cons_Length)==Gait_online_Cons_Length-1)
				  								{
				  									Gait_iter[i]--;
				  								}
				  							}
				  							else
				  							{
				  								IsConsFinished[i]=true;
				  							//	rt_printf("GAIT_MOVE just finished CONS stage...\n");
				  							//  rt_printf("GAIT_MOVE will be deccelarating...\n");

				  							    	m_commandDataMapped[motorID].Position=int(350*65536*online_ideal_screw_pos[motorID]);
				  							}


				  							if(m_gaitCurrentIndex[i]==Gait_online_Acc_Length+Gait_online_Cons_Length*Gait_iter_count[i]+Gait_online_Dec_Length-1)
				  							{
				  								rt_printf("driver %d: GAIT_MOVE will transfer to GAIT_STANDSTILL...\n",i);

				  								p_gait[i]=GAIT_STANDSTILL;


				  									m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
				  									m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
				  									m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

				  									 //only in this cycle, out side get true from IsGaitFinished()
				  									 m_gaitState[i]=EGaitState::GAIT_STOP;
				  									 Gait_iter_count[i]=0;
				  									 Gait_iter[i]=1;
				  									 IsConsFinished[i]=false;
				  							}
				  						}
				  					}


 								    if(i==0)
 								    {
 									    rt_printf("online ideal screw pos:%f\n",online_ideal_screw_pos[motorID]);
 									   // rt_printf("screw pos:%f\n",screw_pos[motorID]);

 								    }

				  					break;

			/* 	if(i==0)
				{
					if(p_gait[i]!=m_currentGait[i])
					{
						online_DoPID(0,p_data);
					}
				}

				if(p_gait[i]!=m_currentGait[i])
				{
				//	rt_printf("driver %d: GAIT_ONLINE begin\n",i);
					m_gaitState[i]=EGaitState::GAIT_RUN;
					m_currentGait[i]=p_gait[i];
					m_gaitStartTime[i]=p_data.time;

					m_gaitCurrentIndex[i]=0;
				    m_commandDataMapped[motorID].Position=(int)(350*65536*online_ideal_screw_pos[motorID]);


				}
				else
				{

					m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);
					if(i==0)
						rt_printf("m_currentIndex %d\n",m_gaitCurrentIndex[i]);

				    m_commandDataMapped[motorID].Position=(int)(350*65536*online_ideal_screw_pos[motorID]);


					if(m_gaitCurrentIndex[i]==Trot.m_gaitLength-1)
					{
						rt_printf("driver %d:GAIT_ONLINE will transfer to GAIT_STANDSTILL...\n",i);
						p_gait[i]=GAIT_STANDSTILL;

						m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
						m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
						m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;
						m_gaitState[i]=EGaitState::GAIT_STOP;
					}
				}

				break;*/


			case GAIT_NULL:
 				p_gait[i]=GAIT_STANDSTILL;
				CGait::m_gaitState[i]=EGaitState::GAIT_STOP;

					m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
					m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
					m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

					m_commandDataMapped[motorID].Position=m_standStillData[motorID].Position;
					m_commandDataMapped[motorID].Velocity=m_standStillData[motorID].Velocity;
					m_commandDataMapped[motorID].Torque=m_standStillData[motorID].Torque;

				rt_printf("driver %d: GAIT_NONE will transfer to GAIT_STANDSTILL...\n",i);

				break;
			case GAIT_HOME:

					m_commandDataMapped[motorID].Position=m_feedbackDataMapped[motorID].Position;
					m_commandDataMapped[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
					m_commandDataMapped[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

				    if(p_data.isMotorHomed[i]==true)
					{
						m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
						m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
						m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

						p_gait[i]=GAIT_STANDSTILL;
						rt_printf("driver %d: HOMED\n",i);

					}

				break;

			case GAIT_STANDSTILL:
				if(p_gait[i]!=m_currentGait[i])
				{
					rt_printf("driver %d:   GAIT_STANDSTILL begins\n",i);
 					m_currentGait[i]=p_gait[i];
					m_gaitStartTime[i]=p_data.time;

						m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
						m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
						m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

						m_commandDataMapped[motorID].Position=m_standStillData[motorID].Position;
						m_commandDataMapped[motorID].Velocity=m_standStillData[motorID].Velocity;
						m_commandDataMapped[motorID].Torque=m_standStillData[motorID].Torque;


				}
				else
				{

						m_commandDataMapped[motorID].Position=m_standStillData[motorID].Position;
						m_commandDataMapped[motorID].Velocity=m_standStillData[motorID].Velocity;
						m_commandDataMapped[motorID].Torque=m_standStillData[motorID].Torque;
				}
				break;
			case GAIT_HOME2START:

				if(p_gait[i]!=m_currentGait[i])
				{
					rt_printf("driver %d: GAIT_HOME2START begin\n",i);
					m_gaitState[i]=EGaitState::GAIT_RUN;
					m_currentGait[i]=p_gait[i];
					m_gaitStartTime[i]=p_data.time;
				    m_commandDataMapped[motorID].Position=GaitHome2Start[0][motorID];

				}
				else
				{
					m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);

			    	m_commandDataMapped[motorID].Position=GaitHome2Start[m_gaitCurrentIndex[i]][motorID];


					if(m_gaitCurrentIndex[i]==GAIT_HOME2START_LEN-1)
					{
						rt_printf("driver %d:GAIT_HOME2START will transfer to GAIT_STANDSTILL...\n",i);
						p_gait[i]=GAIT_STANDSTILL;

						m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
						m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
						m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;
						m_gaitState[i]=EGaitState::GAIT_STOP;
					}

				}
				break;
			case GAIT_LEGUP:

							if(p_gait[i]!=m_currentGait[i])
							{
								rt_printf("driver %d: GAIT_LEGUP begin\n",i);
								m_gaitState[i]=EGaitState::GAIT_RUN;
								m_currentGait[i]=p_gait[i];
								m_gaitStartTime[i]=p_data.time;
							    m_commandDataMapped[motorID].Position=GaitLegUp[0][motorID];

							}
							else
							{
								m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);

						    	m_commandDataMapped[motorID].Position=GaitLegUp[m_gaitCurrentIndex[i]][motorID];


								if(m_gaitCurrentIndex[i]==GAIT_LEGUP_LEN-1)
								{
									rt_printf("driver %d:GAIT_LEGUP will transfer to GAIT_STANDSTILL...\n",i);
									p_gait[i]=GAIT_STANDSTILL;

									m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
									m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
									m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;
									m_gaitState[i]=EGaitState::GAIT_STOP;
								}

							}
							break;
			case GAIT_TURN_LEFT:

							if(p_gait[i]!=m_currentGait[i])
							{
								rt_printf("driver %d: GAIT_TURNLEFT begin\n",i);
								m_gaitState[i]=EGaitState::GAIT_RUN;
								m_currentGait[i]=p_gait[i];
								m_gaitStartTime[i]=p_data.time;
							    m_commandDataMapped[motorID].Position=GaitTurnLeft[0][motorID];

							}
							else
							{
								m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);

						    	m_commandDataMapped[motorID].Position=GaitTurnLeft[m_gaitCurrentIndex[i]][motorID];


								if(m_gaitCurrentIndex[i]==GAIT_TURN_LEN-1)
								{
									rt_printf("driver %d:GAIT_TURNLEFT will transfer to GAIT_STANDSTILL...\n",i);
									p_gait[i]=GAIT_STANDSTILL;

									m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
									m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
									m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;
									m_gaitState[i]=EGaitState::GAIT_STOP;
								}

							}
							break;
			case GAIT_TURN_RIGHT:

							if(p_gait[i]!=m_currentGait[i])
							{
								rt_printf("driver %d: GAIT_TURNRIGHT begin\n",i);
								m_gaitState[i]=EGaitState::GAIT_RUN;
								m_currentGait[i]=p_gait[i];
								m_gaitStartTime[i]=p_data.time;
							    m_commandDataMapped[motorID].Position=GaitTurnRight[0][motorID];

							}
							else
							{
								m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);

						    	m_commandDataMapped[motorID].Position=GaitTurnRight[m_gaitCurrentIndex[i]][motorID];


								if(m_gaitCurrentIndex[i]==GAIT_TURN_LEN-1)
								{
									rt_printf("driver %d:GAIT_TURNRIGHT will transfer to GAIT_STANDSTILL...\n",i);
									p_gait[i]=GAIT_STANDSTILL;

									m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
									m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
									m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;
									m_gaitState[i]=EGaitState::GAIT_STOP;
								}

							}
							break;

			case GAIT_MOVE:

						if(p_gait[i]!=m_currentGait[i])
						{
							rt_printf("driver %d: GAIT_MOVE begin\n",i);
							m_gaitState[i]=EGaitState::GAIT_RUN;
							m_currentGait[i]=p_gait[i];
							m_gaitStartTime[i]=p_data.time;
							m_commandDataMapped[motorID].Position=GaitMove_Acc[0][motorID];

							rt_printf("driver %d:Begin Accelerating foward...\n",i);
						}
						else
						{

							m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);
							//rt_printf("index%d and Gait_iter_count%d and Gait_iter%d\n",m_gaitCurrentIndex,Gait_iter_count,Gait_iter);



							//rt_printf("MOVE order:%d\n",GaitMove_Acc[m_gaitCurrentIndex][0]);
							if(m_gaitCurrentIndex[i]<GAIT_ACC_LEN)
							{
								m_commandDataMapped[motorID].Position=GaitMove_Acc[m_gaitCurrentIndex[i]][motorID];
							}

							else //if(m_gaitCurrentIndex>=GAIT_ACC_LEN)
							{
								if(Gait_iter[i]>=1&&IsConsFinished[i]==false)
								{


									m_commandDataMapped[motorID].Position=GaitMove_Cons[(m_gaitCurrentIndex[i]-GAIT_ACC_LEN)%GAIT_CON_LEN][motorID];

									if(((m_gaitCurrentIndex[i]-GAIT_ACC_LEN)%GAIT_CON_LEN)==0)
									{

										Gait_iter_count[i]++;

									}
									if(((m_gaitCurrentIndex[i]-GAIT_ACC_LEN)%GAIT_CON_LEN)==GAIT_CON_LEN-1)
									{

										Gait_iter[i]--;

									}
								}
								else
								{
									IsConsFinished[i]=true;
								//	rt_printf("GAIT_MOVE just finished CONS stage...\n");
								//  rt_printf("GAIT_MOVE will be deccelarating...\n");

								    	m_commandDataMapped[motorID].Position=GaitMove_Dec[(m_gaitCurrentIndex[i]-GAIT_ACC_LEN-GAIT_CON_LEN*Gait_iter_count[i])][motorID];

								}


								if(m_gaitCurrentIndex[i]==GAIT_ACC_LEN+GAIT_CON_LEN*Gait_iter_count[i]+GAIT_DEC_LEN-1)
								{
									rt_printf("driver %d: GAIT_MOVE will transfer to GAIT_STANDSTILL...\n",i);

									p_gait[i]=GAIT_STANDSTILL;


										m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
										m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
										m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

										 //only in this cycle, out side get true from IsGaitFinished()
										 m_gaitState[i]=EGaitState::GAIT_STOP;
										 Gait_iter_count[i]=0;
										 Gait_iter[i]=1;
										 IsConsFinished[i]=false;

								}
							}
						}
						break;
			case GAIT_MOVE_BACK:

						if(p_gait[i]!=m_currentGait[i])
						{
							rt_printf("driver %d: GAIT_MOVE_BACK begin\n",i);
							m_gaitState[i]=EGaitState::GAIT_RUN;
							m_currentGait[i]=p_gait[i];
							m_gaitStartTime[i]=p_data.time;
							m_commandDataMapped[motorID].Position=GaitMoveBack_Acc[0][motorID];

							rt_printf("driver %d:Begin Accelerating backward...\n",i);
						}
						else
						{

							m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);
							//rt_printf("index%d and Gait_iter_count%d and Gait_iter%d\n",m_gaitCurrentIndex,Gait_iter_count,Gait_iter);

							//rt_printf("MOVE order:%d\n",GaitMove_Acc[m_gaitCurrentIndex][0]);
							if(m_gaitCurrentIndex[i]<GAIT_ACC_LEN)
							{
								m_commandDataMapped[motorID].Position=GaitMoveBack_Acc[m_gaitCurrentIndex[i]][motorID];
							}

							else //if(m_gaitCurrentIndex>=GAIT_ACC_LEN)
							{
								if(Gait_iter[i]>=1&&IsConsFinished[i]==false)
								{


									m_commandDataMapped[motorID].Position=GaitMoveBack_Cons[(m_gaitCurrentIndex[i]-GAIT_ACC_LEN)%GAIT_CON_LEN][motorID];

									if(((m_gaitCurrentIndex[i]-GAIT_ACC_LEN)%GAIT_CON_LEN)==0)
									{

										Gait_iter_count[i]++;

									}
									if(((m_gaitCurrentIndex[i]-GAIT_ACC_LEN)%GAIT_CON_LEN)==GAIT_CON_LEN-1)
									{

										Gait_iter[i]--;

									}
								}
								else
								{
									IsConsFinished[i]=true;
								//	rt_printf("GAIT_MOVE just finished CONS stage...\n");
								//  rt_printf("GAIT_MOVE will be deccelarating...\n");

								    	m_commandDataMapped[motorID].Position=GaitMoveBack_Dec[(m_gaitCurrentIndex[i]-GAIT_ACC_LEN-GAIT_CON_LEN*Gait_iter_count[i])][motorID];

								}


								if(m_gaitCurrentIndex[i]==GAIT_ACC_LEN+GAIT_CON_LEN*Gait_iter_count[i]+GAIT_DEC_LEN-1)
								{
									rt_printf("driver %d: GAIT_MOVE_BACK will transfer to GAIT_STANDSTILL...\n",i);

									p_gait[i]=GAIT_STANDSTILL;


										m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
										m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
										m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

										 //only in this cycle, out side get true from IsGaitFinished()
										 m_gaitState[i]=EGaitState::GAIT_STOP;
										 Gait_iter_count[i]=0;
										 Gait_iter[i]=1;
										 IsConsFinished[i]=false;

								}
							}
						}
						break;

			case GAIT_FAST_MOVE:

						if(p_gait[i]!=m_currentGait[i])
						{
							rt_printf("driver %d: GAIT_FAST_MOVE begin\n",i);
							m_gaitState[i]=EGaitState::GAIT_RUN;
							m_currentGait[i]=p_gait[i];
							m_gaitStartTime[i]=p_data.time;
							m_commandDataMapped[motorID].Position=GaitFastMove_Acc[0][motorID];

							rt_printf("driver %d:Begin Accelerating foward...\n",i);
						}
						else
						{

							m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);
							//rt_printf("index%d and Gait_iter_count%d and Gait_iter%d\n",m_gaitCurrentIndex,Gait_iter_count,Gait_iter);



							//rt_printf("MOVE order:%d\n",GaitMove_Acc[m_gaitCurrentIndex][0]);
							if(m_gaitCurrentIndex[i]<GAIT_FAST_ACC_LEN)
							{
								m_commandDataMapped[motorID].Position=GaitFastMove_Acc[m_gaitCurrentIndex[i]][motorID];
							}

							else //if(m_gaitCurrentIndex>=GAIT_ACC_LEN)
							{
								if(Gait_iter[i]>=1&&IsConsFinished[i]==false)
								{


									m_commandDataMapped[motorID].Position=GaitFastMove_Cons[(m_gaitCurrentIndex[i]-GAIT_FAST_ACC_LEN)%GAIT_FAST_CON_LEN][motorID];

									if(((m_gaitCurrentIndex[i]-GAIT_FAST_ACC_LEN)%GAIT_FAST_CON_LEN)==0)
									{

										Gait_iter_count[i]++;

									}
									if(((m_gaitCurrentIndex[i]-GAIT_FAST_ACC_LEN)%GAIT_FAST_CON_LEN)==GAIT_FAST_CON_LEN-1)
									{

										Gait_iter[i]--;

									}
								}
								else
								{
									IsConsFinished[i]=true;
								//	rt_printf("GAIT_MOVE just finished CONS stage...\n");
								//  rt_printf("GAIT_MOVE will be deccelarating...\n");

								    	m_commandDataMapped[motorID].Position=GaitFastMove_Dec[(m_gaitCurrentIndex[i]-GAIT_FAST_ACC_LEN-GAIT_FAST_CON_LEN*Gait_iter_count[i])][motorID];

								}


								if(m_gaitCurrentIndex[i]==GAIT_FAST_ACC_LEN+GAIT_FAST_CON_LEN*Gait_iter_count[i]+GAIT_FAST_DEC_LEN-1)
								{
									rt_printf("driver %d: GAIT_FAST_MOVE will transfer to GAIT_STANDSTILL...\n",i);

									p_gait[i]=GAIT_STANDSTILL;


										m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
										m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
										m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

										 //only in this cycle, out side get true from IsGaitFinished()
										 m_gaitState[i]=EGaitState::GAIT_STOP;
										 Gait_iter_count[i]=0;
										 Gait_iter[i]=1;
										 IsConsFinished[i]=false;

								}
							}
						}
						break;

			case GAIT_FAST_MOVE_BACK:

						if(p_gait[i]!=m_currentGait[i])
						{
							rt_printf("driver %d: GAIT_FAST_MOVE_BACK begin\n",i);
							m_gaitState[i]=EGaitState::GAIT_RUN;
							m_currentGait[i]=p_gait[i];
							m_gaitStartTime[i]=p_data.time;
							m_commandDataMapped[motorID].Position=GaitFastMoveBack_Acc[0][motorID];

							rt_printf("driver %d:Begin Accelerating backward...\n",i);
						}
						else
						{

							m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);
							//rt_printf("index%d and Gait_iter_count%d and Gait_iter%d\n",m_gaitCurrentIndex,Gait_iter_count,Gait_iter);



							//rt_printf("MOVE order:%d\n",GaitMove_Acc[m_gaitCurrentIndex][0]);
							if(m_gaitCurrentIndex[i]<GAIT_FAST_ACC_LEN)
							{
								m_commandDataMapped[motorID].Position=GaitFastMoveBack_Acc[m_gaitCurrentIndex[i]][motorID];
							}

							else //if(m_gaitCurrentIndex>=GAIT_ACC_LEN)
							{
								if(Gait_iter[i]>=1&&IsConsFinished[i]==false)
								{


									m_commandDataMapped[motorID].Position=GaitFastMoveBack_Cons[(m_gaitCurrentIndex[i]-GAIT_FAST_ACC_LEN)%GAIT_FAST_CON_LEN][motorID];

									if(((m_gaitCurrentIndex[i]-GAIT_FAST_ACC_LEN)%GAIT_FAST_CON_LEN)==0)
									{

										Gait_iter_count[i]++;

									}
									if(((m_gaitCurrentIndex[i]-GAIT_FAST_ACC_LEN)%GAIT_FAST_CON_LEN)==GAIT_FAST_CON_LEN-1)
									{

										Gait_iter[i]--;

									}
								}
								else
								{
									IsConsFinished[i]=true;
								//	rt_printf("GAIT_MOVE just finished CONS stage...\n");
								//  rt_printf("GAIT_MOVE will be deccelarating...\n");

								    	m_commandDataMapped[motorID].Position=GaitFastMoveBack_Dec[(m_gaitCurrentIndex[i]-GAIT_FAST_ACC_LEN-GAIT_FAST_CON_LEN*Gait_iter_count[i])][motorID];

								}


								if(m_gaitCurrentIndex[i]==GAIT_FAST_ACC_LEN+GAIT_FAST_CON_LEN*Gait_iter_count[i]+GAIT_FAST_DEC_LEN-1)
								{
									rt_printf("driver %d: GAIT_FAST_MOVE_BACK will transfer to GAIT_STANDSTILL...\n",i);

									p_gait[i]=GAIT_STANDSTILL;


										m_standStillData[motorID].Position=m_feedbackDataMapped[motorID].Position;
										m_standStillData[motorID].Velocity=m_feedbackDataMapped[motorID].Velocity;
										m_standStillData[motorID].Torque=m_feedbackDataMapped[motorID].Torque;

										 //only in this cycle, out side get true from IsGaitFinished()
										 m_gaitState[i]=EGaitState::GAIT_STOP;
										 Gait_iter_count[i]=0;
										 Gait_iter[i]=1;
										 IsConsFinished[i]=false;

								}
							}
						}
						break;


			default:
				p_gait[i]=GAIT_STANDSTILL;
				rt_printf("enter the default\n");

				break;
			}

		}
	}
	MapCommandDataOut(p_data);
	//rt_printf("command data pos%d\n",p_data.commandData[0].Position);

	return 0;
};

void CGait::online_trot_TargetXZ(double *IMU_angleVel,double H_robot,int LegID,double *XandZ)
{
	double VelX=-IMU_angleVel[1]*H_robot;
	double VelZ=-IMU_angleVel[0]*H_robot;
	double alpha=atan(0.3/0.65);
	double beta;
	double Vel_dump;
	switch(LegID)
	{
	case 0:
	case 5:
		Vel_dump=VelX*cos(alpha)-VelZ*sin(alpha);
		beta=acos(H_robot/(H_robot+Vel_dump*Vel_dump/2/9.8));
		XandZ[0]=H_robot*tan(beta)*cos(alpha);
		XandZ[1]=-H_robot*tan(beta)*sin(alpha);
		break;
	case 2:
	case 3:
		Vel_dump=VelX*cos(alpha)+VelZ*sin(alpha);
		beta=acos(H_robot/(H_robot+Vel_dump*Vel_dump/2/9.8));
		XandZ[0]=H_robot*tan(beta)*cos(alpha);
		XandZ[1]=H_robot*tan(beta)*sin(alpha);
		break;
	case 1:
	case 4:
	default:
		break;
	}
	if (Vel_dump<0)
	{
		XandZ[0]=-XandZ[0];
		XandZ[1]=-XandZ[1];
	}
}

/*
void CGait::online_ToStandstill(int N,double* initial_screw_pos,double* screw_input)
{
	int Nperiod=3000;

	double target_screw_pos[18]=
	{
      19017915/350/65536,18909560/350/65536,19818639/350/65536,
	  18592790/350/65536,19069060/350/65536,19069060/350/65536,
	  19031958/350/65536,19832453/350/65536,18912891/350/65536,
	  19017915/350/65536,19818639/350/65536,18909560/350/65536,
	  18592790/350/65536,19069060/350/65536,19069060/350/65536,
	  19031958/350/65536,18912891/350/65536,19832453/350/65536
	};

	//function: (target-initial)/2*cos(i/N*2*PI-PI)++(tartget+initial)/2;

	for(int i=0;i<18;i++)
	{
		if(MapPhyToAbs[i]%3==1)
			if(i<Nperiod/2)//???not i, should be j???
			{
				screw_input[i]=(-0.8-initial_screw_pos[i])/2*cos(i/N*4*PI-PI)+(-0.8+initial_screw_pos[i])/2;

			}
			else
			{
				screw_input[i]=(target_pos[MapPhyToAbs[i]]+0.8)/2*cos((i-N/2)/N*4*PI-PI)+(target_pos[MapPhyToAbs[i]]-0.8)/2;

			}

		else
			screw_input[i]=(target_pos[MapPhyToAbs[i]]-initial_screw_pos[i])/2*cos(i/N*2*PI-PI)+(target_pos[MapPhyToAbs[i]]+initial_screw_pos[i])/2;
			//screw_input[i]=-(target_pos[MapPhyToAbs[i]]-initial_screw_pos[i])/2*cos(i/Nperiod*PI)+(target_pos[MapPhyToAbs[i]]+initial_screw_pos[i])/2;

	}

}
*/

void CGait::online_ToStandstill(int N,double* initial_screw_pos,double* screw_input)
{
	double Nperiod=6000;

 	double target_body_pos[6]={0,0,0,0,0,0};
	double target_foot_pos1[18]=
		{
				-0.3 , -0.8, -0.65,
				-0.45, -0.8,  0   ,
				-0.3 , -0.8,  0.65,
				 0.3 , -0.8, -0.65,
				 0.45, -0.8,  0   ,
				 0.3 , -0.8,  0.65
		};
	double target_foot_pos2[18]=
		{
				-0.3 , -0.85, -0.65,
				-0.45, -0.85,  0   ,
				-0.3 , -0.85,  0.65,
				 0.3 , -0.85, -0.65,
				 0.45, -0.85,  0   ,
				 0.3 , -0.85,  0.65
		};

	double target_screw_pos1[18];
	double target_screw_pos2[18];


    CGait::robot.SetPee(target_foot_pos1,target_body_pos);
    CGait::robot.GetPin(target_screw_pos1);
    CGait::robot.SetPee(target_foot_pos2,target_body_pos);
    CGait::robot.GetPin(target_screw_pos2);

	if (N<Nperiod/4)
	{
 		for (int j=0;j<3;j++)
		{
			for(int i=0;i<3;i++)//3 branch of a leg
			{
				screw_input[6*j+i]=(initial_screw_pos[6*j+i]+target_screw_pos1[6*j+i])/2+(-target_screw_pos1[6*j+i]+initial_screw_pos[6*j+i])/2*cos(N/(Nperiod/4)*PI);
			    screw_input[6*j+i+3]=initial_screw_pos[6*j+i+3];

			}
		}
	}

	if (N>=Nperiod/4&&N<Nperiod/2)
	{

		for (int j=0;j<3;j++) // 2,4,6 leg
		{
			for(int i=0;i<3;i++)//3 branch of a leg
			{
				screw_input[6*j+i]=(target_screw_pos1[6*j+i]+target_screw_pos2[6*j+i])/2+(-target_screw_pos2[6*j+i]+target_screw_pos1[6*j+i])/2*cos((N-Nperiod/4)/(Nperiod/4)*PI);
			    screw_input[6*j+i+3]=initial_screw_pos[6*j+i+3];
 			}
		}
	}
	if (N>=Nperiod/2&&N<3*Nperiod/4)
	{

		for (int j=0;j<3;j++)
		{
			for(int i=0;i<3;i++)//3 branch of a leg
			{
			    screw_input[6*j+i]=target_screw_pos2[6*j+i];
				screw_input[6*j+i+3]=(initial_screw_pos[6*j+i+3]+target_screw_pos1[6*j+i+3])/2+(-target_screw_pos1[6*j+i+3]+initial_screw_pos[6*j+i+3])/2*cos((N-Nperiod/2)/(Nperiod/4)*PI);

			}
		}
	}
	if (N>=3*Nperiod/4&&N<Nperiod)
	{

		for (int j=0;j<3;j++)
		{
			for(int i=0;i<3;i++)//3 branch of a leg
			{
			    screw_input[6*j+i]=target_screw_pos2[6*j+i];
				screw_input[6*j+i+3]=(target_screw_pos1[6*j+i+3]+target_screw_pos2[6*j+i+3])/2+(-target_screw_pos2[6*j+i+3]+target_screw_pos1[6*j+i+3])/2*cos((N-3*Nperiod/4)/(Nperiod/4)*PI);

  			}
		}
	}

	if(N==int(Nperiod-1))
	{
 		memcpy(online_ideal_screw_pos,screw_input,sizeof(double)*18);
	}


}

