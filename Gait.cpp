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


double CGait::online_angle[3];
double CGait::online_angleVel[3];
double CGait::online_linearAcc[3];
 ROBOT CGait::robot;


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


static int screw_init_pos[AXIS_NUMBER];
static int screw_command_pos[AXIS_NUMBER];

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



    if(p_gait[0]==GAIT_TOSTANDSTILL)
    {

    	if(p_gait[0]!=m_currentGait[0])
    	{


    		for(int i=0;i<AXIS_NUMBER;i++)
    		{
    			memcpy(&(screw_init_pos[i]),&(m_feedbackDataMapped[i].Position),sizeof(int));
     		}

			 online_ToStandstill(0,screw_init_pos,screw_command_pos);
    	}
    	else
    	{
			 online_ToStandstill(m_gaitCurrentIndex[0]+1,screw_init_pos,screw_command_pos);

    	}
    }


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


				if(p_gait[i]!=m_currentGait[i])
				{
					rt_printf("driver %d: GAIT_TOSTANDSTILL begin\n",i);
					m_gaitState[i]=EGaitState::GAIT_RUN;
					m_currentGait[i]=p_gait[i];
					m_gaitStartTime[i]=p_data.time;
					m_gaitCurrentIndex[i]=0;

 					//rt_printf("online ideal screw pos before:%f\n",online_ideal_screw_pos[motorID]);
 				    m_commandDataMapped[motorID].Position= screw_command_pos[motorID];
				}
				else
				{
					m_gaitCurrentIndex[i]=(int)(p_data.time-m_gaitStartTime[i]);


				    m_commandDataMapped[motorID].Position= screw_command_pos[motorID];


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

/*void CGait::online_trot_TargetXZ(double *IMU_angleVel,double H_robot,int LegID,double *XandZ)
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
}*/



void CGait::online_ToStandstill(int N,int* initial_screw,int* screw)
{
	static double initial_screw_pos[AXIS_NUMBER];
	static double screw_input[AXIS_NUMBER];


	for(int i=0;i<AXIS_NUMBER;i++)
	{
		initial_screw_pos[i]=  (double)(initial_screw[i])/ 65536/350 ;
	}

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


	for(int i=0;i<AXIS_NUMBER;i++)
	{
		screw[i]=(int)(screw_input[i]*65536*350);

	}


}

