/*
 * Gait.h
 *
 *  Created on: Nov 28, 2014
 *      Author: hex
 */
#ifndef GAIT_H_
#define GAIT_H_
#include "Aris_Control.h"
#include "Aris_ControlData.h"
#include "TrotGait.h"

#include <string>
using namespace std;

#define GAIT_WIDTH 18
//acc 2001 con 4000 dec2000
#define GAIT_ACC_LEN 2001
#define GAIT_CON_LEN 4000
#define GAIT_DEC_LEN 2000

#define GAIT_FAST_ACC_LEN 1850
#define GAIT_FAST_CON_LEN 3700
#define GAIT_FAST_DEC_LEN 1850
//2.3s
#define GAIT_TROT_ACC_LEN 3150
#define GAIT_TROT_CON_LEN 2300
#define GAIT_TROT_DEC_LEN 3150


#define GAIT_MOVE_LEN 8001
#define GAIT_MOVEBACK_LEN 8001

#define GAIT_FAST_MOVE_LEN 7400
#define GAIT_FAST_MOVE_BACK_LEN 7400

#define GAIT_HOME2START_LEN 4001
#define GAIT_LEGUP_LEN 12201
#define GAIT_TURN_LEN 6001

 //for test
//#define GAIT_HOME2START_LEN 3700
//
#define MOTOR_NUM 18

#include <mutex>


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
enum EGaitState
{
	NONE,
	GAIT_START,
	GAIT_RUN,
	GAIT_STOP,
};

enum EGAIT
{
	GAIT_NULL=0,
	GAIT_STANDSTILL=1,
	GAIT_HOME2START=2,
	GAIT_MOVE=3,
	GAIT_MOVE_BACK=4,
    GAIT_FAST_MOVE=5,
	GAIT_FAST_MOVE_BACK=6,
	GAIT_TROT=7,
	GAIT_LEGUP=8,
	GAIT_TURN_LEFT=9,
	GAIT_TURN_RIGHT=10,
	GAIT_HOME=11,
	GAIT_ONLINE=12,
	GAIT_TOSTANDSTILL=13
};


class CGait
{
public:
	CGait();
	~CGait();
	//read txt to array
	static int InitGait(Aris::RT_CONTROL::CSysInitParameters& param);
 	static int RunGait(EGAIT* p_gait,Aris::RT_CONTROL::CMachineData& p_data);
 	static bool IsGaitFinished();
	static bool IsHomeStarted[AXIS_NUMBER];
 	static bool IsConsFinished[AXIS_NUMBER];
	static int Gait_iter[AXIS_NUMBER];
	static int Gait_iter_count[AXIS_NUMBER];
    static void  IfReadytoSetGait(bool b, int driverID);
	static EGaitState m_gaitState[AXIS_NUMBER];
	static CTrotGait Trot;

	static void online_ToStandstill(int N,int* initial_screw,int* screw_command);
	static double online_angle[3];
	static double online_angleVel[3];
	static double online_linearAcc[3];


	static ROBOT robot;

private:

    static bool isReadytoSetGait[AXIS_NUMBER];
	static EGAIT m_currentGait[AXIS_NUMBER];
	static long long int m_gaitStartTime[AXIS_NUMBER];
	static int m_gaitCurrentIndex[AXIS_NUMBER];
	static int m_gaitCurrentIndexForPID[AXIS_NUMBER];
	static Aris::RT_CONTROL::CMotorData m_standStillData[AXIS_NUMBER];
	static Aris::RT_CONTROL::CMotorData m_commandDataMapped[AXIS_NUMBER];
	static Aris::RT_CONTROL::CMotorData m_feedbackDataMapped[AXIS_NUMBER];
	static void MapFeedbackDataIn(Aris::RT_CONTROL::CMachineData& p_data );
	static void MapCommandDataOut(Aris::RT_CONTROL::CMachineData& p_data );
	static void online_trot_TargetXZ(double *IMU_angleVel,double H_robot,int LegID,double *XandZ);
};



#endif /* GAIT_H_ */
