#include <iomanip>

#include "math.h"

#include "iostream"
#include "fstream"
#include "stdio.h"
#include "Hexapod_Robot.h"

//#define PI 3.14159265358979323846

using namespace std;
using namespace Hexapod_Robot;

enum ETrotGaitState
{
	GaitNone= 0,
	GaitAcc = 1,
	GaitCons= 2,
    GaitDec = 3,
};
enum ETrotGaitPhase
{
	Invalid=0,
	GaitStance=1,
	GaitSwing=2,
};

class CTrotGait
{

public:
	CTrotGait();
	~CTrotGait();
	int CalPee(int N,double* p_foot_pos,double* p_body_pos);

	static int m_pointPerSec;
	static double m_foot_pos[18];
	static double m_body_pos[6];
	static double m_screw_pos[18];
	static double m_midLegDisp[3];
	static ROBOT m_robot;
	static ETrotGaitPhase m_leg_phase[6];

	//GAIT PARAS
	static ETrotGaitState m_gaitState;
	static double m_raiseMidLegsTime;
	static double m_period;
	static double m_stepSize;
	static double m_stepHeight;
	static double m_alpha;
	static int m_constGaitCount;
	static int m_gaitLength;

	void CalPin(double* screw_pos);
	void LoadRobot();
	void SetGaitParas(double & p_raiseMidLegsTime,double& p_period, double& p_stepsize, double& p_stepheight, double& p_alpha );
private:
	ETrotGaitPhase Leg_Stance_Ground(double* InitPos,int N,double* foot_tip);
	ETrotGaitPhase Leg_Swing_Ground(double* InitPos, int N,double* foot_tip);
	void Body_Ground(double* InitPos,int N,double* body);

	//Getters 
	void GetConstVel(double p_vel);
 };

 

 
