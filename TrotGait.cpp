#include "TrotGait.h"

using namespace std;
int CTrotGait::m_pointPerSec = 1000;
double CTrotGait::m_foot_pos[18];// = { -0.3, -0.85, -0.65, -0.45, -0.85, 0, -0.3, -0.85, 0.65, 0.3, -0.85, -0.65, 0.45, -0.85, 0, 0.3, -0.85, 0.65 };
double CTrotGait::m_body_pos[6];// = { 0, 0, 0, 0, 0, 0 };
double CTrotGait::m_screw_pos[18];
double CTrotGait::m_midLegDisp[3] = { 0.2, 0.1, 0.1 };
int CTrotGait::m_constGaitCount = 1;
ROBOT CTrotGait::m_robot;

//Gait paras
ETrotGaitState CTrotGait::m_gaitState;
double CTrotGait::m_period;
double CTrotGait::m_raiseMidLegsTime;
double CTrotGait::m_stepSize;
double CTrotGait::m_stepHeight;
double CTrotGait::m_alpha;
int CTrotGait::m_gaitLength;


CTrotGait::CTrotGait()
{
}
CTrotGait::~CTrotGait()
{
}
// Initializing Gait
void CTrotGait::LoadRobot()
{
	m_robot.LoadXML("/usr/Robots/resource/HexapodIII/HexapodIII.xml");
}

void CTrotGait::SetGaitParas(double & p_raiseMidLegsTime,double& p_period, double& p_stepsize, double& p_stepheight,double& p_alpha )
{
	m_raiseMidLegsTime = p_raiseMidLegsTime;
	m_period = p_period;
	m_stepSize = p_stepsize;
	m_stepHeight = p_stepheight;
	m_alpha = p_alpha;
	m_gaitLength=int((m_raiseMidLegsTime*2+m_period*2)*m_pointPerSec);
}
void CTrotGait::GetConstVel(double p_vel)
{
	p_vel = m_stepSize / m_period;
}

int CTrotGait::CalPee(int N,double* p_foot_pos, double* p_body_pos)
{
	int i;
	int N1 = (int)(m_raiseMidLegsTime*m_pointPerSec);
	int N5 = N1;
	int N2 = (int)(m_period / 2 * m_pointPerSec);
	int N3 = 2 * N2;
	int N4 = N2;

	double b_pos_init[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  	double f_pos_init[18] =

	   {-0.3, -0.85, -0.65,
		-0.45, -0.85, 0,
		-0.3, -0.85, 0.65,
		0.3, -0.85, -0.65,
		0.45, -0.85, 0,
		0.3, -0.85, 0.65};


	if (N == 0)
	{
		memcpy(m_foot_pos, f_pos_init, 18 * sizeof(double));
		memcpy(m_body_pos, b_pos_init, 6 * sizeof(double));
		memcpy(p_foot_pos,m_foot_pos,18*sizeof(double));
		memcpy(p_body_pos,m_body_pos,6*sizeof(double));

 		return 0;
	}


 	double f_pos_init1[18];
	double b_pos_init1[6];
	CalPee(0, f_pos_init1, b_pos_init1);

	if (N >= 1 && N <=N1)
	{
		m_gaitState = ETrotGaitState::GaitNone;
		i = N;
		memcpy(m_body_pos, b_pos_init1, 6 * sizeof(double));

		m_foot_pos[3] = f_pos_init1[3]-m_midLegDisp[0] * (-0.5*cos(pi*i / N1) + 0.5);
		m_foot_pos[4] = f_pos_init1[4]+m_midLegDisp[1] * (-0.5*cos(pi*i / N1) + 0.5);
		m_foot_pos[5] = f_pos_init1[5]-m_midLegDisp[2] * (-0.5*cos(pi*i / N1) + 0.5);
 
		m_foot_pos[12] = f_pos_init1[12]+m_midLegDisp[0] * (-0.5*cos(pi*i / N1) + 0.5);
		m_foot_pos[13] = f_pos_init1[13]+m_midLegDisp[1] * (-0.5*cos(pi*i / N1) + 0.5);
		m_foot_pos[14] = f_pos_init1[14]-m_midLegDisp[2] * (-0.5*cos(pi*i / N1) + 0.5);

		memcpy(p_foot_pos,m_foot_pos,18*sizeof(double));
		memcpy(p_body_pos,m_body_pos,6*sizeof(double));
		return 0;
	}


	double f_pos_init2[18];
	double b_pos_init2[6];
	CalPee(N1, f_pos_init2, b_pos_init2);
 
 
	if (N >= N1+1 && N <= N1+N2)
	{
		m_gaitState = ETrotGaitState::GaitAcc;

		i = N - N1;
 		Body_Ground(b_pos_init2, i,m_body_pos);

		Leg_Swing_Ground(f_pos_init2,i,m_foot_pos);
 		m_foot_pos[3] = m_body_pos[3] + f_pos_init2[3];
		m_foot_pos[4] = m_body_pos[4] + f_pos_init2[4];
		m_foot_pos[5] = m_body_pos[5] + f_pos_init2[5];
		Leg_Stance_Ground(f_pos_init2+6, i,m_foot_pos + 6 );
		Leg_Stance_Ground(f_pos_init2+9, i,m_foot_pos + 9 );
		m_foot_pos[12] = m_body_pos[3] + f_pos_init2[12];
		m_foot_pos[13] = m_body_pos[4] + f_pos_init2[13];
		m_foot_pos[14] = m_body_pos[5] + f_pos_init2[14];
		Leg_Swing_Ground(f_pos_init2 + 15,i,m_foot_pos+15 );

		memcpy(p_foot_pos,m_foot_pos,18*sizeof(double));
		memcpy(p_body_pos,m_body_pos,6*sizeof(double));
		return 0;

	}

	double f_pos_init3[18];
	double b_pos_init3[6];
	double b_pos_temp[18];
	CalPee(N1+N2, f_pos_init3, b_pos_init3);

	if (N >= N1 + N2+ 1 && N <= N1 + N2 + N3)
	{
		m_gaitState = ETrotGaitState::GaitCons;

		i = N - N1-N2;
	    Body_Ground(b_pos_init3, i,m_body_pos);

		m_foot_pos[3] = m_body_pos[3] + f_pos_init3[3];
		m_foot_pos[4] = m_body_pos[4] + f_pos_init3[4];
		m_foot_pos[5] = m_body_pos[5] + f_pos_init3[5];

		m_foot_pos[12] = m_body_pos[3] + f_pos_init3[12];
		m_foot_pos[13] = m_body_pos[4] + f_pos_init3[13];
		m_foot_pos[14] = m_body_pos[5] + f_pos_init3[14];


		if (N <= N1 + N2 + N3 / 2)
		{
			Leg_Stance_Ground(f_pos_init3, i,m_foot_pos);
			Leg_Swing_Ground(f_pos_init3+6,i,m_foot_pos+6);
			Leg_Swing_Ground(f_pos_init3+9,i,m_foot_pos+9);
			Leg_Stance_Ground(f_pos_init3+15, i,m_foot_pos+15);

		}

		else
		{
			Leg_Stance_Ground(f_pos_init3,N3/2,b_pos_temp);
			Leg_Swing_Ground(b_pos_temp,i-N3/2,m_foot_pos);

			Leg_Swing_Ground(f_pos_init3+6,N3/2,b_pos_temp+6);
		    Leg_Stance_Ground(b_pos_temp+6,i-N3/2,m_foot_pos+6);

			Leg_Swing_Ground(f_pos_init3+9,N3/2,b_pos_temp+9);
		    Leg_Stance_Ground(b_pos_temp+9,i-N3/2,m_foot_pos+9 );

			Leg_Stance_Ground(f_pos_init3+15,N3/2,b_pos_temp+15);
			Leg_Swing_Ground(b_pos_temp+15,i-N3/2,m_foot_pos+15);
		}


		memcpy(p_foot_pos,m_foot_pos,18*sizeof(double));
		memcpy(p_body_pos,m_body_pos,6*sizeof(double));
		return 0;

	
	}

	double f_pos_init4[18];
	double b_pos_init4[6];
	CalPee(N1+N2+N3, f_pos_init4, b_pos_init4);

	if (N >= N1 + N2+N3 && N <= N1 + N2+N3+N4)
	{
		m_gaitState = ETrotGaitState::GaitDec;

		i = N - N1-N2-N3;
		Body_Ground(b_pos_init4, i,m_body_pos);

		Leg_Stance_Ground(f_pos_init4, i,m_foot_pos);
		m_foot_pos[3] = m_body_pos[3] + f_pos_init4[3];
		m_foot_pos[4] = m_body_pos[4] + f_pos_init4[4];
		m_foot_pos[5] = m_body_pos[5] + f_pos_init4[5];
		Leg_Swing_Ground(f_pos_init4 + 6,  i,m_foot_pos + 6  );
		Leg_Swing_Ground(f_pos_init4 + 9  , i,m_foot_pos + 9  );
		m_foot_pos[12] = m_body_pos[3] + f_pos_init4[12];
		m_foot_pos[13] = m_body_pos[4] + f_pos_init4[13];
		m_foot_pos[14] = m_body_pos[5] + f_pos_init4[14];
		Leg_Stance_Ground(f_pos_init4 + 15 , i,m_foot_pos + 15  );


		memcpy(p_foot_pos,m_foot_pos,18*sizeof(double));
		memcpy(p_body_pos,m_body_pos,6*sizeof(double));
		return 0;

	}

	double f_pos_init5[18];
	double b_pos_init5[6];
	CalPee(N1 + N2 + N3+N4, f_pos_init5, b_pos_init5);
	if (N >= 1+N1 + N2 + N3 + N4 && N <= N1 + N2 + N3 + N4 + N5)
	{
		//cout<<"N"<<N1+N2+N3+N4+N5<<endl;
		m_gaitState = ETrotGaitState::GaitNone;
		i = N;
		memcpy(m_body_pos, b_pos_init5,6*sizeof(double));
 
		m_foot_pos[3] = f_pos_init5[3] + m_midLegDisp[0] * (-0.5*cos(pi*i / N1) + 0.5);
		m_foot_pos[4] = f_pos_init5[4] - m_midLegDisp[1] * (-0.5*cos(pi*i / N1) + 0.5);
		m_foot_pos[5] = f_pos_init5[5] + m_midLegDisp[2] * (-0.5*cos(pi*i / N1) + 0.5);
 
		m_foot_pos[12] = f_pos_init5[12] - m_midLegDisp[0] * (-0.5*cos(pi*i / N1) + 0.5);
		m_foot_pos[13] = f_pos_init5[13] - m_midLegDisp[1] * (-0.5*cos(pi*i / N1) + 0.5);
		m_foot_pos[14] = f_pos_init5[14] + m_midLegDisp[2] * (-0.5*cos(pi*i / N1) + 0.5);
		
		memcpy(p_foot_pos,m_foot_pos,18*sizeof(double));
		memcpy(p_body_pos,m_body_pos,6*sizeof(double));
		return 0;

	}
}


void CTrotGait::CalPin(double * screw_pos)
{
 	m_robot.SetPee(m_foot_pos,m_body_pos);
	m_robot.GetPin(m_screw_pos);
	memcpy(screw_pos,m_screw_pos,sizeof(double)*18);
	for(int i=0;i<18;i++)
		screw_pos[i]=screw_pos[i]*350*65536;
}

void CTrotGait::Leg_Swing_Ground(double* p_InitPos,int N,double* foot_tip)
{ 
	double foot_pos[3];
	int Nswing = (int)(m_pointPerSec*m_period*(1 - m_alpha));

	double D;
	if (m_gaitState == ETrotGaitState::GaitAcc || m_gaitState ==ETrotGaitState::GaitDec)//???
	{
 		D = m_stepSize / 2;


	}
	else
	{
			D = m_stepSize;

	}

	
	if (N >= 1 && N <= Nswing)
	{
		foot_pos[0] = p_InitPos[0];
		foot_pos[1] = p_InitPos[1] - m_stepHeight / 2 * cos(2 * pi*N / Nswing) + m_stepHeight / 2;
		foot_pos[2] = p_InitPos[2] + D / 2 * cos(pi*N / Nswing) - D / 2;
		//cout<<"footpos3"<<foot_pos[3]<<endl;
	}

	if (N > Nswing&&N <= (int)(m_pointPerSec*m_period / 2))
	{
		foot_pos[0] = p_InitPos[0];
		foot_pos[1] = p_InitPos[1];
		foot_pos[2] = p_InitPos[2]-D;


	}
	memcpy(foot_tip,foot_pos,sizeof(double)*3);
}

void CTrotGait::Leg_Stance_Ground(double* p_InitPos, int N,double* foot_tip)
{
	memcpy(foot_tip,p_InitPos,sizeof(double)*3);
}

void CTrotGait::Body_Ground(double* p_InitPos, int N, double* body)
{

	double body_pos[6];
	double Vel = m_stepSize / m_period;
	switch (m_gaitState)
	{
	case ETrotGaitState::GaitAcc:
		memcpy(body_pos, p_InitPos, sizeof(double)* 6);

		body_pos[5]+= -0.5 * Vel / m_period * 2*N /m_pointPerSec*N /m_pointPerSec;
		//cout<<"body_pos[5]"<<-0.5 * Vel / (m_period / 2)*(N / m_pointPerSec)*(N / m_pointPerSec)<<endl;
        //cout<<"m_pointPerSec"<<m_pointPerSec<<endl;
		//cout<<"period"<<m_period<<endl;
		//cout<<"Vel"<<Vel<<endl;
		//cout<<"N"<<N<<endl;
		//cout<<"body_pos[5]"<<body_pos[5]<<endl;
		break;
	case ETrotGaitState::GaitCons:
		memcpy(body_pos, p_InitPos, sizeof(double)* 6);
		body_pos[5] += -Vel*N / m_pointPerSec;
		break;
	case ETrotGaitState::GaitDec:
		memcpy(body_pos, p_InitPos, sizeof(double)* 6);
		body_pos[5] += -Vel*N / m_pointPerSec+ 0.5 * Vel/ m_period * 2*N / m_pointPerSec*N / m_pointPerSec;
		break;
	default:
		break;

	}

	memcpy(body,body_pos,sizeof(double)*6);
}

