#ifndef SOLVER_H
#define SOLVER_H
#include <math.h>
#include <iostream>
// #include "liblo1.h"
#include <time.h>
#include <Robot_Base.h>
#include <Robot_Gait.h>


const double swingID0[3]={0,2,4};
const double swingID1[3]={1,5,3};
const double stdLegPee2B[6][3]={-0.3,-0.85,-0.65,
                          -0.45,-0.85, 0,
                          -0.3, -0.85, 0.65,
                          -0.3,-0.85,-0.65,
                          -0.45,-0.85, 0,
                          -0.3, -0.85, 0.65};

struct optGaitParams final:public aris::server::GaitParamBase
{
    double currentLegPee[6][3];
    double nextLegPee[6][3];

    double twp1SwPee[3][3];
    double twp2SwPee[3][3];

    double wp1LegPee[6][3];
    double wp2LegPee[6][3];

    double wp1BodyPee[3];
    double wp2BodyPee[3];

    double currentBodyPee[3];
    double currentBodyAttitude[3];
    double nextBodyPee[3];
    double nextBodyAttitude[3];

    double COGMargin;
    double collisionMargin;

    double angleTurn;
    double stepLength;

    int swingID[3]={0,2,4};
    int stanceID[3]={1,5,3};

};

class Solver
{
public:
    Solver();
    ~Solver();
    //LoadRobotModel();
    int InitSolver();
    int TerminateSolver();
    //void MergeCurrentMap(const float GridMap[120][120]);

   // void UpdateBodyPose();
    void SetWalkingDir(const double angleTurn);
    void SetStepLength(const double stepLength);
    void SetBodyPitch(const double pitch);
    void SetBodyRoll(const double roll);

    optGaitParams GetRobotStepParams();

    int UpdateStepParam(const float GridMap[400][400]);
    int StepSolve();



private:
    optGaitParams m_params;

    int m_optStepCount;

};

#endif // COMPATIBLE_H
