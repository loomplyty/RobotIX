#include "Solver.h"

Solver::Solver()
{
/*    m_optStepCount=0;
    m_bodyPose={0,0,0,0,0,0};
    m_solverIn.currentBodyPee={0,0,0};
    m_solverIn.currentBodyPee={0,0,0};*/

    InitSolver();
}

int Solver::InitSolver()
{
    std::cout<<"OPT solver initialized!"<<std::endl;
}

int Solver::TerminateSolver()
{
    std::cout<<"OPT solver Terminated!"<<std::endl;
}

/*void Solver::MergeCurrentMap(const float GridMap[120][120])
{
    //** first: transform the grid map to the current body coordinate system

   //** second:merge
}*/

void Solver::SetStepLength(const double stepLength)
{
    m_params.stepLength=stepLength;
}

void Solver::SetWalkingDir(const double angleTurn)
{
    m_params.angleTurn=angleTurn;
}

int Solver::UpdateStepParam(const float GridMap[400][400])
{
    m_optStepCount+=1;
    if(m_optStepCount%2==1)
    {
        memcpy(m_params.swingID,swingID0,sizeof(swingID0));
        memcpy(m_params.stanceID,swingID1,sizeof(swingID1));
    }
    else
    {
        memcpy(m_params.swingID,swingID1,sizeof(swingID1));
        memcpy(m_params.stanceID,swingID0,sizeof(swingID0));
    }

  //  extNextBodyPee=m_params.currentBodyPee+m_params.stepLength*m_params.walkingDir
   // need add aris into this to do some matrix computation



 //  m_params.nextLegPee

    return 1;
}


int Solver::StepSolve()
{


    //**allocate varaible space for matlab function

    //**call the matlab function and get results

}









