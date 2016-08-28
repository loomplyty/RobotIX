#ifndef GAITPLANNER_H
#define GAITPLANNER_H
#include <aris.h>

#include <thread>
#include <functional>
#include <cstdint>
#include <map>
//#include <Robot_Base.h>
#include <Robot_Gait.h>


namespace Gait
{

extern double gridMap[400][400];

enum TerrainType
{
    Flat=0,
    Obstacle=1,
    Slope=2,
};

enum GaitMode
{
    Single=1,
    Multi=2,
    Dynamic=3,
};

enum GaitState
{
    None=0,
    Walking=1,
    Scanning=2,
    End=3,
};

enum GaitCommand
{
    None=0,
    Stop=1,
};



void parseStopSlope(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);

void parseGoSlope(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);

int GoSlopeByVision(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);
int GoSlopeByHuman(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);



struct RobotConfig
{
    double BodyPee[6];
    double LegPee[18];
};

struct WalkGaitParams :public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 3000 };
    double d{ 0.2 };//stepLength
    double h{ 0.08 };//stepHeight
    double a{ 0 };//navigation
    double b{ 0 };//overall yaw
    int m{GaitMode::Single};//mode
};

const int MaximumConfig=10;
struct RobotPrimitives
{
    int Nconfig;
    RobotConfig Configs[MaximumConfig];
    double Tratio[MaximumConfig];
};




typedef void(*FUNC_ROBOT_CONFIGURE)(aris::dynamic::Model &robot,aris::server::GaitParamBase &param);
typedef void(*FUNC_PRIMITIVE_GENERATION)(RobotPrimitives &primitives);
typedef void(*FUNC_TRAJ_GENERATION)(const int count, const int totalCount,RobotPrimitives &primitives,double* traj);


typedef void(*FUNC_FOOTHOLD_SELECTION)(double* map, double* footholds);


class GaitPlanner
{
public:
    GaitPlanner();
    ~GaitPlanner();

    void ConfigureRobot(aris::dynamic::Model &robot,aris::server::GaitParamBase &param);// data of sensors are in param.


    void UpdateIMU(const double* euler);
    void SetWalkParams(const WalkGaitParams param);
    void UpdateRobotConfig(const double* legPee2b);

    void GaitDetermineNextConfigByVision();
    void GaitDetermineNextConfigByHuman(const double terrainPitch,const double terrainRoll);

    void GenerateTraj(const int count,const int totalCount,RobotConfig& config_2_b0);

    // elevationMap w.r.t. the current body config
    double m_TerrainMap[10][10];
    double m_EulerAngles[3];
    //double m_ForceData[6][6];
    const double m_mapReso{0.01};
public:

    aris::server::GaitParamBase m_param;

    RobotConfig m_CurrentConfig_b0;
    RobotConfig m_CurrentConfig_g;
    RobotConfig m_NextConfig_b1;
    RobotConfig m_NextConfig_b0;
    //  int m_GaitType;
    int swingID[3]{0,2,4};
    int stanceID[3]{1,5,3};

    // useful functions


};

namespace Computation
{
    void c_trans(const double* trans,double*TM);
    void c_rx(const double rx,double* TM);
    void c_ry(const double ry,double* TM);
    void c_rz(const double rz,double* TM);
    double c_norm(double* vec);
    void c_normalize(double* vec);
    int c_sign(const double d);

}

namespace Algorithm {




}


void GetTerrainHeight2b( double* pos);
void GetBodyOffset(const double yaw, const double pitch, double* offset);
void GetPlaneFromStanceLegs(const double* stanceLegs,double* normalVector);
void TMbody(const double* bodyP,const double* bodyR,double*tmbody);


void RyAlongAxis(const double ry,const double *axisPos,double *TM);

void Rz(const double rz,double* TM);

void TriangleIncenter(const double* stLegs,double* center);
void LegsTransform(const double* LegPee,const double* TM,double *LegPeeTranformed);
void GetYawAxis(const double* TM, double* Yaxis);

void Rot_2_TM(const double theta, const double*u,double* TM);
void TM_2_Rot(const double* TM, double& theta, double* u);
void Display(const double *vec,int length);
void TrajEllipsoid(const double *p0,const double* p1,const int count,const int totalCount,double* legpos);



















}



















#endif //
