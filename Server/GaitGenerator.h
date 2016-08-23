#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H
#include <aris.h>

#include <thread>
#include <functional>
#include <cstdint>
#include <map>
#include <Robot_Base.h>
#include <Robot_Gait.h>


namespace VersatileGait
{

extern double gridMap[400][400];

enum GaitType
{
    Flat=0,
    Obstacle=1,
    Slope=2,
};
enum GaitMode
{
    Single=1,
    Multi=2,
};
enum GaitState
{
    None=0,
    Walking=1,
    Scanning=2,
    //    PrepareToEnd=3,
    End=4,
};
enum GaitCommand
{
    NoCommand=0,
    //    Go=1,
    Stop=2,
};

enum GaitForceState
{
    Init=0,
    TouchDown=1,
    Stance=2,
   // LiftOff=3,
    Swing=4,

};
void parseAdjustSlope(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
void parseForce(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
void parseForceZeroing(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
int ForceZeroing(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

void parseGoSlopeVision(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);
void parseGoSlopeHuman(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);

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
    double n{1};
    double d{ 0.1 };
    double h{ 0.08 };
    double a{ 0 };
    double b{ 0 };
    int m{GaitMode::Single};
 };


class GaitGenerator
{
public:
    GaitGenerator();
    //    ~GaitGenerator();

    void UpdateIMU(const double* euler);
    void SetWalkParams(const WalkGaitParams param,const double dDist,const double dAngle);
    void UpdateRobotConfig(const double* legPee2b);

    void GaitDetermineNextConfigByVision();
    void GaitDetermineNextConfigByHuman(const double terrainPitch,const double terrainRoll);

    bool GenerateTraj(const int count,const int totalCount,WalkGaitParams param,RobotConfig& config_2_b0);
    void isForceInTransition(double * force,bool* judge);


    // elevationMap w.r.t. the current body config
    double m_TerrainMap[10][10];
    double m_EulerAngles[3];
    //double m_ForceData[6][6];
    const double m_mapReso{0.01};

    WalkGaitParams m_Params;
    RobotConfig m_CurrentConfig_b0;
    RobotConfig m_CurrentConfig_g;
    RobotConfig m_NextConfig_b1;
    RobotConfig m_NextConfig_b0;
    //  int m_GaitType;
    int swingID[3]{0,2,4};
    int stanceID[3]{1,5,3};

    // useful functions
    void GetTerrainHeight2b( double* pos);
    void GetBodyOffset(const double yaw, const double pitch, double* offset);
    void GetPlaneFromStanceLegs(const double* stanceLegs,double* normalVector);
    void TMbody(const double* bodyP,const double* bodyR,double*tmbody);
    void Trans(const double* trans,double*TM);
    void Rx(const double rx,double* TM);
    void Ry(const double ry,double* TM);


    void RyAlongAxis(const double ry,const double *axisPos,double *TM);

    void Rz(const double rz,double* TM);
    double norm(double* vec);
    void normalize(double* vec);
    int sign(double d);
    void TriangleIncenter(const double* stLegs,double* center);
    void LegsTransform(const double* LegPee,const double* TM,double *LegPeeTranformed);
    void GetYawAxis(const double* TM, double* Yaxis);

    void Rot_2_TM(const double theta, const double*u,double* TM);
    void TM_2_Rot(const double* TM, double& theta, double* u);
    void Display(const double *vec,int length);
    void TrajEllipsoid(const double *p0,const double* p1,const int count,const int totalCount,double* legpos);

};





















}



















#endif // GAITGENERATOR_H
