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

void parseGoSlope(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg);

int GoSlope(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

struct RobotConfig
{
    double BodyPee[6];
    double LegPee[18];
};
struct WalkGaitParams :public aris::server::GaitParamBase
{
    double d;
    double b;
    double a;
    double t;
    double h;
};

enum GaitType
{
    Flat=0,
    Obstacle=1,
    Slope=2,
};



class GaitGenerator
{
public:
    GaitGenerator();
    ~GaitGenerator();

    void UpdateVision(const double map[400][400]);
    void UpdateIMU(const double* euler);
    void SetWalkParams(const WalkGaitParams param);
    void UpdateRobotConfig(const double* legPee2b);

    void GaitDetermineNextConfig();
 //    GaitGenerateTraj();

    // elevationMap w.r.t. the current body config
    double m_TerrainMap[400][400];
    double m_EulerAngles[3];
    double m_ForceData[6][6];
    const double m_mapReso{0.01};
    WalkGaitParams m_Params;
    RobotConfig m_CurrentConfig_b0;
    RobotConfig m_CurrentConfig_g;
    RobotConfig m_NextConfig_b1;
    RobotConfig m_NextConfig_b0;
    int m_GaitType;
     int swingID[3]={0,2,4};
    int stanceID[3]={1,5,3};

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
    void Display(double *vec,int length);

};





















}



















#endif // GAITGENERATOR_H
