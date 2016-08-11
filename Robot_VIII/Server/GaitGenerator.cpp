#include "GaitGenerator.h"
#include "math.h"
#define YawLimit 0.08
#include "rtdk.h"


using namespace std;

namespace VersatileGait
{

const double stdLegPee2B[18]=
{  -0.3,-0.85,-0.65,
   -0.45,-0.85,0,
    -0.3,-0.85,0.65,
    0.3,-0.85,-0.65,
    0.45,-0.85,0,
    0.3,-0.85,0.65
};

void parseGoSlope(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
   WalkGaitParams param;
   param.a=PI/3;
   param.b=0.1;
   param.d=0.2;
   param.h=0.08;
   msg.copyStruct(param);

   GaitGenerator g;
   double map[400][400];
    std::cout<<"map: "<<map[0][0]<<" "<<map[400][400]<<" "<<map[500][501]<< std::endl;
    g.UpdateVision(map);
    double euler[3];
    euler[0]=0;
    euler[1]=0;
    euler[2]=0;
    g.UpdateIMU(euler);
    cout<<"g euler"<<g.m_EulerAngles[0]<<" "<<g.m_EulerAngles[1]<<" "<<g.m_EulerAngles[2]<<endl;
    g.SetWalkParams(param);
    g.UpdateRobotConfig(stdLegPee2B);
//    cout<<"legPee"<<endl;
//    g.Display(g.m_CurrentConfig_g.LegPee,18);
//    cout<<"bodypee"<<endl;
//    g.Display(g.m_CurrentConfig_g.BodyPee,6);
//    cout<<"legPee"<<endl;
//    g.Display(g.m_CurrentConfig_b0.LegPee,18);
//    cout<<"bodypee"<<endl;
//    g.Display(g.m_CurrentConfig_b0.BodyPee,6);

    g.GaitDetermineNextConfig();
//    cout<<"legPeeNext"<<endl;
//    g.Display(g.m_NextConfig_b0.LegPee,18);
//    cout<<"bodypeeNext"<<endl;
//    g.Display(g.m_NextConfig_b0.BodyPee,6);

}

int GoSlope(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{


}

GaitGenerator::GaitGenerator()
{

}

GaitGenerator::~GaitGenerator()
{

}


void GaitGenerator::GaitDetermineNextConfig()
{
 //  1. get the estimated next robot configuration
    double est_TM_b1_2_b0[16];
    double dstraight;

    if(m_Params.b!=0)
    {
        double r;
        r=m_Params.d/m_Params.b;
        dstraight=abs(r*sin(m_Params.b/2)*2);
        double axisRaw[3];
        axisRaw[0]=-r*cos(m_Params.a);
        axisRaw[1]=0;
        axisRaw[2]=r*sin(m_Params.a);
        RyAlongAxis(m_Params.b,axisRaw,est_TM_b1_2_b0);
    }
    else
    {
        dstraight=m_Params.d;
        double Displacement[3];
        Displacement[0]=-dstraight*sin(m_Params.a);
        Displacement[1]=0;
        Displacement[2]=-dstraight*cos(m_Params.a);
        Trans(Displacement,est_TM_b1_2_b0);
    }
//    cout<<"dstraight"<<dstraight<<endl;
//    cout<<"est body tm"<<endl;
//    Display(est_TM_b1_2_b0,16);

//2. Find  footholds
    double TM_b0_2_g[16];
    aris::dynamic::s_pe2pm(m_CurrentConfig_g.BodyPee,TM_b0_2_g,"213");
    double est_TM_b1_2_g[16];
    aris::dynamic::s_pm_dot_pm(TM_b0_2_g,est_TM_b1_2_b0,est_TM_b1_2_g);
    double estBody_2_g[6];
    aris::dynamic::s_pm2pe(est_TM_b1_2_g,estBody_2_g,"213");
    double est_BodyOffset[3];
    GetBodyOffset(estBody_2_g[4],estBody_2_g[5],est_BodyOffset);

//    cout<<"est pitch"<<estBody_2_g[4]<<endl;
//    cout<<"est roll"<<estBody_2_g[5]<<endl;

    //estimate  footholds w.r.t. b0
    double estSWFoothold_2_b0[9];
    double estSWFoothold_2_b1[9];
    for(int i=0;i<3;i++)
    {
        estSWFoothold_2_b1[i*3+0]=-est_BodyOffset[0]+stdLegPee2B[swingID[i]*3+0]-sin(m_Params.a)*dstraight/2;
        estSWFoothold_2_b1[i*3+1]=-est_BodyOffset[1]+stdLegPee2B[swingID[i]*3+1];
        estSWFoothold_2_b1[i*3+2]=-est_BodyOffset[2]+stdLegPee2B[swingID[i]*3+2]-cos(m_Params.a)*dstraight/2;//relavant to the angle of a
        aris::dynamic::s_pm_dot_pnt(est_TM_b1_2_b0,&estSWFoothold_2_b1[i*3],&estSWFoothold_2_b0[i*3]);
    }

   //find footholds w.r.t. b0 with terrain information
    double SWFoothold_2_b0[9];
    memcpy(SWFoothold_2_b0,estSWFoothold_2_b0,sizeof(SWFoothold_2_b0));
    GetTerrainHeight2b(&SWFoothold_2_b0[0]);
    GetTerrainHeight2b(&SWFoothold_2_b0[3]);
    GetTerrainHeight2b(&SWFoothold_2_b0[6]);
//    Display(SWFoothold_2_b0,9);

//3.  determine bodypos w.r.t. b0
    double x1_2_b0[3];
    double x1_2_b0_prime[3];
    double y1_2_b0[3];
    double z1_2_b0[3];
    GetPlaneFromStanceLegs(SWFoothold_2_b0,y1_2_b0);

    x1_2_b0_prime[0]=cos(m_Params.b);
    x1_2_b0_prime[1]=0;
    x1_2_b0_prime[2]=-sin(m_Params.b);
    aris::dynamic::s_cro3(x1_2_b0_prime,y1_2_b0,z1_2_b0);
    aris::dynamic::s_cro3(y1_2_b0,z1_2_b0,x1_2_b0);
    normalize(x1_2_b0);
    normalize(y1_2_b0);
    normalize(z1_2_b0);

    double TM_b1_2_b0[16];
    TM_b1_2_b0[0]=x1_2_b0[0];
    TM_b1_2_b0[1]=y1_2_b0[0];
    TM_b1_2_b0[2]=z1_2_b0[0];
    TM_b1_2_b0[4]=x1_2_b0[1];
    TM_b1_2_b0[5]=y1_2_b0[1];
    TM_b1_2_b0[6]=z1_2_b0[1];
    TM_b1_2_b0[8]=x1_2_b0[2];
    TM_b1_2_b0[9]=y1_2_b0[2];
    TM_b1_2_b0[10]=z1_2_b0[2];
    TM_b1_2_b0[15]=1;

    cout<<"x y z 2 bo"<<endl;
    Display(x1_2_b0,3);
    Display(y1_2_b0,3);
    Display(z1_2_b0,3);
    cout<<endl;

//     Display(TM_b1_2_b0,16);
    double Body_2_b0[6];
    aris::dynamic::s_pm2pe(TM_b1_2_b0,Body_2_b0,"213");
    double BodyOffset[3];
    GetBodyOffset(Body_2_b0[4],Body_2_b0[5],BodyOffset);
    double SPCenter[3];
    TriangleIncenter(SWFoothold_2_b0,SPCenter);



    double BodyPos_2_b1_spCenter[3];
    double BodyPos_2_b0_spCenter[3];

    BodyPos_2_b1_spCenter[0]=dstraight/2*sin(m_Params.a)+BodyOffset[0];
    BodyPos_2_b1_spCenter[1]=-stdLegPee2B[1]+BodyOffset[1];//stdLegPee2B[1]=-0.85
    BodyPos_2_b1_spCenter[2]=dstraight/2*cos(m_Params.a)+BodyOffset[2];
    aris::dynamic::s_pm_dot_v3(TM_b1_2_b0,BodyPos_2_b1_spCenter,BodyPos_2_b0_spCenter);

//    cout<<" BodyPos_2_b0_spCenter"<<endl;
//    Display(BodyPos_2_b0_spCenter,3);
//    cout<<" BodyPos_2_b1_spCenter"<<endl;
//    Display(BodyPos_2_b1_spCenter,3);// should be ok now

    Body_2_b0[0]=BodyPos_2_b0_spCenter[0]+SPCenter[0];
    Body_2_b0[1]=BodyPos_2_b0_spCenter[1]+SPCenter[1];
    Body_2_b0[2]=BodyPos_2_b0_spCenter[2]+SPCenter[2];
    TM_b1_2_b0[3]=Body_2_b0[0];
    TM_b1_2_b0[7]=Body_2_b0[1];
    TM_b1_2_b0[11]=Body_2_b0[2];


    // update body and leg configuration
    memcpy(&m_NextConfig_b0.BodyPee,Body_2_b0,sizeof(double)*6);
//    cout<<"bodyPee2b0"<<endl;
//    Display(m_NextConfig_b0.BodyPee,6);
//    cout<<"TMB1_2_B0"<<endl;
//    Display(TM_b1_2_b0,16);
    for(int i=0;i<3;i++)
    {
        memcpy(&m_NextConfig_b0.LegPee[swingID[i]*3],&SWFoothold_2_b0[i*3],sizeof(double)*3);
        memcpy(&m_NextConfig_b0.LegPee[stanceID[i]*3],&m_CurrentConfig_b0.LegPee[stanceID[i]*3],sizeof(double)*3);
    }

    memset(m_NextConfig_b1.BodyPee,0,sizeof(m_NextConfig_b1.BodyPee));
    double TM_b0_2_b1[16];
    aris::dynamic::s_inv_pm(TM_b1_2_b0,TM_b0_2_b1);


    LegsTransform(m_NextConfig_b0.LegPee,TM_b0_2_b1,m_NextConfig_b1.LegPee);

    cout<<"estTM B0_2B1"<<endl;
    Display(est_TM_b1_2_b0,16);
        cout<<"tri center"<<endl;
        Display(SPCenter,3);
        cout<<"BodyPos_2_b1_spCenter"<<endl;
       Display(BodyPos_2_b1_spCenter,3);
       cout<<"BodyPos_2_b0_spCenter"<<endl;
      Display(BodyPos_2_b0_spCenter,3);
      cout<<"Body_2_b0"<<endl;
     Display(Body_2_b0,3);

    cout<<"TMB1_2_B0"<<endl;
    Display(TM_b1_2_b0,16);
    cout<<"TMB0_2_B1"<<endl;
    Display(TM_b0_2_b1,16);
    cout<<"currentlegPee2B0"<<endl;
    Display(m_CurrentConfig_b0.LegPee,18);
    cout<<"legPee2B0"<<endl;
    Display(m_NextConfig_b0.LegPee,18);
    cout<<"legPee2B1"<<endl;
    Display(m_NextConfig_b1.LegPee,18);


}


void GaitGenerator::UpdateRobotConfig(const double *legPee2b)
{
     // Initially, ground CS is set on the body Geometric center
    memset(m_CurrentConfig_b0.BodyPee,0,sizeof(double)*6);
    memcpy(m_CurrentConfig_b0.LegPee,legPee2b,sizeof(double)*18);

    memset(&m_CurrentConfig_g.BodyPee[0],0,sizeof(double)*3);
    memcpy(&m_CurrentConfig_g.BodyPee[3],m_EulerAngles,sizeof(double)*3);
    m_CurrentConfig_g.BodyPee[3]=0;

    double TM_b0_2_g[16];
    aris::dynamic::s_pe2pm(m_CurrentConfig_g.BodyPee,TM_b0_2_g,"213");
    Display(TM_b0_2_g,16);

    LegsTransform(legPee2b,TM_b0_2_g,m_CurrentConfig_g.LegPee);
}

void GaitGenerator::LegsTransform(const double *LegPee, const double *TM, double *LegPeeTranformed)
{
   for(int i=0;i<6;i++)
   {
       aris::dynamic::s_pm_dot_pnt(TM,&LegPee[i*3],&LegPeeTranformed[i*3]);
    }
}

void GaitGenerator::Trans(const double *trans,double*TM)
{
  TM[0]=1;
  TM[1]=0;
  TM[2]=0;
  TM[3]=trans[0];
  TM[4]=0;
  TM[5]=1;
  TM[6]=0;
  TM[7]=trans[1];
  TM[8]=0;
  TM[9]=0;
  TM[10]=1;
  TM[11]=trans[2];
  TM[12]=0;
  TM[13]=0;
  TM[14]=0;
  TM[15]=1;
 }

void GaitGenerator::Rx(const double rx,double* TM)
{
     TM[0]=1;
    TM[1]=0;
    TM[2]=0;
    TM[3]=0;
    TM[4]=0;
    TM[5]=cos(rx);
    TM[6]=-sin(rx);
    TM[7]=0;
    TM[8]=0;
    TM[9]=sin(rx);
    TM[10]=cos(rx);
    TM[11]=0;
    TM[12]=0;
    TM[13]=0;
    TM[14]=0;
    TM[15]=1;
 }
void GaitGenerator::Ry(const double ry,double* TM)
{
     TM[0]=cos(ry);
    TM[1]=0;
    TM[2]=sin(ry);
    TM[3]=0;
    TM[4]=0;
    TM[5]=1;
    TM[6]=0;
    TM[7]=0;
    TM[8]=-sin(ry);
    TM[9]=0;
    TM[10]=cos(ry);
    TM[11]=0;
    TM[12]=0;
    TM[13]=0;
    TM[14]=0;
    TM[15]=1;
 }
void GaitGenerator::Rz(const double rz,double* TM)
{
     TM[0]=cos(rz);
    TM[1]=-sin(rz);
    TM[2]=0;
    TM[3]=0;
    TM[4]=sin(rz);
    TM[5]=cos(rz);
    TM[6]=0;
    TM[7]=0;
    TM[8]=0;
    TM[9]=0;
    TM[10]=1;
    TM[11]=0;
    TM[12]=0;
    TM[13]=0;
    TM[14]=0;
    TM[15]=1;
 }
void GaitGenerator::RyAlongAxis(const double ry, const double *axisPos,double *TM)
{
    double TransTo[16];
    double Rot[16];
    double TransBack[16];
    Trans(axisPos,TransTo);
    double minus_axisPos[3];
    minus_axisPos[0]=-axisPos[0];
    minus_axisPos[1]=-axisPos[1];
    minus_axisPos[2]=-axisPos[2];
    Trans(minus_axisPos,TransBack);
    Ry(ry,Rot);
    double TMmid[16];
    aris::dynamic::s_pm_dot_pm(TransTo,Rot,TMmid);
    aris::dynamic::s_pm_dot_pm(TMmid,TransBack,TM);
}

void GaitGenerator::TMbody(const double *bodyP, const double *bodyR, double *tmbody)
{
    double TMtrans[16];
    double TMrot[16];
    Trans(bodyP,TMtrans);
    aris::dynamic::s_pe2pm(bodyR,TMrot,"213");
    aris::dynamic::s_pm_dot_pm(TMtrans,TMrot,tmbody);
}
void GaitGenerator::SetWalkParams(const WalkGaitParams param)
{
     cout<<"param size"<<sizeof(param)<<endl;
     memcpy(&m_Params,&param,sizeof(param));
     cout<<"param b"<<m_Params.b<<endl;
}

void GaitGenerator::UpdateVision(const double map[400][400])
{
    memcpy(m_TerrainMap,map,sizeof(map));
}
void GaitGenerator::UpdateIMU(const double* euler)
{
    memcpy(m_EulerAngles,euler,sizeof(double)*3);
}
void GaitGenerator::GetTerrainHeight2b( double *pos)
{
    //map 400*400*0.25 origined on the robot center
    //suppose vision device is mounted along the -z direction

    double gridRaw[2];
    gridRaw[0]=pos[0]/this->m_mapReso+200;
    gridRaw[1]=-pos[2]/this->m_mapReso;
    int grid[2];
    if(ceil(gridRaw[0])/2+floor(gridRaw[0])/2>gridRaw[0])
        grid[0]=floor(gridRaw[0]);
    else
        grid[0]=ceil(gridRaw[0]);

    if(ceil(gridRaw[1])/2+floor(gridRaw[1])/2>gridRaw[1])
        grid[1]=floor(gridRaw[1]);
    else
        grid[1]=ceil(gridRaw[1]);

    pos[1]=m_TerrainMap[grid[0]][grid[1]];
    // for test
    pos[1]=-0.85;
}
void GaitGenerator::GetBodyOffset(const double pitch, const double roll, double* offset)
{
    // only for test
    offset[0]=0;
    offset[1]=0;
    offset[2]=0;


}
void GaitGenerator::GetPlaneFromStanceLegs(const double *stanceLegs, double *normalVector)
{
    double stLeg_2_to_1[3];
    double stLeg_3_to_1[3];

    stLeg_2_to_1[0]=stanceLegs[0]-stanceLegs[3];
    stLeg_2_to_1[1]=stanceLegs[1]-stanceLegs[4];
    stLeg_2_to_1[2]=stanceLegs[2]-stanceLegs[5];
    stLeg_3_to_1[0]=stanceLegs[0]-stanceLegs[6];
    stLeg_3_to_1[1]=stanceLegs[1]-stanceLegs[7];
    stLeg_3_to_1[2]=stanceLegs[2]-stanceLegs[8];


    aris::dynamic::s_cro3(stLeg_2_to_1,stLeg_3_to_1,normalVector);
    normalVector[0]=normalVector[0]*sign(normalVector[1])/norm(normalVector);
    normalVector[1]=normalVector[1]*sign(normalVector[1])/norm(normalVector);
    normalVector[2]=normalVector[2]*sign(normalVector[1])/norm(normalVector);
}
double GaitGenerator::norm(double *vec)
{
    return sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
}

void GaitGenerator::normalize(double *vec)
{
    vec[0]=vec[0]/norm(vec);
    vec[1]=vec[1]/norm(vec);
    vec[2]=vec[2]/norm(vec);
}

int GaitGenerator::sign(double d)
{
    if(d>0)
        return 1;
    else if(d<0)
        return -1;
    else
        return 0;
}
void GaitGenerator::TriangleIncenter(const double *stLegs, double *center)
{
    double P1[3];
    double P2[3];
    double P3[3];

    memcpy(P1,&stLegs[0],sizeof(double)*3);
    memcpy(P2,&stLegs[3],sizeof(double)*3);
    memcpy(P3,&stLegs[6],sizeof(double)*3);

    double P12[3];
    double P23[3];
    double P31[3];
    double L1,L2,L3;

    P12[0]=P2[0]-P1[0];
    P12[1]=P2[1]-P1[1];
    P12[2]=P2[2]-P1[2];
    P23[0]=P3[0]-P2[0];
    P23[1]=P3[1]-P2[1];
    P23[2]=P3[2]-P2[2];
    P31[0]=P1[0]-P3[0];
    P31[1]=P1[1]-P3[1];
    P31[2]=P1[2]-P3[2];
    L1=norm(P23);
    L2=norm(P31);
    L3=norm(P12);

    center[0]=(P1[0]*L1+P2[0]*L2+P3[0]*L3)/(L1+L2+L3);
    center[1]=(P1[1]*L1+P2[1]*L2+P3[1]*L3)/(L1+L2+L3);
    center[2]=(P1[2]*L1+P2[2]*L2+P3[2]*L3)/(L1+L2+L3);

}

void GaitGenerator::Display(double *vec,int length)
{
    if(length==16)
    {
        for(int i=0;i<4;i++)
        {
            std::cout<<vec[i*4]<<", "<<vec[i*4+1]<<", "<<vec[i*4+2]<<", "<<vec[i*4+3]<<std::endl;
        }
        std::cout<<std::endl;
    }


    else
    {
        int row;
        row=length/3;
        for(int i=0;i<row;i++)
        {
            std::cout<<vec[i*3]<<", "<<vec[i*3+1]<<", "<<vec[i*3+2]<<std::endl;
        }
        std::cout<<std::endl;

    }


}

}



