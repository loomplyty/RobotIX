#include "GaitPlanner.h"
#include <math.h>
#include "rtdk.h"



#define YawLimit 0.08



using namespace std;

namespace Gait
{

const double stdLegPee2B[18]=
{  -0.3,-0.85,-0.65,
   -0.45,-0.85,0,
   -0.3,-0.85,0.65,
   0.3,-0.85,-0.65,
   0.45,-0.85,0,
   0.3,-0.85,0.65
};







static GaitGenerator g;
static int gaitState=GaitState::None;
static int gaitCommand=GaitCommand::NoCommand;
static double pitch_2_b0=0;
static double roll_2_b0=0;

void parseStopSlope(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    gaitCommand=GaitCommand::Stop;
    //   msg.copyStruct(param);
    cout<<"parse finished"<<endl;

}


void parseGoSlope(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    WalkGaitParams param;
    for (auto &i : params)
    {
        if (i.first == "totalCount")
        {
            param.totalCount = std::stoi(i.second);
        }
        else if (i.first == "distance")
        {
            param.d = stod(i.second);
        }
        else if (i.first == "height")
        {
            param.h = stod(i.second);
        }
        else if (i.first == "alpha")
        {
            param.a = stod(i.second);
        }
        else if (i.first == "beta")
        {
            param.b = stod(i.second);
        }
        else if (i.first == "mode")
        {
            param.m= stoi(i.second);
        }
        else if(i.first == "dpitch")
        {
            pitch_2_b0+=stod(i.second);

        }
        else if(i.first == "droll")
        {
            roll_2_b0=stod(i.second);
        }



    }
    msg.copyStruct(param);
    cout<<"parse finished"<<endl;


    //   GaitGenerator g;
    //    double euler[3];
    //    euler[0]=0;
    //   euler[1]=0;
    //   euler[2]=0;
    //   g.UpdateIMU(euler);
    //   // cout<<"g euler"<<g.m_EulerAngles[0]<<" "<<g.m_EulerAngles[1]<<" "<<g.m_EulerAngles[2]<<endl;
    //    g.SetWalkParams(param);
    //    g.UpdateRobotConfig(stdLegPee2B);
    //////    cout<<"legPee"<<endl;
    //////    g.Display(g.m_CurrentConfig_g.LegPee,18);
    //////    cout<<"bodypee"<<endl;
    //////    g.Display(g.m_CurrentConfig_g.BodyPee,6);
    //////    cout<<"legPee"<<endl;
    //////    g.Display(g.m_CurrentConfig_b0.LegPee,18);
    //////    cout<<"bodypee"<<endl;
    //////    g.Display(g.m_CurrentConfig_b0.BodyPee,6);

    //    g.GaitDetermineNextConfigByVision();
    //    cout<<"legPeeNext"<<endl;
    //    g.Display(g.m_NextConfig_b0.LegPee,18);
    //    cout<<"bodypeeNext"<<endl;
    //    g.Display(g.m_NextConfig_b0.BodyPee,6);

    //    RobotConfig config_0_2_b0;
    //    RobotConfig config_N_2_b0;

    //    g.GenerateTraj(0,1000,config_0_2_b0);


    //    g.GenerateTraj(1000,1000,config_N_2_b0);




    //    cout<<"config0 body"<<endl;
    //    g.Display(config_0_2_b0.BodyPee,6);
    //    cout<<"config0 leg"<<endl;
    //    g.Display(config_0_2_b0.LegPee,18);
    //    cout<<"configN body"<<endl;
    //    g.Display(config_N_2_b0.BodyPee,6);
    //    cout<<"configNleg"<<endl;
    //    g.Display(config_N_2_b0.LegPee,18);



}

// could be one step gait or several-step gait
int GoSlopeByVision(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{

    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param=static_cast<const WalkGaitParams &>(param_in);
    static aris::dynamic::FloatMarker beginMak{robot.ground()};

    static int stepNumFinished=0;
    static int stepCount=0;

    switch(gaitState)
    {
    case GaitState::None:
        gaitState=GaitState::Scanning;
        return 1;
    case GaitState::Scanning:
        static bool isScanningFinished=false;
        rt_printf("scanning...\n");
        rt_printf("scanning...\n");
        rt_printf("scanning...\n");

        //send command and m_nextConfig_2_b0 to vision and get map
        //receive map and copy it to map
        //then walk
        if(gaitCommand==GaitCommand::Stop)
        {
            rt_printf("get ending command\n");
            gaitState=GaitState::End;
            gaitCommand=GaitCommand::NoCommand;
        }
        else
            gaitState=GaitState::Walking;
        return 1;

    case GaitState::Walking:
        if(stepCount==0)
        {
            cout<<"a new step begins..."<<"stepCount:"<<stepCount<<endl;
            cout<<"count "<<param.count<<endl;
            double euler[3];
            // param.imu_data->toEulBody2Ground(euler,"213");
            euler[0]=0.1;
            euler[1]=0.1;
            euler[2]=0.1;
            g.UpdateIMU(euler);

            g.SetWalkParams(param);
            double currentLegPee2b[18];


            robot.GetPee(currentLegPee2b,robot.body());

            beginMak.setPrtPm(*robot.body().pm());
            beginMak.update();



            g.UpdateRobotConfig(currentLegPee2b);
            g.GaitDetermineNextConfigByVision();
        }
        RobotConfig config_2_b0;
        g.GenerateTraj(stepCount+1,param.totalCount,config_2_b0);
        if(param.count%300==0)
        {

            //              cout<<"(stepCount)/totalCount"<<double((stepCount))/param.totalCount<<endl;
            //              cout<<"body"<<config_2_b0.BodyPee[0]<<" "<<config_2_b0.BodyPee[1]<<" "<<config_2_b0.BodyPee[2]<<" "<<config_2_b0.BodyPee[3]<<" "<<config_2_b0.BodyPee[4]<<" "<<config_2_b0.BodyPee[5]<<endl;
            //              cout<<"legPee2b0"<<endl;
            //              g.Display(config_2_b0.LegPee,18);

        }

        robot.SetPee(config_2_b0.LegPee,beginMak);
        robot.SetPeb(config_2_b0.BodyPee,beginMak,"213");

        stepCount+=1;

        if (stepCount==param.totalCount)
        {
            stepNumFinished+=1;
            stepCount=0;

            if (gaitCommand==GaitCommand::Stop)
            {
                gaitState=GaitState::End;
                gaitCommand=GaitCommand::NoCommand;
            }
            else if(param.m==GaitMode::Single)
                gaitState=GaitState::End;
            else
                gaitState=GaitState::Scanning;

        }
        return 1;

    case GaitState::End:
    default:
        gaitState=GaitState::None;
        return 0;
    }
}



// only one step gait
int GoSlopeByHuman(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{

    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param=static_cast<const WalkGaitParams &>(param_in);
    static aris::dynamic::FloatMarker beginMak{robot.ground()};

    static int stepCount=0;

    switch(gaitState)
    {
    case GaitState::None:
        gaitState=GaitState::Walking;
        return 1;


    case GaitState::Walking:
        if(stepCount==0)
        {
            cout<<"a new step begins..."<<"stepCount:"<<stepCount<<endl;
            cout<<"count "<<param.count<<endl;
            double euler[3];
            // param.imu_data->toEulBody2Ground(euler,"213");
            euler[0]=0.1;
            euler[1]=0.1;
            euler[2]=0.1;
            g.UpdateIMU(euler);

            g.SetWalkParams(param);
            double currentLegPee2b[18];


            robot.GetPee(currentLegPee2b,robot.body());

            beginMak.setPrtPm(*robot.body().pm());
            beginMak.update();

            g.UpdateRobotConfig(currentLegPee2b);
            g.GaitDetermineNextConfigByHuman(pitch_2_b0,roll_2_b0);
        }
        RobotConfig config_2_b0;
        g.GenerateTraj(stepCount+1,param.totalCount,config_2_b0);
        if(param.count%300==0)
        {

            //              cout<<"(stepCount)/totalCount"<<double((stepCount))/param.totalCount<<endl;
            //              cout<<"body"<<config_2_b0.BodyPee[0]<<" "<<config_2_b0.BodyPee[1]<<" "<<config_2_b0.BodyPee[2]<<" "<<config_2_b0.BodyPee[3]<<" "<<config_2_b0.BodyPee[4]<<" "<<config_2_b0.BodyPee[5]<<endl;
            //              cout<<"legPee2b0"<<endl;
            //              g.Display(config_2_b0.LegPee,18);

        }

        robot.SetPee(config_2_b0.LegPee,beginMak);
        robot.SetPeb(config_2_b0.BodyPee,beginMak,"213");

        stepCount+=1;

        if (stepCount==param.totalCount)
        {
            //stepCount=0;
            gaitState=GaitState::End;
        }
        return 1;

    case GaitState::End:
    default:
        gaitState=GaitState::None;
        return 0;
    }
}









//    if(isSlopeStopped==false)
//    {
//        auto &robot = static_cast<Robots::RobotBase &>(model);
//        auto &param=static_cast<const WalkGaitParams &>(param_in);
//        static int stepNumFinished=0;
//        static int stepCount=0;
//        // static GaitGenerator g;
//        // static aris::dynamic::FloatMarker beginMak{ robot.ground() };

//        //start a new step
//        if(stepCount==0)
//        {
//            //gaitState=GaitState::Walking;
//            cout<<"a new step begins..."<<"stepCount:"<<stepCount<<endl;
//            cout<<"count "<<param.count<<endl;
//            //ask for map
//            double euler[3];
//            // param.imu_data->toEulBody2Ground(euler,"213");
//            euler[0]=0.1;
//            euler[1]=0.1;
//            euler[2]=0.1;
//            g.UpdateIMU(euler);

//            g.SetWalkParams(param);
//            double currentLegPee2b[18];
//            //             beginMak.setPrtPm(*robot.body().pm());
//            //             beginMak.update();

//            robot.GetPee(currentLegPee2b,robot.body());
//            // g.Display(*robot.body().pm(),16);

//            g.UpdateRobotConfig(currentLegPee2b);
//            g.GaitDetermineNextConfigByVision();
//            //             cout<<"getPeefrommodel"<<endl;
//            //             g.Display(currentLegPee2b,18);
//            //             cout<<"nextPee"<<endl;
//            //             g.Display(g.m_NextConfig_b0.LegPee,18);
//            //             cout<<"nextPee"<<endl;
//            //             g.Display(g.m_CurrentConfig_b0.LegPee,18);
//        }


//        RobotConfig config_2_b0;
//        g.GenerateTraj(stepCount+1,param.totalCount,config_2_b0);
//        if(param.count%300==0)
//        {

//            //              cout<<"(stepCount)/totalCount"<<double((stepCount))/param.totalCount<<endl;
//            //              cout<<"body"<<config_2_b0.BodyPee[0]<<" "<<config_2_b0.BodyPee[1]<<" "<<config_2_b0.BodyPee[2]<<" "<<config_2_b0.BodyPee[3]<<" "<<config_2_b0.BodyPee[4]<<" "<<config_2_b0.BodyPee[5]<<endl;
//            //              cout<<"legPee2b0"<<endl;
//            //              g.Display(config_2_b0.LegPee,18);

//        }

//        robot.SetPeb(config_2_b0.BodyPee,"213");
//        robot.SetPee(config_2_b0.LegPee,robot.body());

//        stepCount+=1;
//        if (stepCount==param.totalCount)
//        {
//            stepNumFinished+=1;
//            stepCount=0;
//        }
//        return -1;

//    }
//    else
//        return 0;
//}


GaitGenerator::GaitGenerator()
{

}

void GaitGenerator::GaitDetermineNextConfigByVision()
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
        //        estSWFoothold_2_b1[i*3+1]=-est_BodyOffset[1]+stdLegPee2B[swingID[i]*3+1];
        estSWFoothold_2_b1[i*3+1]=m_CurrentConfig_b0.LegPee[swingID[i]*3+1];// refer to the current robot elevation
        estSWFoothold_2_b1[i*3+2]=-est_BodyOffset[2]+stdLegPee2B[swingID[i]*3+2]-cos(m_Params.a)*dstraight/2;//relavant to the angle of a
        aris::dynamic::s_pm_dot_pnt(est_TM_b1_2_b0,&estSWFoothold_2_b1[i*3],&estSWFoothold_2_b0[i*3]);
    }

    //find footholds w.r.t. b0 with terrain information
    double SWFoothold_2_b0[9];
    memcpy(SWFoothold_2_b0,estSWFoothold_2_b0,sizeof(SWFoothold_2_b0));
    GetTerrainHeight2b(&SWFoothold_2_b0[0]);
    GetTerrainHeight2b(&SWFoothold_2_b0[3]);
    GetTerrainHeight2b(&SWFoothold_2_b0[6]);
    // cout<<"swingID"<<swingID[0]<<swingID[1]<<swingID[2]<<endl;
    //  cout<<"swing foothold ........."<<endl;
    //  Display(SWFoothold_2_b0,9);

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

    //    cout<<"x y z 2 bo"<<endl;
    //    Display(x1_2_b0,3);
    //    Display(y1_2_b0,3);
    //    Display(z1_2_b0,3);
    //    cout<<endl;

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
    //    Display(BodyPos_2_b1_spCenter,3);

    Body_2_b0[0]=BodyPos_2_b0_spCenter[0]+SPCenter[0];
    Body_2_b0[1]=BodyPos_2_b0_spCenter[1]+SPCenter[1];
    Body_2_b0[2]=BodyPos_2_b0_spCenter[2]+SPCenter[2];
    TM_b1_2_b0[3]=Body_2_b0[0];
    TM_b1_2_b0[7]=Body_2_b0[1];
    TM_b1_2_b0[11]=Body_2_b0[2];


    //4.  update body and leg configuration
    memcpy(&m_NextConfig_b0.BodyPee,Body_2_b0,sizeof(double)*6);

    for(int i=0;i<3;i++)
    {
        memcpy(&m_NextConfig_b0.LegPee[swingID[i]*3],&SWFoothold_2_b0[i*3],sizeof(double)*3);
        memcpy(&m_NextConfig_b0.LegPee[stanceID[i]*3],&m_CurrentConfig_b0.LegPee[stanceID[i]*3],sizeof(double)*3);
    }

    memset(m_NextConfig_b1.BodyPee,0,sizeof(m_NextConfig_b1.BodyPee));
    double TM_b0_2_b1[16];
    aris::dynamic::s_inv_pm(TM_b1_2_b0,TM_b0_2_b1);
    LegsTransform(m_NextConfig_b0.LegPee,TM_b0_2_b1,m_NextConfig_b1.LegPee);

    //    cout<<"estTM B1_2B0"<<endl;
    //    Display(est_TM_b1_2_b0,16);
    //        cout<<"tri center"<<endl;
    //        Display(SPCenter,3);
    //        cout<<"BodyPos_2_b1_spCenter"<<endl;
    //       Display(BodyPos_2_b1_spCenter,3);
    //       cout<<"BodyPos_2_b0_spCenter"<<endl;
    //      Display(BodyPos_2_b0_spCenter,3);
    //      cout<<"Body_2_b0"<<endl;
    //     Display(Body_2_b0,3);

    //    cout<<"TMB1_2_B0"<<endl;
    //    Display(TM_b1_2_b0,16);
    //    cout<<"TMB0_2_B1"<<endl;
    //    Display(TM_b0_2_b1,16);
    //    cout<<"currentlegPee2B0"<<endl;
    //    Display(m_CurrentConfig_b0.LegPee,18);
    //    cout<<"legPee2B0"<<endl;
    //    Display(m_NextConfig_b0.LegPee,18);
    //    cout<<"legPee2B1"<<endl;
    //    Display(m_NextConfig_b1.LegPee,18);

}

void GaitGenerator::GaitDetermineNextConfigByHuman(const double Pitch_2_b0, const double Roll_2_b0)
{
    //robot moves along the -z direction

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
    est_TM_b1_2_b0[7]=dstraight*sin(Pitch_2_b0);

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
        //        estSWFoothold_2_b1[i*3+1]=-est_BodyOffset[1]+stdLegPee2B[swingID[i]*3+1];
        estSWFoothold_2_b1[i*3+1]=m_CurrentConfig_b0.LegPee[swingID[i]*3+1];// refer to the current robot elevation
        estSWFoothold_2_b1[i*3+2]=-est_BodyOffset[2]+stdLegPee2B[swingID[i]*3+2]-cos(m_Params.a)*dstraight/2;//relavant to the angle of a
        aris::dynamic::s_pm_dot_pnt(est_TM_b1_2_b0,&estSWFoothold_2_b1[i*3],&estSWFoothold_2_b0[i*3]);
    }

    //find footholds w.r.t. b0 with terrain information
    double SWFoothold_2_b0[9];
    memcpy(SWFoothold_2_b0,estSWFoothold_2_b0,sizeof(SWFoothold_2_b0));

    //3.  determine bodypos w.r.t. b0
    double TM_b1_2_b0[16];
    double Body_2_b0[6];
    Body_2_b0[3]=m_Params.b;
    Body_2_b0[4]=Pitch_2_b0;
    Body_2_b0[5]=Roll_2_b0;
    aris::dynamic::s_pe2pm(Body_2_b0,TM_b1_2_b0,"213");

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
    //           cout<<"BodyPos_2_b1_spCenter"<<endl;
    //          Display(BodyPos_2_b1_spCenter,3);
    //          cout<<"BodyPos_2_b0_spCenter"<<endl;
    //          Display(BodyPos_2_b0_spCenter,3);
    //    cout<<" BodyPos_2_b0_spCenter"<<endl;
    //    Display(BodyPos_2_b0_spCenter,3);
    //    cout<<" BodyPos_2_b1_spCenter"<<endl;
    //    Display(BodyPos_2_b1_spCenter,3);

    Body_2_b0[0]=BodyPos_2_b0_spCenter[0]+SPCenter[0];
    Body_2_b0[1]=BodyPos_2_b0_spCenter[1]+SPCenter[1];
    Body_2_b0[2]=BodyPos_2_b0_spCenter[2]+SPCenter[2];
    TM_b1_2_b0[3]=Body_2_b0[0];
    TM_b1_2_b0[7]=Body_2_b0[1];
    TM_b1_2_b0[11]=Body_2_b0[2];
    //4.  update body and leg configuration
    memcpy(&m_NextConfig_b0.BodyPee,Body_2_b0,sizeof(double)*6);

    for(int i=0;i<3;i++)
    {
        memcpy(&m_NextConfig_b0.LegPee[swingID[i]*3],&SWFoothold_2_b0[i*3],sizeof(double)*3);
        memcpy(&m_NextConfig_b0.LegPee[stanceID[i]*3],&m_CurrentConfig_b0.LegPee[stanceID[i]*3],sizeof(double)*3);
    }

    memset(m_NextConfig_b1.BodyPee,0,sizeof(m_NextConfig_b1.BodyPee));
    double TM_b0_2_b1[16];
    aris::dynamic::s_inv_pm(TM_b1_2_b0,TM_b0_2_b1);
    LegsTransform(m_NextConfig_b0.LegPee,TM_b0_2_b1,m_NextConfig_b1.LegPee);

    //           cout<<"estTM B1_2B0"<<endl;
    //           Display(est_TM_b1_2_b0,16);
    //               cout<<"tri center"<<endl;
    //               Display(SPCenter,3);
    ////               cout<<"BodyPos_2_b1_spCenter"<<endl;
    ////              Display(BodyPos_2_b1_spCenter,3);
    ////              cout<<"BodyPos_2_b0_spCenter"<<endl;
    ////             Display(BodyPos_2_b0_spCenter,3);
    //             cout<<"Body_2_b0"<<endl;
    //            Display(Body_2_b0,3);

    //           cout<<"TMB1_2_B0"<<endl;
    //           Display(TM_b1_2_b0,16);
    //           cout<<"TMB0_2_B1"<<endl;
    //           Display(TM_b0_2_b1,16);
    //           cout<<"currentlegPee2B0"<<endl;
    //           Display(m_CurrentConfig_b0.LegPee,18);
    //           cout<<"legPee2B0"<<endl;
    //           Display(m_NextConfig_b0.LegPee,18);
    //           cout<<"legPee2B1"<<endl;
    //           Display(m_NextConfig_b1.LegPee,18);
}

void GaitGenerator::UpdateRobotConfig(const double *legPee2b)//legPee2b is got the Robot model
{
    // Initially, ground CS is set on the body Geometric center
    memset(m_CurrentConfig_b0.BodyPee,0,sizeof(double)*6);
    memcpy(m_CurrentConfig_b0.LegPee,legPee2b,sizeof(double)*18);

    memset(&m_CurrentConfig_g.BodyPee[0],0,sizeof(double)*3);
    memcpy(&m_CurrentConfig_g.BodyPee[3],m_EulerAngles,sizeof(double)*3);
    m_CurrentConfig_g.BodyPee[3]=0;

    double TM_b0_2_g[16];
    aris::dynamic::s_pe2pm(m_CurrentConfig_g.BodyPee,TM_b0_2_g,"213");

    LegsTransform(legPee2b,TM_b0_2_g,m_CurrentConfig_g.LegPee);
    double swingid[3];
    memcpy(swingid,swingID,sizeof(double)*3);
    memcpy(swingID,stanceID,sizeof(double)*3);
    memcpy(stanceID,swingid,sizeof(double)*3);
}

void GaitGenerator::GenerateTraj(const int count, const int totalCount, RobotConfig& config_2_b0)
{
    double TM_b1_2_b0[16];
    aris::dynamic::s_pe2pm(m_NextConfig_b0.BodyPee,TM_b1_2_b0,"213");
    //    cout<<"TM_b1_2_b0"<<endl;
    //    Display(TM_b1_2_b0,16);


    double s;
    s=(1-cos(double(count)/totalCount*PI))/2;

    // compute body pee, pos
    if(m_Params.b!=0)
    {
        double YawAxisPos[3];
        GetYawAxis(TM_b1_2_b0,YawAxisPos);
        double TMpos[16];
        RyAlongAxis(m_Params.b*s,YawAxisPos,TMpos);
        config_2_b0.BodyPee[0]=TMpos[3];
        config_2_b0.BodyPee[1]=s*TM_b1_2_b0[7];
        config_2_b0.BodyPee[2]=TMpos[11];
    }
    else
    {
        config_2_b0.BodyPee[0]=s*TM_b1_2_b0[3];
        config_2_b0.BodyPee[1]=s*TM_b1_2_b0[7];
        config_2_b0.BodyPee[2]=s*TM_b1_2_b0[11];
    }

    //angle
    double RotAxis[3];
    double RotAngle;
    TM_2_Rot(TM_b1_2_b0,RotAngle,RotAxis);
    //    cout<<"rot axis"<<endl;
    //    Display(RotAxis,3);
    //    cout<<"rot angle:"<<RotAngle<<endl;

    double TM_2_b0[16];
    Rot_2_TM(s*RotAngle,RotAxis,TM_2_b0);
    TM_2_b0[3]=config_2_b0.BodyPee[0];
    TM_2_b0[7]=config_2_b0.BodyPee[1];
    TM_2_b0[11]=config_2_b0.BodyPee[2];

    aris::dynamic::s_pm2pe(TM_2_b0,config_2_b0.BodyPee,"213");//body ok

    //stance Leg pee
    for (int i=0;i<3;i++)
    {
        memcpy(&config_2_b0.LegPee[stanceID[i]*3],&m_CurrentConfig_b0.LegPee[stanceID[i]*3],sizeof(double)*3);//stancelegs are ok
    }

    //swing Leg Pee
    double swLegPee2b[9];
    double swLegPee2b0[9];

    for(int i=0;i<3;i++)
    {
        TrajEllipsoid(&m_CurrentConfig_b0.LegPee[swingID[i]*3],&m_NextConfig_b1.LegPee[swingID[i]*3],count,totalCount,&swLegPee2b[i*3]);// a problem is here
        aris::dynamic::s_pm_dot_pnt(TM_2_b0,&swLegPee2b[3*i],&swLegPee2b0[3*i]);

        memcpy(&config_2_b0.LegPee[swingID[i]*3],&swLegPee2b0[3*i],sizeof(double)*3);
    }
    //    cout<<"swingLeg2b"<<endl;
    //     Display(swLegPee2b,9);
    ////    cout<<"tm_2_b0"<<endl;
    ////    Display(TM_2_b0,16);
    //    cout<<"legPee2b0"<<endl;
    //    Display(config_2_b0.LegPee,18);
    //     cout<<"bodyPee"<<endl;
    //     Display(config_2_b0.BodyPee,6);

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
    memcpy(&m_Params,&param,sizeof(param));
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

    pos[1]=gridMap[grid[0]][grid[1]];
    // for test
    pos[1]=-0.85;
}
void GaitGenerator::GetBodyOffset(const double pitch, const double roll, double* offset)
{
    // only for test
    offset[0]=0.9*roll;
    offset[1]=0.07;
    offset[2]=-0.8*pitch;


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

void GaitGenerator::Display(const double *vec,int length)
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

void GaitGenerator::GetYawAxis(const double *TM, double *Yaxis)
{
    double Euler[3];
    aris::dynamic::s_pm2pe(TM,Euler,"213");
    double yaw;
    yaw=Euler[3];
    double Pmid[3];//0.5 horizontal displacement
    Pmid[0]=TM[3];
    Pmid[1]=0;
    Pmid[2]=TM[11];
    double D;
    D=norm(Pmid);
    double R;
    R=D/sin(yaw/2);
    double rotAxis[3];
    rotAxis[0]=0;
    rotAxis[2]=0;
    rotAxis[1]=sign(yaw);
    double normDir[3];
    aris::dynamic::s_cro3(rotAxis,Pmid,normDir);
    normalize(normDir);
    Yaxis[0]=Pmid[0]+normDir[0]*R*cos(yaw/2);
    Yaxis[1]=Pmid[1]+normDir[1]*R*cos(yaw/2);
    Yaxis[2]=Pmid[2]+normDir[2]*R*cos(yaw/2);

}
void GaitGenerator::TM_2_Rot(const double *TM, double& theta, double *u)
{
    double q[4];
    q[0]=0.5*sqrt(1+TM[0]+TM[5]+TM[10]);
    q[1]=(TM[9]-TM[6])/(4*q[0]);
    q[2]=(TM[2]-TM[8])/(4*q[0]);
    q[3]=(TM[4]-TM[1])/(4*q[0]);
    theta=acos(q[0])*2;
    if(theta==0)
    {
        u[0]=0;
        u[1]=1;
        u[2]=0;
    }
    else
    {
        u[0]=q[1]/sin(theta/2);
        u[1]=q[2]/sin(theta/2);
        u[2]=q[3]/sin(theta/2);
    }
}
void GaitGenerator::Rot_2_TM(const double theta, const double *u, double *TM)
{
    double q[4];
    q[0]=cos(theta/2);
    q[1]=u[0]*sin(theta/2);
    q[2]=u[1]*sin(theta/2);
    q[3]=u[2]*sin(theta/2);

    TM[0]=q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
    TM[1]=2*q[1]*q[2]-2*q[0]*q[3];
    TM[2]=2*q[1]*q[3]+2*q[0]*q[2];
    TM[4]=2*q[1]*q[2]+2*q[0]*q[3];
    TM[5]=q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3];
    TM[6]=2*q[2]*q[3]-2*q[0]*q[1] ;
    TM[8]=2*q[1]*q[3]-2*q[0]*q[2];
    TM[9]=2*q[2]*q[3]+2*q[0]*q[1] ;
    TM[10]=q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
    TM[15]=1;
}
void GaitGenerator::TrajEllipsoid(const double *p0, const double *p1, const int count, const int totalCount, double *legpos)
{
    double theta;
    theta=PI*(1-cos(double(count)/totalCount*PI))/2;
    double axisShort[3];
    double x[3];
    x[0]=1;
    x[1]=0;
    x[2]=0;
    double p01[3];
    p01[0]=p1[0]-p0[0];
    p01[1]=p1[1]-p0[1];
    p01[2]=p1[2]-p0[2];

    aris::dynamic::s_cro3(x,p01,axisShort);
    axisShort[0]=axisShort[0]/axisShort[1]*m_Params.h;
    axisShort[1]=m_Params.h;
    axisShort[2]=axisShort[2]/axisShort[1]*m_Params.h;

    //    rt_printf("theta %f\n",theta);




    legpos[0]=(p0[0]+p1[0])/2+(p0[0]-p1[0])/2*cos(theta)+axisShort[0]*sin(theta);
    legpos[1]=(p0[1]+p1[1])/2+(p0[1]-p1[1])/2*cos(theta)+axisShort[1]*sin(theta);
    legpos[2]=(p0[2]+p1[2])/2+(p0[2]-p1[2])/2*cos(theta)+axisShort[2]*sin(theta);
}

}



