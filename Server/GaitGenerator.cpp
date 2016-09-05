#include "GaitGenerator.h"
#include <math.h>
#define YawLimit 0.08
#include "rtdk.h"

using namespace std;

namespace VersatileGait
{
//atomic_bool isSlopeStopped(false);


aris::control::Pipe<VersatileGait::ScanningInfo> visionSlopePipe(true);
atomic_bool isScanningFinished{false};
//atomic_bool isUsingGridMap{false};
int FlagV{FlagVision::Free};

float gridMapBuff[400][400];

const int Leg2Force[6]{0,1,2,3,4,5};

const double stdLegPee2B[18]=
{  -0.3,-0.99,-0.55,
   -0.45,-0.99,0,
   -0.3,-0.99,0.55,
   0.3,-0.99,-0.55,
   0.45,-0.99,0,
   0.3,-0.99,0.55
};//change 0.85 to std offset height
//{  -0.3,-0.99,-0.45,
//   -0.45,-0.99,0,
//   -0.45,-0.99,0.65,
//   0.3,-0.99,-0.45,
//   0.45,-0.99,0,
//   0.45,-0.99,0.65
//};//change 0.85 to std offset height
const double stdLegPee2C[18]=
{  -0.3,0,-0.55,
   -0.45,0,0,
   -0.3,0,0.55,
   0.3,0,-0.55,
   0.45,0,0,
   0.3,0,0.55
};//change 0.85 to std offset height
//{  -0.3,0,-0.45,
//   -0.45,0,0,
//   -0.45,0,0.65,
//   0.3,0,-0.45,
//   0.45,0,0,
//   0.45,0,0.65
//};//change 0.85 to std offset height
const double ScrewUpLimit[18]=
{
    1.112,1.128,1.128,
    1.112,1.128,1.128,
    1.112,1.128,1.128,
    1.112,1.128,1.128,
    1.112,1.128,1.128,
    1.112,1.128,1.128,
};

const double ScrewDownLimit[18]=
{
    0.722,0.738,0.738,
    0.722,0.738,0.738,
    0.722,0.738,0.738,
    0.722,0.738,0.738,
    0.722,0.738,0.738,
    0.722,0.738,0.738,
};
const double ScrewMargin{0.01};
const double ForceTDvalue{70};// POSITIVE HERE
const double dutyFactor{0.65};
//const double adjPitch{0.05};


static float gridMap[400][400];
static int gaitState=GaitState::None;
static int gaitCommand=GaitCommand::NoCommand;
//static double pitch_2_b0=0;
//static double roll_2_b0=0;
static double dDist=0;
static double dAngle=0;
static double dLateral=0;
static bool isForceUsed=false;
static bool isIMUUsed=false;
static bool isVisionUsed=false;
static double TDextend=0.06;
static double dPitch=0;
aris::control::Pipe<robotData> logPipe(true);


void startLogDataThread()
{
    static auto logThread = std::thread([]()
    {
        sleep(1);
        //aris::core::logFileNames();

        std::time_t beginTime{};
        time(&beginTime);

        auto timeinfo = localtime(&beginTime);
        char timeCh[1024]={0};

        strftime(timeCh,1024,"_%Y-%m-%d_%H-%M-%S_log.txt",timeinfo);
        std::string filename = "data" + string(timeCh);

        static std::ofstream file;
        file.open(filename);
        cout << "Sleep for a second waiting for something to init\n";


        while(true)
        {

            robotData data;
            logPipe.recvInNrt(data);

            file << std::setprecision(15);
            file<<data.waist<<" ";

            for (int j = 0; j < 3; j++)
            {
                file << data.imu[j] << "   ";
            }

            for (int j = 0; j < 6; j++)
            {
                file << data.bodyPee[j] << "   ";
            }

            for (int j = 0; j < 18; j++)
            {
                file << data.legPee[j] << "   ";
            }
            for (int j = 0; j < 6; j++)
            {
                file << data.force[j] << "   ";
            }
            for (int j = 0; j < 6; j++)
            {
                file << data.legPhase[j] << "   ";
            }



            file << std::endl;

        }
        file.close();
    });


}

void parseGoSlopeVisionFast2(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
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

        else if (i.first == "lateral")
        {
            param.l=stod(i.second);
        }
        else if (i.first == "mode")
        {
            param.m= stoi(i.second);
        }
    }
    msg.copyStruct(param);
    cout<<"parse finished"<<endl;


}

int GoSlopeByVisionFast2(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{

    static GaitGenerator g;

    auto &robot = static_cast<Robots::RobotTypeIII &>(model);
    auto &Param=static_cast<const WalkGaitParams &>(param_in);

    WalkGaitParams param;
    memcpy(&param,&Param,sizeof(param));



    //param.b+=dAngle;
    static aris::dynamic::FloatMarker beginMak{robot.ground()};
    static aris::dynamic::FloatMarker InitMak{robot.body()};

    static int stepNumFinished=0;
    static int stepCount=0;
    static bool isStepFinished=false;

    if(param.count==0)
    {
        rt_printf("reset static variables at start!\n");
        stepNumFinished=0;
        stepCount=0;
        isStepFinished=false;
        dDist=0;
        dAngle=0;
        dLateral=0;
        //        for(int i=0;i<400;i++)
        //            for(int j=0;j<400;j++)
        //            {
        //                gridMap[i][j]=-0.9;
        //               // gridMapBuff[i][j]=-0.9;
        //            }
    }

    static double waistStart;
    static double waistEnd;
    static RobotConfig Config0_2_c0;
    static RobotConfig Config1_2_c0;
    static RobotConfig Config1_2_c1;
    // static RobotConfig Config1_2_b1;
    // static RobotConfig Config1_2_b0;

    static double TM_c0_2_g[16];

    static int swingID[3]{0,2,4};
    static int stanceID[3]{1,5,3};

    static double bodyVelStart[2];// z and x direction
    //  static double bodyVelEnd[2];
    static double bodyAcc[2];
    static double bodyVelDes[2];
    static double bodyVelDes_2_c[2];
    static bool isTD[3]={false,false,false};

    //   static double HeightAdj_c1_2_c0;



    //    rt_printf("count %d\n",param.count);
    //if(param.count%10==0)
    // if(isVisionUsed==true)


    //sending for map update

    //    if(FlagV==FlagVision::Free)
    if(param.count%300==0)
    {
        ScanningInfo sendInfo;
        memset(sendInfo.TM,0,sizeof(double)*16);
        sendInfo.isInit=true;
        VersatileGait::visionSlopePipe.sendToNrt(sendInfo);
        rt_printf("visionCommand Sending...\n");
    }

    if(param.count%100==0)
    {
        if(FlagV==FlagVision::Free)
        {
            FlagV=FlagVision::DataCopying;
            memcpy(gridMap,gridMapBuff,sizeof(float)*400*400);
            FlagV=FlagVision::Free;
            // rt_printf("map buff  is : %f %f\n",gridMapBuff[200][200],gridMapBuff[300][200]);
            //rt_printf("map   is : %f %f\n",gridMap[200][200],gridMap[300][200]);
        }
        else
            rt_printf("vision is scanning ,could not update map...........................................................\n");
    }
    //    if(param.count%100==0)
    //    {
    //        rt_printf("flag %d\n",FlagV);
    //        rt_printf("map buff  is : %f %f\n",gridMapBuff[200][200],gridMapBuff[300][200]);
    //        rt_printf("map   is : %f %f\n",gridMap[200][200],gridMap[300][200]);
    //    }


    switch(gaitState)
    {
    case GaitState::None:
        gaitState=GaitState::Walking;
        return 1;

    case GaitState::Walking:

        int swingCount;
        int stanceCount;
        swingCount=param.totalCount*2*(1-dutyFactor);
        stanceCount=param.totalCount*2*dutyFactor;

        if(stepCount==0) //update vision and imu to update this configuration and compute the next configuration
        {
            param.d+=-dDist;
            param.l+=-dLateral;
            param.b+=dAngle;

            rt_printf("/////////////////current step started!//////////////////////////////\n");


            rt_printf("A new step begins...swingID %d %d %d\n",swingID[0],swingID[1],swingID[2]);
            rt_printf("Param!!!!!!\nwalk d %f,\n walk lateral %f,\n walk b %f \n pitch p %f\n",param.d,param.l,param.b,dPitch);

            double euler[3];
            memset(euler,0,sizeof(double)*3);
            if(isIMUUsed==true)
                param.imu_data->toEulBody2Ground(euler,"231");

            rt_printf("231 YAW ROLL PITCH,imu_data:%f %f %f corrected %f %f %f\n",euler[0],euler[1],euler[2],asin(sin(euler[0])),asin(sin(euler[1])),asin(sin(euler[2])));

            euler[0]=0;// yaw being zero
            euler[1]=asin(sin(euler[1]));
            euler[2]=asin(sin(euler[2]));



            //in this scheduling scheme, the body is set on the waist
            RobotConfig Config0_2_b0;
            robot.GetWa(waistStart);
            waistStart=asin(sin(waistStart));
            double b0_2_s0[6];//"231"
            memset(b0_2_s0,0,sizeof(double)*6);
            b0_2_s0[5]=waistStart;
            double TM_b0_2_s0[16];
            aris::dynamic::s_pe2pm(b0_2_s0,TM_b0_2_s0,"231");
            double TM_b0_2_g[16];
            aris::dynamic::s_pm_dot_pm(*robot.body().pm(),TM_b0_2_s0,TM_b0_2_g);

            //            rt_printf("b0_2_s0 %f %f %f %f %f %f\n",b0_2_s0[0],b0_2_s0[1],b0_2_s0[2],b0_2_s0[3],b0_2_s0[4],b0_2_s0[5]);
            //            rt_printf("TM_b0_2_s0\n");
            //            for(int i=0;i<4;i++)
            //                rt_printf(" %f %f %f %f\n",TM_b0_2_s0[i*4+0],TM_b0_2_s0[i*4+1],TM_b0_2_s0[i*4+2],TM_b0_2_s0[i*4+3]);

            beginMak.setPrtPm(TM_b0_2_g);
            beginMak.update();

            robot.GetPee(Config0_2_b0.LegPee,beginMak);
            double TM_c0_2_b0[16];
            // double TM_bo_2_c0[16];
            g.GetLeg2bodyFromLegs(Config0_2_b0.LegPee,TM_c0_2_b0);
            //            rt_printf("TM_c0_2_b0\n");
            //            for (int i=0;i<4;i++)
            //                rt_printf("%f %f %f %f\n",TM_c0_2_b0[i*4],TM_c0_2_b0[i*4+1],TM_c0_2_b0[i*4+2],TM_c0_2_b0[i*4+3]);
            //aris::dynamic::s_inv_pm(TM_c0_2_b0,TM_b0_2_c0);
            memset(Config0_2_b0.BodyPee,0,sizeof(double)*6);
            aris::dynamic::s_pm_dot_pm(TM_b0_2_g,TM_c0_2_b0,TM_c0_2_g);

            beginMak.setPrtPm(TM_c0_2_g);
            beginMak.update();
            robot.GetPeb(Config0_2_c0.BodyPee,beginMak,"231");
            Config0_2_c0.BodyPee[5]+=waistStart;

            robot.GetPee(Config0_2_c0.LegPee,beginMak);
            //            rt_printf("bodyInit 2 c0 %f %f %f %f %f %f\n",Config0_2_c0.BodyPee[0],Config0_2_c0.BodyPee[1],Config0_2_c0.BodyPee[2],Config0_2_c0.BodyPee[3],Config0_2_c0.BodyPee[4],Config0_2_c0.BodyPee[5]);

            //            rt_printf("LegPee 2 c0\n");
            //            for(int i=0;i<6;i++)
            //            {
            //                rt_printf("%f %f %f\n",Config0_2_c0.LegPee[3*i],Config0_2_c0.LegPee[3*i+1],Config0_2_c0.LegPee[3*i+2]);

            //            }
            //       Config0_2_c0.BodyPee[3]=asin(sin(Config0_2_c0.BodyPee[3]));
            //       Config0_2_c0.BodyPee[5]=asin(sin(Config0_2_c0.BodyPee[5]));


            // 2.set walking params in c0 coordinate system
            bodyVelDes_2_c[0]=-param.d/param.totalCount*1000;
            bodyVelDes_2_c[1]=-param.l/param.totalCount*1000;

            bodyVelDes[0]=bodyVelDes_2_c[0]*cos(param.b)-bodyVelDes_2_c[1]*sin(param.b);//end velocity of c to c0
            bodyVelDes[1]=bodyVelDes_2_c[1]*cos(param.b)+bodyVelDes_2_c[0]*sin(param.b);

            double t_spend=stanceCount/double(1000);
            bodyAcc[0]=(bodyVelDes[0]-bodyVelStart[0])/t_spend;
            bodyAcc[1]=(bodyVelDes[1]-bodyVelStart[1])/t_spend;



            rt_printf("stance count %d\n",stanceCount);
            //rt_printf("bodyVelDes_2_c  %f %f, vel end %f %f\n",bodyVelStart[0],bodyVelStart[1],bodyVelDes[0],bodyVelDes[1]);
            rt_printf("vel start  z %f x %f, vel end  z %f x %f\n",bodyVelStart[0],bodyVelStart[1],bodyVelDes[0],bodyVelDes[1]);
            rt_printf("acc %f %f\n",bodyAcc[0],bodyAcc[1]);


            //param d is negative pointing to -z,dstraight is positive
            //param l is negative pointing to -x,lstraight is positive

            double dstraight;
            double lstraight;

            dstraight=t_spend*bodyVelStart[0]+0.5*bodyAcc[0]*t_spend*t_spend;
            lstraight=t_spend*bodyVelStart[1]+0.5*bodyAcc[1]*t_spend*t_spend;

            rt_printf("c displacement for this step forward: %f, left: %f\n",dstraight,lstraight);

            // first, use only the current ground, in which c0  coicide with the assumd ground
            //body velocity

            \
            double TM_c1_2_c0[16];
            double TM_c0_2_c1[16];

            double c1_2_c0[6];
            c1_2_c0[0]=lstraight;
            c1_2_c0[1]=0;
            c1_2_c0[2]=dstraight;
            c1_2_c0[3]=param.b;
            c1_2_c0[4]=0;
            c1_2_c0[5]=0;


            //            if(dPitch!=0 )
            //            {
            //                c1_2_c0[5]=dPitch;
            //                dPitch=0;
            //             }

            aris::dynamic::s_pe2pm(c1_2_c0,TM_c1_2_c0,"231");
            aris::dynamic::s_inv_pm(TM_c1_2_c0,TM_c0_2_c1);



            //rt_printf("isVisionUsed:%d\n",int(isVisionUsed));

            ////   second, could get vision to TM_visTerran_2_c0//////////////////////////////////////

            // this only differenct between vision and noraml is the pitch and roll between c1 and c0
            if(isVisionUsed==true&&dstraight>0)//atan(l/d) belongs to [-pi/2,pi/2]
            {

                //isUsingGridMap=true;
                rt_printf("From Vision:using vision map!!!!!!!!\n");


                double theta[6];
                theta[3]=atan(lstraight/dstraight);
                double thetaTM[16];
                aris::dynamic::s_pe2pm(theta,thetaTM,"213");


                double stdTriangle[9] //has some relationship with the kinect installation !!!
                {0.3,0,0.7,
                    -0.3,0,0.7,
                    0, 0, 1};
                double terrainTriangle[9];



                aris::dynamic::s_pm_dot_v3(thetaTM,&stdTriangle[0],&terrainTriangle[0]);
                aris::dynamic::s_pm_dot_v3(thetaTM,&stdTriangle[3],&terrainTriangle[3]);
                aris::dynamic::s_pm_dot_v3(thetaTM,&stdTriangle[6],&terrainTriangle[6]);

                double terrainTriangle_2_c0[9];
                double TM_b0_2_c0[16];
                aris::dynamic::s_inv_pm(TM_c0_2_b0,TM_b0_2_c0);
                for(int i=0;i<3;i++)
                {
                    g.GetTerrainHeight2b(&terrainTriangle[i*3]);
                    aris::dynamic::s_pm_dot_pnt(TM_b0_2_c0,&terrainTriangle[3*i],&terrainTriangle_2_c0[3*i]);
                }

                //                rt_printf("TM_c0_2_b0\n");
                //                for(int i=0;i<4;i++)
                //                    rt_printf(" %f %f %f %f\n",TM_c0_2_b0[i*4+0],TM_c0_2_b0[i*4+1],TM_c0_2_b0[i*4+2],TM_c0_2_b0[i*4+3]);

                //                rt_printf("TM_b0_2_c0\n");
                //                for(int i=0;i<4;i++)
                //                    rt_printf(" %f %f %f %f\n",TM_b0_2_c0[i*4+0],TM_b0_2_c0[i*4+1],TM_b0_2_c0[i*4+2],TM_b0_2_c0[i*4+3]);

                //                for (int i=0;i<3;i++)
                //                    rt_printf("terrainTriangle_2_c0 %f %f %f\n",terrainTriangle_2_c0[i*3],terrainTriangle_2_c0[i*3+1],terrainTriangle_2_c0[i*3+2]);

                double est_TM_c1_2_c0[16];

                g.GetTri2bodyFromTri(terrainTriangle_2_c0,param.b,est_TM_c1_2_c0);
                double est_euler_c1_2_c0[6];
                aris::dynamic::s_pm2pe(est_TM_c1_2_c0,est_euler_c1_2_c0,"231");

                est_euler_c1_2_c0[3]=asin(sin(est_euler_c1_2_c0[3]));//[0,pi/2]
                est_euler_c1_2_c0[4]=asin(sin(est_euler_c1_2_c0[4]));//[0,pi/2]
                est_euler_c1_2_c0[5]=asin(sin(est_euler_c1_2_c0[5]));

                //half the euler angles
                //here, it is firstly trans then rotate, we can actally change it to firstly rotate then trans, but acc could be hard
                c1_2_c0[0]=lstraight;//x
                c1_2_c0[1]=0;
                c1_2_c0[2]=dstraight;
                c1_2_c0[3]=param.b;//which==est_euler_c1_2_c0[3];// euqals to param.b
                c1_2_c0[4]=0;
                c1_2_c0[5]=0;
                rt_printf("theta walking direction %f\n",theta[3]);

                if(abs(theta[3])<PI/3)
                {
                    c1_2_c0[4]=0.5*est_euler_c1_2_c0[4]*(PI/3-abs(theta[3])); //believe region, [-pi/3,pi/3]
                    c1_2_c0[5]=0.5*est_euler_c1_2_c0[5]*(PI/3-abs(theta[3]));
                }


                aris::dynamic::s_pe2pm(c1_2_c0,TM_c1_2_c0,"231");
                rt_printf("From Vision: c1_2_c0 euler angle from vision 231: %f (yaw) %f %f\n",c1_2_c0[3],c1_2_c0[4],c1_2_c0[5]);

                aris::dynamic::s_inv_pm(TM_c1_2_c0,TM_c0_2_c1);
                //isUsingGridMap=false;

            }
            ////////////////////***********///////////////vision////////////
            rt_printf("TM_c1_2_c0\n");
            for(int i=0;i<4;i++)
                rt_printf(" %f %f %f %f\n",TM_c1_2_c0[i*4+0],TM_c1_2_c0[i*4+1],TM_c1_2_c0[i*4+2],TM_c1_2_c0[i*4+3]);


            //stance legs 2 c0
            for(int i=0;i<3;i++)
                memcpy(&Config1_2_c0.LegPee[stanceID[i]*3],&Config0_2_c0.LegPee[stanceID[i]*3],sizeof(double)*3);



            //swing legs 2 c0
            //            double SW_2_c1[9];
            //            double SW_2_c0[9];

            //            for(int i=0;i<3;i++)
            //            {
            //                memcpy(&SW_2_c1[i*3],&stdLegPee2C[swingID[i]*3],sizeof(double)*3);
            //                SW_2_c1[i*3]=SW_2_c1[i*3]+lstraight/2;
            //                SW_2_c1[i*3+1]=SW_2_c1[i*3+1];
            //                SW_2_c1[i*3+2]=SW_2_c1[i*3+2]+dstraight/2;
            //                aris::dynamic::s_pm_dot_pnt(TM_c1_2_c0,&SW_2_c1[i*3],&SW_2_c0[i*3]);
            //                memcpy(&Config1_2_c0.LegPee[swingID[i]*3],&SW_2_c0[i*3],sizeof(double)*3);
            //            }



            //seperate the leg trajectory planning

            double SW_2_c11[3];
            double SW_2_c12[3];
            double SW_2_c13[3];
            double TM_c11_2_c0[16];
            double TM_c12_2_c0[16];
            double TM_c13_2_c0[16];

            //[0 2 4]   [1 5 3]

            if(swingID[0]==0)
            {
                //for leg  0  -->(3 1 4)
                double triangle1[9];

                memcpy(&triangle1[0],&Config0_2_c0.LegPee[3*3],sizeof(double)*3);
                memcpy(&triangle1[3],&Config0_2_c0.LegPee[1*3],sizeof(double)*3);
                memcpy(&triangle1[6],&Config0_2_c0.LegPee[4*3],sizeof(double)*3);
                g.GetTri2bodyFromTri(triangle1,param.b,TM_c11_2_c0);

                TM_c11_2_c0[3]=lstraight;
                TM_c11_2_c0[7]=0;
                TM_c11_2_c0[11]=dstraight;

                //                if(dPitch!=0)
                //                {
                //                    double c11_2_c0[6];
                //                    aris::dynamic::s_pm2pe(TM_c11_2_c0,c11_2_c0,"231");
                //                    c11_2_c0[5]+=dPitch;
                //                    aris::dynamic::s_pe2pm(c11_2_c0,TM_c11_2_c0,"231");
                //                }

                SW_2_c11[0]=stdLegPee2C[0*3]+lstraight/2;
                SW_2_c11[1]=stdLegPee2C[0*3+1];
                SW_2_c11[2]=stdLegPee2C[0*3+2]+dstraight/2;

                aris::dynamic::s_pm_dot_pnt(TM_c11_2_c0, SW_2_c11,&Config1_2_c0.LegPee[0*3]);
                //for leg 2 --> (5 1 4)
                double triangle2[9];
                memcpy(&triangle2[0],&Config0_2_c0.LegPee[5*3],sizeof(double)*3);
                memcpy(&triangle2[3],&Config0_2_c0.LegPee[1*3],sizeof(double)*3);
                memcpy(&triangle2[6],&Config0_2_c0.LegPee[4*3],sizeof(double)*3);
                g.GetTri2bodyFromTri(triangle2,param.b,TM_c12_2_c0);

                TM_c12_2_c0[3]=lstraight;
                TM_c12_2_c0[7]=0;
                TM_c12_2_c0[11]=dstraight;

                //                if(dPitch!=0)
                //                {
                //                    double c12_2_c0[6];
                //                    aris::dynamic::s_pm2pe(TM_c12_2_c0,c12_2_c0,"231");
                //                    c12_2_c0[5]+=dPitch;
                //                    aris::dynamic::s_pe2pm(c12_2_c0,TM_c12_2_c0,"231");
                //                }

                SW_2_c12[0]=stdLegPee2C[2*3]+lstraight/2;
                SW_2_c12[1]=stdLegPee2C[2*3+1];
                SW_2_c12[2]=stdLegPee2C[2*3+2]+dstraight/2;

                aris::dynamic::s_pm_dot_pnt(TM_c12_2_c0, SW_2_c12,&Config1_2_c0.LegPee[2*3]);

                //for leg 4, same with the body
                memcpy(TM_c13_2_c0,TM_c1_2_c0,sizeof(double)*16);

                SW_2_c13[0]=stdLegPee2C[4*3]+lstraight/2;
                SW_2_c13[1]=stdLegPee2C[4*3+1];
                SW_2_c13[2]=stdLegPee2C[4*3+2]+dstraight/2;

                aris::dynamic::s_pm_dot_pnt(TM_c13_2_c0, SW_2_c13,&Config1_2_c0.LegPee[4*3]);

            }
            else
            {
                //for leg  3  -->(0 1 4)
                double triangle1[9];

                memcpy(&triangle1[0],&Config0_2_c0.LegPee[0*3],sizeof(double)*3);
                memcpy(&triangle1[3],&Config0_2_c0.LegPee[1*3],sizeof(double)*3);
                memcpy(&triangle1[6],&Config0_2_c0.LegPee[4*3],sizeof(double)*3);
                g.GetTri2bodyFromTri(triangle1,param.b,TM_c11_2_c0);

                TM_c11_2_c0[3]=lstraight;
                TM_c11_2_c0[7]=0;
                TM_c11_2_c0[11]=dstraight;

                //                if(dPitch!=0)
                //                {
                //                    double c11_2_c0[6];
                //                    aris::dynamic::s_pm2pe(TM_c11_2_c0,c11_2_c0,"231");
                //                    c11_2_c0[5]+=dPitch;
                //                    aris::dynamic::s_pe2pm(c11_2_c0,TM_c11_2_c0,"231");
                //                }

                SW_2_c11[0]=stdLegPee2C[3*3]+lstraight/2;
                SW_2_c11[1]=stdLegPee2C[3*3+1];
                SW_2_c11[2]=stdLegPee2C[3*3+2]+dstraight/2;

                aris::dynamic::s_pm_dot_pnt(TM_c11_2_c0, SW_2_c11,&Config1_2_c0.LegPee[3*3]);

                //for leg 5 --> (2 1 4)
                double triangle2[9];
                memcpy(&triangle2[0],&Config0_2_c0.LegPee[2*3],sizeof(double)*3);
                memcpy(&triangle2[3],&Config0_2_c0.LegPee[1*3],sizeof(double)*3);
                memcpy(&triangle2[6],&Config0_2_c0.LegPee[4*3],sizeof(double)*3);
                g.GetTri2bodyFromTri(triangle2,param.b,TM_c12_2_c0);

                TM_c12_2_c0[3]=lstraight;
                TM_c12_2_c0[7]=0;
                TM_c12_2_c0[11]=dstraight;


                //                if(dPitch!=0)
                //                {
                //                    double c12_2_c0[6];
                //                    aris::dynamic::s_pm2pe(TM_c12_2_c0,c12_2_c0,"231");
                //                    c12_2_c0[5]+=dPitch;
                //                    aris::dynamic::s_pe2pm(c12_2_c0,TM_c12_2_c0,"231");
                //                }

                SW_2_c12[0]=stdLegPee2C[5*3]+lstraight/2;
                SW_2_c12[1]=stdLegPee2C[5*3+1];
                SW_2_c12[2]=stdLegPee2C[5*3+2]+dstraight/2;

                aris::dynamic::s_pm_dot_pnt(TM_c12_2_c0, SW_2_c12,&Config1_2_c0.LegPee[5*3]);

                //for leg 1, same with the body
                memcpy(TM_c13_2_c0,TM_c1_2_c0,sizeof(double)*16);

                SW_2_c13[0]=stdLegPee2C[1*3]+lstraight/2;
                SW_2_c13[1]=stdLegPee2C[1*3+1];
                SW_2_c13[2]=stdLegPee2C[1*3+2]+dstraight/2;

                aris::dynamic::s_pm_dot_pnt(TM_c13_2_c0, SW_2_c13,&Config1_2_c0.LegPee[1*3]);

            }

            rt_printf("TM_c11_2_c0\n");
            for(int i=0;i<4;i++)
                rt_printf(" %f %f %f %f\n",TM_c11_2_c0[i*4+0],TM_c11_2_c0[i*4+1],TM_c11_2_c0[i*4+2],TM_c11_2_c0[i*4+3]);

            rt_printf("TM_c12_2_c0\n");
            for(int i=0;i<4;i++)
                rt_printf(" %f %f %f %f\n",TM_c12_2_c0[i*4+0],TM_c12_2_c0[i*4+1],TM_c12_2_c0[i*4+2],TM_c12_2_c0[i*4+3]);

            rt_printf("TM_c13_2_c0\n");
            for(int i=0;i<4;i++)
                rt_printf(" %f %f %f %f\n",TM_c13_2_c0[i*4+0],TM_c13_2_c0[i*4+1],TM_c13_2_c0[i*4+2],TM_c13_2_c0[i*4+3]);





            //waist
            //            double absb0_2_g[16];
            //            double b0_2_g[6];
            //            memcpy(&Euler0_2_g[3],euler,sizeof(double)*3);//231 euler angles
            //            aris::dynamic::s_pe2pm(Euler0_2_g,absb0_2_g,"231");

            double bo_2_g0[6];
            memcpy(&bo_2_g0[3],euler,sizeof(double)*3);
            double TM_b0_2_g0[16];
            double TM_c0_2_g0[16];
            double TM_c1_2_g0[16];
            aris::dynamic::s_pe2pm(bo_2_g0,TM_b0_2_g0,"231");
            aris::dynamic::s_pm_dot_pm(TM_b0_2_g0,TM_c0_2_b0,TM_c0_2_g0);
            aris::dynamic::s_pm_dot_pm(TM_c0_2_g0,TM_c1_2_c0,TM_c1_2_g0);

            double c1_2_g0[6];
            aris::dynamic::s_pm2pe(TM_c1_2_g0,c1_2_g0,"231");
            //rt_printf("c1_2_g0  %f %f %f \n",b1_2_g0[3],b1_2_g0[4],b1_2_g0[5]);

            waistEnd=asin(sin(c1_2_g0[5]));//+adjPitch;//decrease the angle


            // body 2 c0  -->ok for RobotIX
            double body_2_c1[6];
            double body_2_c0[6];
            double bodyOffset[3];
            g.GetBodyOffsetRobotIX(c1_2_g0[5],c1_2_g0[4],bodyOffset);//here use the last imu value to adjust body , or else use the next imu value to do this.
            body_2_c1[0]=0+bodyOffset[0];//+offset
            body_2_c1[1]=-stdLegPee2B[1]+bodyOffset[1];//0.85+offset
            body_2_c1[2]=0+bodyOffset[2];//+offset
            body_2_c1[3]=0;
            body_2_c1[4]=0;
            body_2_c1[5]=0;//-adjPitch;


            aris::dynamic::s_pm_dot_pnt(TM_c1_2_c0,body_2_c1,body_2_c0);
            body_2_c0[3]=c1_2_c0[3];
            body_2_c0[4]=c1_2_c0[4];
            body_2_c0[5]=c1_2_c0[5];
            memcpy(&Config1_2_c0.BodyPee,body_2_c0,sizeof(double)*6);







            memcpy(Config1_2_c1.BodyPee,body_2_c1,sizeof(double)*6);
            g.LegsTransform(Config1_2_c0.LegPee,TM_c0_2_c1,Config1_2_c1.LegPee);

            rt_printf("bodyEnd 2 c0 %f %f %f %f %f %f\n",Config1_2_c0.BodyPee[0],Config1_2_c0.BodyPee[1],Config1_2_c0.BodyPee[2],Config1_2_c0.BodyPee[3],Config1_2_c0.BodyPee[4],Config1_2_c0.BodyPee[5]);

            //            rt_printf("LegPeeStart 2 c0\n");
            //            for(int i=0;i<6;i++)
            //            {
            //                rt_printf("%f %f %f\n",Config0_2_c0.LegPee[3*i],Config0_2_c0.LegPee[3*i+1],Config0_2_c0.LegPee[3*i+2]);

            //            }

            rt_printf("LegPeeEnd 2 c0\n");
            for(int i=0;i<6;i++)
            {
                rt_printf("%f %f %f\n",Config1_2_c0.LegPee[3*i],Config1_2_c0.LegPee[3*i+1],Config1_2_c0.LegPee[3*i+2]);

            }

            //            rt_printf("LegPeeEnd 2 c1\n");
            //            for(int i=0;i<6;i++)
            //            {
            //                rt_printf("%f %f %f\n",Config1_2_c1.LegPee[3*i],Config1_2_c1.LegPee[3*i+1],Config1_2_c1.LegPee[3*i+2]);

            //            }
            rt_printf("waist Angle %f\n",waistStart);

            rt_printf("waist End %f\n",waistEnd);




        }


        //generate trajectory for each ms

        RobotConfig config_2_c0;
        double waistAngle;
        //bodypee angle  2 c0
        double s;
        s=(1-cos(double(stepCount+1)/stanceCount*PI))/2;
        //        double body0_2_c0[6];
        //        memcpy(body0_2_c0,Config0_2_c0.BodyPee,sizeof(double)*6);
        //        body0_2_c0[3]=asin(sin(body0_2_c0[3]));
        //        body0_2_c0[5]=asin(sin(body0_2_c0[5]));
        //        double body1_2_c0[6];
        //        memcpy(body1_2_c0,Config1_2_c0.BodyPee,sizeof(double)*6);
        //        body1_2_c0[3]=asin(sin(body1_2_c0[3]));
        //        body1_2_c0[5]=asin(sin(body1_2_c0[5]));
        //        for(int i=3;i<6;i++)//euler interpolation on a plane
        //        {
        //            config_2_c0.BodyPee[i]=body0_2_c0[i]+s*(body1_2_c0[i]-body0_2_c0[i]);
        //        }

        double quat_body0_2_c0[7];//quaternion representation
        double quat_body1_2_c0[7];
        double quat_body_2_c0[7];
        memset(quat_body_2_c0,0,sizeof(double)*7);
        aris::dynamic::s_pe2pq(Config0_2_c0.BodyPee,quat_body0_2_c0,"231");
        aris::dynamic::s_pe2pq(Config1_2_c0.BodyPee,quat_body1_2_c0,"231");

        for (int i=3;i<7;i++)
        {
            quat_body_2_c0[i]=quat_body0_2_c0[i]+s*(quat_body1_2_c0[i]-quat_body0_2_c0[i]);
        }
        g.qautNormalize(&quat_body_2_c0[3]);
        aris::dynamic::s_pq2pe(quat_body_2_c0,config_2_c0.BodyPee,"231");
        config_2_c0.BodyPee[3]=asin(sin(config_2_c0.BodyPee[3]));
        config_2_c0.BodyPee[4]=asin(sin(config_2_c0.BodyPee[4]));
        config_2_c0.BodyPee[5]=asin(sin(config_2_c0.BodyPee[5]));


        //waist angle
        waistAngle=waistStart+s*(waistEnd-waistStart);

        //bodypee_pos_2_c0=c_2_c0+body_2_c(which is also the offset variation)

        double t;
        t=double(stepCount+1)/1000;

        double c_2_c0[3];

        c_2_c0[0]=t*bodyVelStart[1]+0.5*bodyAcc[1]*t*t;//x
        c_2_c0[1]=0;
        c_2_c0[2]=t*bodyVelStart[0]+0.5*bodyAcc[0]*t*t;//z

        config_2_c0.BodyPee[0]=c_2_c0[0]+s*(Config1_2_c1.BodyPee[0]-Config0_2_c0.BodyPee[0])+Config0_2_c0.BodyPee[0];
        config_2_c0.BodyPee[1]=c_2_c0[1]+s*(Config1_2_c1.BodyPee[1]-Config0_2_c0.BodyPee[1])+Config0_2_c0.BodyPee[1];
        config_2_c0.BodyPee[2]=c_2_c0[2]+s*(Config1_2_c1.BodyPee[2]-Config0_2_c0.BodyPee[2])+Config0_2_c0.BodyPee[2];
        if(stepCount%200==0)
        {
            // rt_printf("body %f %f %f %f %f %f\n",config_2_c0.BodyPee[0],config_2_c0.BodyPee[1],config_2_c0.BodyPee[2],config_2_c0.BodyPee[3],config_2_c0.BodyPee[4],config_2_c0.BodyPee[5]);
        }


        //swing Leg Pee 2 c0
        double swLegPee2c0[9];

        if(stepCount+1<=swingCount)
        {
            //normal traj
            for(int i=0;i<3;i++)
            {
                g.TrajEllipsoid(&Config0_2_c0.LegPee[swingID[i]*3],&Config1_2_c0.LegPee[swingID[i]*3],stepCount+1,swingCount,param.h,&swLegPee2c0[i*3]);
            }
        }
        else
        {
            memcpy(&swLegPee2c0[0],&Config1_2_c0.LegPee[swingID[0]*3],sizeof(double)*3);
            memcpy(&swLegPee2c0[3],&Config1_2_c0.LegPee[swingID[1]*3],sizeof(double)*3);
            memcpy(&swLegPee2c0[6],&Config1_2_c0.LegPee[swingID[2]*3],sizeof(double)*3);
        }





        //using force
        static double swTD_2_c0[9];

        static bool isStepForceUsed=false;
        if(stepCount==0)
        {
            isStepForceUsed=isForceUsed;
            //rt_printf("force usage updated!\n");

        }



        if(isStepForceUsed==true)
        {
            if(stepCount%100==0)
            {
                // rt_printf("force used!\n");
            }
            bool isInTrans[6];
            double force[6];


            //force judgement
            for(int i=0;i<6;i++)
            {
                force[i]=param.force_data->at(Leg2Force[i]).Fz;
            }

            for (int i=0;i<6;i++)
            {
                //            if(force[i]<-50&&force[i]>-200)
                if(force[i]<-ForceTDvalue)
                    isInTrans[i]=true;
                else
                    isInTrans[i]=false;
            }

            // enlong the swing leg for touching down
            int extraCount=stanceCount-swingCount;
            if(stepCount+1>swingCount)
            {
                // 5cm in 2s
                double new_s=(1-cos(double(stepCount+1-swingCount)/extraCount*PI))/2;
                for(int i=0;i<3;i++)
                {
                    swLegPee2c0[3*i]=Config1_2_c0.LegPee[3*swingID[i]];
                    swLegPee2c0[3*i+1]=Config1_2_c0.LegPee[3*swingID[i]+1]-new_s*TDextend;//y direction enlong
                    swLegPee2c0[3*i+2]=Config1_2_c0.LegPee[3*swingID[i]+2];
                }
                //memcpy(config_2_c0.BodyPee,Config1_2_c0.BodyPee,sizeof(double)*6);
            }

            for(int i=0;i<3;i++)
            {
                if(stepCount+1>swingCount*2/3)
                    if(isInTrans[swingID[i]]==true&&isTD[i]==false)
                    {// record td position
                        isTD[i]=true;
                        memcpy(&swTD_2_c0[i*3],&swLegPee2c0[i*3],sizeof(double)*3);
                        rt_printf("leg %d\n touch down!\n",swingID[i]);
                    }

                if(isTD[i]==true)
                {
                    memcpy(&swLegPee2c0[i*3],&swTD_2_c0[i*3],sizeof(double)*3);
                }
                //                if(isTD[i]==false&&stepCount+1==stanceCount)
                //                {
                //                    memcpy(&swTD_2_c0[i*3],&swLegPee2c0[i*3],sizeof(double)*3);

                //                }
            }

        }
        else
        {
            if(stepCount+1==swingCount)
            {
                isTD[0]=true;
                isTD[1]=true;
                isTD[2]=true;
            }


        }

        if(stepCount+1==stanceCount)
        {
            isTD[0]=false;
            isTD[1]=false;
            isTD[2]=false;
            // if leg prelongs, then next step c1 should go down, on the contrary, if leg td ealier, then the next step should go up. used for changement of terrain
            //HeightAdj_c1_2_c0=(swTD_2_c0[1]-Config1_2_c0.LegPee[3*swingID[0]+1])+(swTD_2_c0[4]-Config1_2_c0.LegPee[3*swingID[1]+1])+(swTD_2_c0[7]-Config1_2_c0.LegPee[3*swingID[2]+1]);
            //HeightAdj_c1_2_c0=HeightAdj_c1_2_c0/3;//negative if leg prelongs(going down),positive if leg td early(going up).
            isStepFinished=true;
        }


        for (int i=0;i<3;i++)
        {
            memcpy(&config_2_c0.LegPee[stanceID[i]*3],&Config0_2_c0.LegPee[stanceID[i]*3],sizeof(double)*3);//stancelegs are ok
            memcpy(&config_2_c0.LegPee[swingID[i]*3],&swLegPee2c0[3*i],sizeof(double)*3);
        }


        //        if(stepCount%100==0)
        //        {
        //            cout<<"count"<<stepCount<<endl;

        //            cout<<" Config current_2_c0, body"<<endl;
        //            g.Display(config_2_c0.BodyPee,6);
        //            cout<<" Config current_2_c0, leg"<<endl;
        //            g.Display(config_2_c0.LegPee,18);
        //        }


        //       // isStepFinished=g.GenerateTraj(stepCount+1,param.totalCount,param,config_2_b0);
        //        beginMak.setPrtPm(TM_c0_2_g);
        //        beginMak.update();
        //        robot.GetPeb(Config0_2_c0.BodyPee,beginMak,"213");
        //        robot.GetPee(Config0_2_c0.LegPee,beginMak);

        double bodyPeeForRobotIX[6];
        memcpy(bodyPeeForRobotIX,config_2_c0.BodyPee,sizeof(double)*6);
        bodyPeeForRobotIX[5]-=waistAngle;

        robot.SetPeb(bodyPeeForRobotIX,beginMak,"231");
        robot.SetWa(waistAngle);
        robot.SetPee(config_2_c0.LegPee,beginMak);


        //length limitation


        double ScrewIn[18];double ScrewInFinal[18];
        memset(ScrewIn,0,sizeof(double)*18);

        robot.GetPin(ScrewIn);

        memcpy(ScrewInFinal,ScrewIn,sizeof(double)*18);
        bool isReset;
        isReset=SetScrewLimits(ScrewIn,ScrewInFinal);

//                //for test
//                isReset=false;

        if(isReset==true)
        {
            robot.SetPin(ScrewInFinal);
        }

        int in_2_count;
        in_2_count=11556181;

        //        for(int i=0;i<6;i++)
        //            rt_printf("ScrewIn %f %f %f\n",ScrewIn[i*3],ScrewIn[i*3+1],ScrewIn[i*3+2]);
        //        for(int i=0;i<6;i++)
        //            rt_printf("ScrewFinal %f %f %f\n",ScrewInFinal[i*3],ScrewInFinal[i*3+1],ScrewInFinal[i*3+2]);
        //        for(int i=0;i<6;i++)
        //            rt_printf("Count %f %f %f\n",in_2_count*ScrewInFinal[i*3],in_2_count*ScrewInFinal[i*3+1],in_2_count*ScrewInFinal[i*3+2]);


        //        //

        stepCount+=1;

        if (isStepFinished==true)
        {
            int swingID_rec[3];
            memcpy(swingID_rec,swingID,sizeof(int)*3);
            memcpy(swingID,stanceID,sizeof(int)*3);
            memcpy(stanceID,swingID_rec,sizeof(int)*3);

            stepNumFinished+=1;
            stepCount=0;
            isStepFinished=false;
            memcpy(bodyVelStart,bodyVelDes_2_c,sizeof(double)*2);//need turn and update

            rt_printf("/////////////////current step finished //////////////////////////////\n");

            if (gaitCommand==GaitCommand::Stop)
            {
                gaitState=GaitState::End;
                gaitCommand=GaitCommand::NoCommand;
            }
            else if(param.m==GaitMode::Single)
                gaitState=GaitState::End;
            else
                gaitState=GaitState::Walking;

        }

        //*** log data during walking period***//
        if(stepCount%10==0)
        {
            robotData data;
            //imu
            double imu[3];
            memset(imu,0,sizeof(double)*3);
            if(isIMUUsed==true)
                param.imu_data->toEulBody2Ground(imu,"231");
            imu[0]=0;
            imu[1]=asin(sin(imu[1]));
            imu[2]=asin(sin(imu[2]));
            memcpy(data.imu,imu,sizeof(double)*3);
            //phase
            int legPhase[6];
            memset(legPhase,0,sizeof(int)*6);
            for(int i=0;i<3;i++)
            {
                legPhase[swingID[i]]=int(isTD[i]==true);
                legPhase[stanceID[i]]=1;
            }
            memcpy(data.legPhase,legPhase,sizeof(int)*6);
            //force
            for(int i=0;i<6;i++)
                data.force[i]=param.force_data->at(i).Fz;
            //body leg waist
            robot.GetPeb(data.bodyPee,InitMak,"231");
            robot.GetPee(data.legPee,InitMak);
            robot.GetWa(data.waist);
            logPipe.sendToNrt(data);
        }



        return 1;

    case GaitState::End:
        //        dDist=0;
        //        dAngle=0;
        gaitState=GaitState::None;
        memset(bodyVelStart,0,sizeof(double)*2);
        rt_printf("step end\n");
        stepNumFinished=0;
        return 0;
        //    default:
        //        gaitState=GaitState::None;
        //        return 0;
    }
}

void GaitGenerator::GetTri2bodyFromTri( double* Tris,double yaw,double *TM_C2B)
{


    double y[3];
    double z[3];
    double x_prime[3]{cos(yaw),0,-sin(yaw)};
    double x[3];

    GetPlaneFromStanceLegs(Tris,y);

    //GetPlaneFromStanceLegs(SWFoothold_2_b0,y1_2_b0);
    aris::dynamic::s_cro3(x_prime,y,z);
    aris::dynamic::s_cro3(y,z,x);
    normalize(x);
    normalize(y);
    normalize(z);


    double TriC_origin[3];
    TriangleIncenter(Tris,TriC_origin);


    TM_C2B[0]=x[0];
    TM_C2B[1]=y[0];
    TM_C2B[2]=z[0];
    TM_C2B[3]=TriC_origin[0];
    TM_C2B[4]=x[1];
    TM_C2B[5]=y[1];
    TM_C2B[6]=z[1];
    TM_C2B[7]= TriC_origin[1];
    TM_C2B[8]=x[2];
    TM_C2B[9]=y[2];
    TM_C2B[10]=z[2];
    TM_C2B[11]= TriC_origin[2] ;
    TM_C2B[15]=1;
    double est_euler_c1_2_c0[6];
    aris::dynamic::s_pm2pe(TM_C2B,est_euler_c1_2_c0,"213");

}



void GaitGenerator::GetLeg2bodyFromLegs( double* Legs, double *TM_C2B)
{

    int stanceID[3]={0,2,4};
    int swingID[3]={1,5,3};

    double stanceLegs[9];
    double swingLegs[9];
    for (int i=0;i<3;i++)
    {
        memcpy(&stanceLegs[3*i],&Legs[stanceID[i]*3],sizeof(double)*3);
        memcpy(&swingLegs[3*i],&Legs[swingID[i]*3],sizeof(double)*3);
    }

    double y1[3];
    double y2[3];
    double y[3];
    double z[3];
    double x_prime[3]{1,0,0};
    double x[3];

    GetPlaneFromStanceLegs(stanceLegs,y1);
    GetPlaneFromStanceLegs(swingLegs,y2);


    normalize(y1);
    normalize(y2);
    y[0]=y1[0]+y2[0];  // this is actually the same as half quaternion method
    y[1]=y1[1]+y2[1];
    y[2]=y1[2]+y2[2];


    //GetPlaneFromStanceLegs(SWFoothold_2_b0,y1_2_b0);
    aris::dynamic::s_cro3(x_prime,y,z);
    aris::dynamic::s_cro3(y,z,x);
    normalize(x);
    normalize(y);
    normalize(z);


    //  double TriC_origin[3];
    double TriC_stance[3];
    double TriC_swing[3];
    TriangleIncenter(stanceLegs,TriC_stance);
    TriangleIncenter(swingLegs,TriC_swing);


    TM_C2B[0]=x[0];
    TM_C2B[1]=y[0];
    TM_C2B[2]=z[0];
    TM_C2B[3]=0.5*TriC_stance[0]+0.5*TriC_swing[0];
    TM_C2B[4]=x[1];
    TM_C2B[5]=y[1];
    TM_C2B[6]=z[1];
    TM_C2B[7]=0.5*TriC_stance[1]+0.5*TriC_swing[1];
    TM_C2B[8]=x[2];
    TM_C2B[9]=y[2];
    TM_C2B[10]=z[2];
    TM_C2B[11]=0.5*TriC_stance[2]+0.5*TriC_swing[2];
    TM_C2B[15]=1;
}

void parsePitch(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    for (auto &i : params)
    {
        if(i.first =="pitch")
        {
            dPitch=std::stod(i.second);
            break;
        }
        else
        {

            std::cout<<"parse failed"<<std::endl;

        }

        cout<<"parse finished"<<endl;
    }

}


void parseAdjustSlope(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{

    for (auto &i : params)
    {
        if(i.first =="forward")
        {
            dDist+=0.04;
            break;
        }
        else if (i.first == "backward")
        {
            dDist-=0.04;
            break;
        }
        else if(i.first =="turnleft")
        {
            dAngle+=0.04;
            break;
        }
        else if (i.first == "turnright")
        {
            dAngle-=0.04;
            break;
        }
        else if(i.first =="left")
        {
            dLateral+=0.04;
            break;
        }
        else if (i.first == "right")
        {
            dLateral-=0.04;
            break;
        }
        else if (i.first == "stop")
        {
            gaitCommand=GaitCommand::Stop;
            cout<<"stop command received !"<<endl;
            break;
        }
        else
        {
            std::cout<<"parse failed"<<std::endl;
        }

    }
    //   msg.copyStruct(param);
    cout<<"parse finished"<<endl;
}

void parseForce(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    for (auto &i : params)
    {
        if(i.first =="open")
        {
            isForceUsed=true;
            break;
        }
        else if (i.first == "closed")
        {
            isForceUsed=false;
            break;
        }

        else
        {
            std::cout<<"parse failed"<<std::endl;
        }

    }
    cout<<"parse finished"<<endl;
}

void parseIMU(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    for (auto &i : params)
    {
        if(i.first =="open")
        {
            isIMUUsed=true;
            break;
        }
        else if (i.first == "closed")
        {
            isIMUUsed=false;
            break;
        }

        else
        {
            std::cout<<"parse failed"<<std::endl;
        }

    }
    cout<<"parse finished"<<endl;
}

void parseVision(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
{
    for (auto &i : params)
    {
        if(i.first =="open")
        {
            isVisionUsed=true;
            break;
        }
        else if (i.first == "closed")
        {
            isVisionUsed=false;
            break;
        }

        else
        {
            std::cout<<"parse failed"<<std::endl;
        }

    }
    cout<<"parse finished"<<endl;
}

bool SetScrewLimits(double* ScrewIn,double* ScrewInFinal)
{

    bool isChanged=false;

    for(int i=0;i<18;i++)
    {
        if(ScrewIn[i]>=ScrewUpLimit[i]-ScrewMargin)
        {
            ScrewInFinal[i]=ScrewUpLimit[i]-ScrewMargin;
            isChanged=true;
        }
        else if(ScrewIn[i]<=ScrewDownLimit[i]+ScrewMargin)
        {
            ScrewInFinal[i]=ScrewDownLimit[i]+ScrewMargin;
            isChanged=true;
        }
        else
        {
            ScrewInFinal[i]=ScrewIn[i];
        }

    }
    return isChanged;

}




GaitGenerator::GaitGenerator()
{
    memcpy(m_CurrentConfig_b0.LegPee,stdLegPee2B,sizeof(double)*18);

    memcpy(m_NextConfig_b1.LegPee,stdLegPee2B,sizeof(double)*18);
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

void GaitGenerator::TMbody(const double *bodyP, const double *bodyR, double *tmbody)
{
    double TMtrans[16];
    double TMrot[16];
    Trans(bodyP,TMtrans);
    aris::dynamic::s_pe2pm(bodyR,TMrot,"213");
    aris::dynamic::s_pm_dot_pm(TMtrans,TMrot,tmbody);
}



void GaitGenerator::GetTerrainHeight2b( double *pos)
{
    //map 400*400*0.01 origined on the robot center
    //suppose vision device is mounted along the -z direction

    double gridRaw[2];
    gridRaw[0]=pos[0]/0.01+200;
    gridRaw[1]=pos[2]/0.01+200;


    int grid[2];
    if(ceil(gridRaw[0])/2+floor(gridRaw[0])/2>gridRaw[0])
        grid[0]=floor(gridRaw[0]);
    else
        grid[0]=ceil(gridRaw[0]);

    if(ceil(gridRaw[1])/2+floor(gridRaw[1])/2>gridRaw[1])
        grid[1]=floor(gridRaw[1]);
    else
        grid[1]=ceil(gridRaw[1]);



    // find the elevation with least variance
    double elevation[9];
    elevation[0]=gridMap[grid[0]-1][grid[1]-1];
    elevation[1]=gridMap[grid[0]][grid[1]-1];
    elevation[2]=gridMap[grid[0]+1][grid[1]-1];
    elevation[3]=gridMap[grid[0]-1][grid[1]];
    elevation[4]=gridMap[grid[0]][grid[1]];
    elevation[5]=gridMap[grid[0]+1][grid[1]];
    elevation[6]=gridMap[grid[0]-1][grid[1]+1];
    elevation[7]=gridMap[grid[0]][grid[1]+1];
    elevation[8]=gridMap[grid[0]+1][grid[1]+1];

    double variance[9];
    double mean[9];
    double points[8];
    for(int i=0;i<9;i++)
    {
        for (int j=0;j<8;j++)
            points[j]=elevation[(j+1)%9];
        variance[i]=Variance(points,8);
        mean[i]=Mean(points,8);
    }
    double v=100;
    for(int i=0;i<9;i++)
    {
        if(variance[i]<v)
        {
            v=variance[i];
            pos[1]=mean[i];
        }
    }



    //    if(grid[0]>0&&grid[0]<400&&grid[1]>0&&grid[1]<400)
    //        pos[1]=gridMap[grid[0]][grid[1]];
    //    else
    //        pos[1]=-0.85;
    pos[1]+=0.04;

    rt_printf("From vision: terrain height for sw, grid %d %d, elevation %f\n",grid[0],grid[1],pos[1]);
    //    rt_printf("grid[0][0]: %f\n",gridMap[0][0]);
    //    rt_printf("grid[0][100]: %f\n",gridMap[0][100]);
    //    rt_printf("grid[0][200]: %f\n",gridMap[0][200]);
    //    rt_printf("grid[100][0]: %f\n",gridMap[100][0]);
    //    rt_printf("grid[200][0]: %f\n",gridMap[200][0]);
    //    rt_printf("grid[100][100]: %f\n",gridMap[100][100]);


    if(pos[1]>-0.5||pos[1]<-1.2)
    {
        rt_printf("From Vision: impossible terrain height!\n");
        pos[1]=-0.9;

    }
}
double GaitGenerator::Mean(double *points,int N)
{
    double sum=0;
    for(int i=0;i<N;i++)
    {
        sum+=points[i];
    }
    double m=sum/N;
    return m;
}

double GaitGenerator::Variance(double *points,int N)
{
    double v=0;
    double m=Mean(points,N);
    for(int i=0;i<N;i++)
    {
        v+=(points[i]-m)*(points[i]-m);
    }
    v=v/N;
    return v;
}


void GaitGenerator::GetBodyOffset(const double pitch, const double roll, double* offset)
{

    double Roll=asin(sin(roll));
    // only for test
    offset[0]=-stdLegPee2B[1]*sin(Roll);
    offset[1]=0.0;
    offset[2]=stdLegPee2B[1]*sin(pitch);
}
void GaitGenerator::GetBodyOffsetRobotIX(const double pitch, const double roll, double* offset)
{

    // only for test
    offset[0]=-stdLegPee2B[1]*sin(roll);
    offset[1]=0.0;
    offset[2]=(stdLegPee2B[1])*tan(pitch);// not sure depend on robot elevation
    rt_printf("offset %f %f %f\n",offset[0],offset[1],offset[2]);
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
    double Norm=norm(vec);
    vec[0]=vec[0]/Norm;
    vec[1]=vec[1]/Norm;
    vec[2]=vec[2]/Norm;
}

void GaitGenerator::qautNormalize(double* vec)
{
    double Norm=sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]+vec[3]*vec[3]);
    vec[0]=vec[0]/Norm;
    vec[1]=vec[1]/Norm;
    vec[2]=vec[2]/Norm;
    vec[3]=vec[3]/Norm;
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



void GaitGenerator::TrajEllipsoid(const double *p0,const double* p1,const int count,const int totalCount,const double h,double* legpos)
{
    double theta;
    theta=PI*(1-cos(double(count)/totalCount*PI))/2; // 0 to PI
    double axisShort[3];
    //    double x[3];
    //    x[0]=1;
    //    x[1]=0;
    //    x[2]=0;
    double p01[3];
    p01[0]=p1[0]-p0[0];
    p01[1]=p1[1]-p0[1];
    p01[2]=p1[2]-p0[2];



    //this is simply not possible, because when p1 and p0 are so close, the traj will display a bizzare curve
    //    double lateralAxis[3];
    //    double y[3]={0, 1 ,0};
    //    aris::dynamic::s_cro3(p01,y,lateralAxis);
    //    aris::dynamic::s_cro3(p01,lateralAxis,axisShort);

    //    double h_axis=axisShort[1];

    //    if(h_axis==0)
    //    {

    //        axisShort[0]=0;
    //        axisShort[1]=h;
    //        axisShort[2]=0;
    //    }
    //    else
    //    {

    //        axisShort[0]=axisShort[0]/h_axis*h;
    //        axisShort[1]=h;
    //        axisShort[2]=axisShort[2]/h_axis*h;
    //    }

    double h_actual;
    h_actual=sqrt(h*h+h*abs(p1[1]-p0[1]));

    axisShort[0]=0;
    axisShort[1]=h_actual;
    axisShort[2]=0;

    legpos[0]=(p0[0]+p1[0])/2+(p0[0]-p1[0])/2*cos(theta)+axisShort[0]*sin(theta);
    legpos[1]=(p0[1]+p1[1])/2+(p0[1]-p1[1])/2*cos(theta)+axisShort[1]*sin(theta);
    legpos[2]=(p0[2]+p1[2])/2+(p0[2]-p1[2])/2*cos(theta)+axisShort[2]*sin(theta);
    // rt_printf("legpos %f %f %f\n",legpos[0],legpos[1],legpos[2]);
}

void GaitGenerator::TrajEllipsoid(const double *p0, const double *p1, const int count, const int totalCount, double *legpos)
{
    double theta;
    theta=PI*(1-cos(double(count)/totalCount*PI))/2; // 0 to PI
    double axisShort[3];
    //    double x[3];
    //    x[0]=1;
    //    x[1]=0;
    //    x[2]=0;
    double p01[3];
    p01[0]=p1[0]-p0[0];
    p01[1]=p1[1]-p0[1];
    p01[2]=p1[2]-p0[2];


    //    if (abs(p01[2])<0.01)
    //    {
    //        axisShort[0]=0;
    //        axisShort[1]=1;
    //        axisShort[2]=0;
    //    }
    //    else
    //    {
    //        aris::dynamic::s_cro3(x,p01,axisShort);
    //    }
    //    axisShort[0]=axisShort[0]/axisShort[1]*m_Params.h;
    //    axisShort[1]=m_Params.h;
    //    axisShort[2]=axisShort[2]/axisShort[1]*m_Params.h;


    axisShort[0]=0;
    axisShort[1]=m_Params.h;
    axisShort[2]=0;


    legpos[0]=(p0[0]+p1[0])/2+(p0[0]-p1[0])/2*cos(theta)+axisShort[0]*sin(theta);
    legpos[1]=(p0[1]+p1[1])/2+(p0[1]-p1[1])/2*cos(theta)+axisShort[1]*sin(theta);
    legpos[2]=(p0[2]+p1[2])/2+(p0[2]-p1[2])/2*cos(theta)+axisShort[2]*sin(theta);
    // rt_printf("legpos %f %f %f\n",legpos[0],legpos[1],legpos[2]);

}

}



