#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <map>
#include <string>

using namespace std;

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>
#include <Robot_Type_I.h>
#include <Aris_Vision.h>
#include "Vision_Terrain0.h"
#include "Vision_Control0.h"
#include "Vision_Gait0.h"
#include "Motions.h"

#include "rtdk.h"
#include "unistd.h"

//liujimu's gaits
#include "continuous_walk_with_force.h"
#include "push_recovery.h"
#include "continuous_move.h"
#include "twist_waist.h"
#include "say_hello.h"
#include "peg_in_hole.h"
//zhaoyue
#include "swing.h"
//sunqiao
#include "MovePushDoorSimple.h"
#include "MoveCrowdPassingGait.h"
#include "rofo2.h"
// tianyuan optsolver
#include "Solver.h"
#include"GaitGenerator.h"
using namespace aris::core;

aris::sensor::KINECT kinect1;

TerrainAnalysis terrainAnalysisResult;

atomic_bool isTerrainAnalysisFinished(false);
atomic_bool isTerrainCaliRecorded(false);
enum CalibrationState
{
    None=0,
    Go=1,
    Processing=2,
    Back=3,
};
atomic_int calibrationState{CalibrationState::None};

atomic_bool isSending(false);
atomic_bool isStop(false);

VISION_WALK_PARAM visionWalkParam;




struct matrix44
{
    double pm[4][4];
};

aris::control::Pipe<int> visionPipe(true);
aris::control::Pipe<matrix44> visionRecordPipe(true);
//aris::control::Pipe<optInput> InputToOptPipe(true); //vision to opt
aris::control::Pipe<int> visionCalibratePipe(true);



bool optInit=false;
bool optTerminated=false;


enum  VISION_DEMAND_TYPE
{
    FIRST_STEP=0,
    CONTINUOUS_STEP=1,
};

/*static auto optThread= std::thread([]()
{
    while(true)
    {
        if(optInit=false)
        {
            //init calling matlab mosek
            optInit=true;
        }
        OptInput optIn;
        InputToOptPipe.recvInNrt(optIn);
        // call matlab function and compute output

        OutputFromOptPipe.sendToNrt(optOut);
    }

});*/

//static auto visionRecordThread = std::thread([]()
//{
//    while(true)
//    {
//        matrix44 currentPm;

//        visionRecordPipe.recvInNrt(currentPm);

//        auto visiondata = kinect1.getSensorData();
//        terrainAnalysisResult.TerrainRecordAll(visiondata.get().gridMap,currentPm.pm);
//        cout<<"big map recorded."<<endl;

//    }
//});

//static auto visionCalibrateThread = std::thread([]()
//{
//    while(true)
//    {
//        int postureCount;
//        visionCalibratePipe.recvInNrt(postureCount);
//        auto visiondata = kinect1.getSensorData();
//        terrainAnalysisResult.TerrainRecord(visiondata.get().gridMap);
//        cout<<" map recorded"<<endl;
//        isTerrainCaliRecorded=true;
//    }
//});

//static auto visionThread = std::thread([]()
//{
//    while(true)
//    {
//        int a;
//        visionPipe.recvInNrt(a);

//        auto visiondata = kinect1.getSensorData();
//        terrainAnalysisResult.TerrainAnalyze(visiondata.get().gridMap);

//        if(terrainAnalysisResult.Terrain != FlatTerrain)
//        {
//            /*Adjust x y z theta*/
//            double paramAdjust[4] = {0, 0, 0, 0};
//            bool adjustFinished = false;
//            visionAdjust(paramAdjust, &adjustFinished);

//            if(adjustFinished == false)
//            {
//                /*let robot move*/
//                if(paramAdjust[3] != 0)
//                {
//                    visionWalkParam.movetype = turn;
//                    visionWalkParam.turndata = paramAdjust[3];
//                    visionWalkParam.totalCount = 6000/2;
//                    cout<<"terrain turn"<<endl;
//                }
//                else
//                {
//                    visionWalkParam.movetype = flatmove;
//                    memcpy(visionWalkParam.movedata,paramAdjust,3*sizeof(double));
//                    visionWalkParam.totalCount = 5000/2;
//                    cout<<"terrain move"<<endl;
//                }
//            }
//            else
//            {
//                cout<<"Find the Position!!!"<<endl;
//                cout<<"Begin Climb Up!!!"<<endl;
//                visionWalkParam.movetype = stepup;
//                switch (terrainAnalysisResult.Terrain)
//                {
//                case StepUpTerrain:
//                {
//                    /*the robot move body*/
//                    cout<<"Begin Climb Up!!!"<<endl;
//                    visionWalkParam.movetype = stepup;
//                }
//                    break;
//                case StepDownTerrain:
//                {
//                    /*the robot move body*/
//                    cout<<"Begin Climb Down!!!"<<endl;
//                    visionWalkParam.movetype = stepdown;
//                }
//                    break;
//                default:
//                    break;
//                }
//            }
//        }
//        else
//        {
//            cout<<"FLAT TERRAIN MOVE"<<endl;
//            cout<<"MOVE FORWARD: "<<0.325<<endl;
//            double move_data[3] = {0, 0, 0.325};

//            visionWalkParam.movetype = flatmove;
//            visionWalkParam.totalCount = 5000/2;
//            memcpy(visionWalkParam.movedata,move_data,sizeof(move_data));
//        }

//        isTerrainAnalysisFinished = true;
//        cout<<"terrrainended"<<endl;
//    }
//});


auto visionWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    aris::server::GaitParamBase param;
    msg_out.copyStruct(param);
}

auto visionWalk(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int
{
    static bool isFirstTime = true;

    if (isTerrainAnalysisFinished)
    {
        if(isFirstTime)
        {
            visionWalkParam.count = 0;
            isFirstTime = false;
        }

        auto &robot = static_cast<Robots::RobotBase &>(model);

        switch(visionWalkParam.movetype)
        {
        case turn:
        {

            int remainCount = RobotVisionWalk(robot, visionWalkParam);
            visionWalkParam.count++;

            if(remainCount == 0 && isStop == true)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return -1;
            }
        }
            break;
        case flatmove:
        {
            int remainCount = RobotVisionWalk(robot, visionWalkParam);
            visionWalkParam.count++;

            if(remainCount == 0 && isStop == true)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return -1;
            }
        }
            break;

        case bodymove:
        {
            RobotBody(robot, visionWalkParam);
            int remainCount = visionWalkParam.totalCount - visionWalkParam.count - 1;
            visionWalkParam.count++;
            if(remainCount == 0 && isStop == true)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return -1;
            }
        }
            break;
        case stepup:
        {
            isStop = false;
            isFirstTime = true;
            isSending = false;
            isTerrainAnalysisFinished = false;
            return 0;
        }
            break;
        case stepdown:
        {
            isStop = false;
            isFirstTime = true;
            isSending = false;
            isTerrainAnalysisFinished = false;
            return 0;
        }
            break;
        }
    }
    else
    {
        if(isSending)
        {
            return -1;
        }
        else
        {
            visionPipe.sendToNrt(6);
            isSending = true;
            return -1;
        }
    }
}

auto stopVisionWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    isStop = true;
}

auto visionCalibrateParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    VISION_CALIBRATION_PARAM param;

    for (auto &i:params)
    {
        if(i.first=="file")
        {

            std::ifstream file;
            std::string FileName = i.second;
            cout<<"file name:"<<FileName<<endl;

            int postureNum{ -1 };

            file.open(FileName);
            if (!file) throw std::logic_error("calibration params file not exist");
            for (double tem; !file.eof(); file >> tem) ++postureNum;
            if (postureNum % 6 != 0) throw std::logic_error("calibration params file invalid, because the num of numbers is not valid");
            postureNum /= 6;
            file.close();

            param.postureNum = postureNum;


            file.open(FileName);
            for (int i = 0; !file.eof(); file >> param.postureLib[i++]);
            file.close();

            cout<<"postureNum:"<<postureNum<<endl;
            // cout<<"pin 1: "<<param.pIn[0]<<" "<<param.pIn[17]<<" "<<param.pIn[18]<<endl;
            // cout<<"pin end: "<<param.pIn[gaitNum*18-17]<<" "<<param.pIn[gaitNum*18-16]<<" "<<param.pIn[gaitNum*18-1]<<endl;
            // cout<<"pin 1 global poiter: "<<Gait_GoUpStair[0]<<" "<<Gait_GoUpStair[17]<<" "<<Gait_GoUpStair[18]<<endl;

            //  walkFileMap.insert(std::make_pair(i.second, std::make_tuple(gaitNum, std::move(p))));
            //}
        }

        else
        {
            throw std::logic_error("internal error happened, because invalid params in parseVisionCalibration");
        }
    }

    msg_out.copyStruct(param);


}

/*auto visionPosture(aris::dynamic::Model &model,  aris::dynamic::PlanParamBase & cali_param)->int
{
    auto &robot=static_cast<Robots::RobotBase &>(model);
    auto &pSP=static_cast<  VISION_CALIBRATION_PARAM &>(cali_param);
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    double currentPeb[6];
    double s;
    switch(calibrationState)
    {
    case Go:
        if(pSP.localCount==0)
        {
            rt_printf("calibration posture %d\n",pSP.postureCount);
            memcpy(pSP.targetPosture,&pSP.postureLib[6*pSP.postureCount],sizeof(pSP.targetPosture));
         //   cout<<"fixed rotation value"<<pSP.targetPosture[0]<<" "<<pSP.targetPosture[1]<<" "<<pSP.targetPosture[2]<<endl;
          //  cout<<"elevation value"<<pSP.targetPosture[4]<<endl;
        }

        s=PI*(pSP.localCount+1)/pSP.gaitLength;// (0,pi]
        currentPeb[3]=pSP.targetPosture[2]*(1-cos(s))/2;
        currentPeb[4]=pSP.targetPosture[1]*(1-cos(s))/2;
        currentPeb[5]=pSP.targetPosture[0]*(1-cos(s))/2;
        currentPeb[0]=pSP.targetPosture[3]*(1-cos(s))/2;
        currentPeb[1]=pSP.targetPosture[4]*(1-cos(s))/2;
        currentPeb[2]=pSP.targetPosture[5]*(1-cos(s))/2;


        robot.SetPeb(currentPeb,beginMak,"132");
       return pSP.gaitLength-pSP.localCount-1;
    case Back:
        if(pSP.localCount==0)
        {
            rt_printf("calibration posture finished going back%d\n",pSP.postureCount);
         }
          s=PI*(pSP.localCount+1)/pSP.gaitLength;// (0,pi]
        currentPeb[3]=pSP.targetPosture[2]*(1+cos(s))/2;
        currentPeb[4]=pSP.targetPosture[1]*(1+cos(s))/2;
        currentPeb[5]=pSP.targetPosture[0]*(1+cos(s))/2;
        currentPeb[0]=pSP.targetPosture[3]*(1+cos(s))/2;
        currentPeb[1]=pSP.targetPosture[4]*(1+cos(s))/2;
        currentPeb[2]=pSP.targetPosture[5]*(1+cos(s))/2;


        robot.SetPeb(currentPeb,beginMak,"132");

       return pSP.gaitLength-pSP.localCount-1;
    default:
        rt_printf("invalid status");
        return -1;
    }



    if(pSP.localCount==0)
    {
        rt_printf("calibration posture %d\n",pSP.postureCount);
        memcpy(pSP.targetPosture,&pSP.postureLib[6*pSP.postureCount],sizeof(pSP.targetPosture));
     //   cout<<"fixed rotation value"<<pSP.targetPosture[0]<<" "<<pSP.targetPosture[1]<<" "<<pSP.targetPosture[2]<<endl;
      //  cout<<"elevation value"<<pSP.targetPosture[4]<<endl;
    }
    double currentPeb[6];

    double s;
    s=PI*(pSP.localCount+1)/pSP.gaitLength;// (0,pi]
    currentPeb[3]=pSP.targetPosture[2]*(1-cos(s))/2;
    currentPeb[4]=pSP.targetPosture[1]*(1-cos(s))/2;
    currentPeb[5]=pSP.targetPosture[0]*(1-cos(s))/2;
    currentPeb[0]=pSP.targetPosture[3]*(1-cos(s))/2;
    currentPeb[1]=pSP.targetPosture[4]*(1-cos(s))/2;
    currentPeb[2]=pSP.targetPosture[5]*(1-cos(s))/2;


    robot.SetPeb(currentPeb,beginMak,"132");
   return pSP.gaitLength-pSP.localCount-1;
}*/

/*auto visionPostureBack(aris::dynamic::Model &model,  aris::dynamic::PlanParamBase & cali_param)->int
{
    auto &robot=static_cast<Robots::RobotBase &>(model);
    auto &pSP=static_cast<  VISION_CALIBRATION_PARAM &>(cali_param);
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };

    if(pSP.localCount==0)
    {
        rt_printf("calibration posture finished going back%d\n",pSP.postureCount);
     }
    double currentPeb[6];
    double s;
    s=PI*(pSP.localCount+1)/pSP.gaitLength;// (0,pi]
    currentPeb[3]=pSP.targetPosture[2]*(1+cos(s))/2;
    currentPeb[4]=pSP.targetPosture[1]*(1+cos(s))/2;
    currentPeb[5]=pSP.targetPosture[0]*(1+cos(s))/2;
    currentPeb[0]=pSP.targetPosture[3]*(1+cos(s))/2;
    currentPeb[1]=pSP.targetPosture[4]*(1+cos(s))/2;
    currentPeb[2]=pSP.targetPosture[5]*(1+cos(s))/2;


    robot.SetPeb(currentPeb,beginMak,"132");

   return pSP.gaitLength-pSP.localCount-1;

}*/

auto visionCalibrate(aris::dynamic::Model &model,  aris::dynamic::PlanParamBase & cali_param)->int
{
    auto &robot=static_cast<Robots::RobotBase &>(model);
    auto &pSP=static_cast<  VISION_CALIBRATION_PARAM &>(cali_param);
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    double currentPeb[6];
    double s;

    if(pSP.count==0)
    {
        rt_printf("calibration gait begins\n");
    }

    pSP.localCount=pSP.localCount%pSP.gaitLength;

    switch(calibrationState)
    {
    case None:
        calibrationState=Go;
        break;
    case Go:
        //  ret=visionPosture(model,cali_param);

        if(pSP.localCount==0)
        {
            rt_printf("calibration posture %d\n",pSP.postureCount);
            memcpy(pSP.targetPosture,&pSP.postureLib[6*pSP.postureCount],sizeof(pSP.targetPosture));
            //   cout<<"fixed rotation value"<<pSP.targetPosture[0]<<" "<<pSP.targetPosture[1]<<" "<<pSP.targetPosture[2]<<endl;
            //  cout<<"elevation value"<<pSP.targetPosture[4]<<endl;
        }

        s=PI*(pSP.localCount+1)/pSP.gaitLength;// (0,pi]
        currentPeb[3]=pSP.targetPosture[2]*(1-cos(s))/2;
        currentPeb[4]=pSP.targetPosture[1]*(1-cos(s))/2;
        currentPeb[5]=pSP.targetPosture[0]*(1-cos(s))/2;
        currentPeb[0]=pSP.targetPosture[3]*(1-cos(s))/2;
        currentPeb[1]=pSP.targetPosture[4]*(1-cos(s))/2;
        currentPeb[2]=pSP.targetPosture[5]*(1-cos(s))/2;


        robot.SetPeb(currentPeb,beginMak,"132");
        pSP.localCount+=1;

        if(pSP.gaitLength-pSP.localCount==0)
        {
            calibrationState=Processing;
            visionCalibratePipe.sendToNrt(pSP.postureCount);
        }
        break;
    case Processing:
        if(isTerrainCaliRecorded==true)
        {
            calibrationState=Back;
            isTerrainCaliRecorded=false;
        }
        break;
    case Back:
        //  ret=visionPosture(model,cali_param);
        if(pSP.localCount==0)
        {
            rt_printf("calibration posture finished going back%d\n",pSP.postureCount);
        }
        s=PI*(pSP.localCount+1)/pSP.gaitLength;// (0,pi]
        currentPeb[3]=pSP.targetPosture[2]*(1+cos(s))/2;
        currentPeb[4]=pSP.targetPosture[1]*(1+cos(s))/2;
        currentPeb[5]=pSP.targetPosture[0]*(1+cos(s))/2;
        currentPeb[0]=pSP.targetPosture[3]*(1+cos(s))/2;
        currentPeb[1]=pSP.targetPosture[4]*(1+cos(s))/2;
        currentPeb[2]=pSP.targetPosture[5]*(1+cos(s))/2;


        robot.SetPeb(currentPeb,beginMak,"132");
        pSP.localCount+=1;

        if(pSP.gaitLength-pSP.localCount==0)
        {
            pSP.postureCount+=1;
            calibrationState=None;
            if(pSP.postureCount==pSP.postureNum)
                return 0;
        }
        break;
    default:
        break;
    }

    return 1;

}

auto mapShot(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int
{
    auto &robot = static_cast<Robots::RobotBase &>(model);

    matrix44 currentPm;
    robot.GetPmb(*currentPm.pm);


    rt_printf("currentPm.pm %f,%f,%f,%f\n",currentPm.pm[0][0],currentPm.pm[0][1],currentPm.pm[0][2],currentPm.pm[0][3]);
    rt_printf("currentPm.pm %f,%f,%f,%f\n",currentPm.pm[1][0],currentPm.pm[1][1],currentPm.pm[1][2],currentPm.pm[1][3]);
    rt_printf("currentPm.pm %f,%f,%f,%f\n",currentPm.pm[2][0],currentPm.pm[2][1],currentPm.pm[2][2],currentPm.pm[2][3]);
    rt_printf("currentPm.pm %f,%f,%f,%f\n",currentPm.pm[3][0],currentPm.pm[3][1],currentPm.pm[3][2],currentPm.pm[3][3]);

    visionRecordPipe.sendToNrt(currentPm);
    return 0;
}

auto visionRecordParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    aris::server::GaitParamBase param;
    msg_out.copyStruct(param);
}

int main(int argc, char *argv[])
{
    //kinect1.start();

    PushDoorSimple::PushDoorSimpleWrapper::StartReceiveData();
    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address ="/home/hex/Desktop/Robot_VIII/resource/Robot_VIII.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/Desktop/RobotIII/resource/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
        xml_address = "/home/hex/ArisVision/VisionStairs_Test/Robot_VIII.xml";
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

    auto &rs = aris::server::ControlServer::instance();

    rs.createModel<Robots::RobotTypeI>();
    rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::basicParse, nullptr);
    rs.addCmd("ds", Robots::basicParse, nullptr);
    rs.addCmd("hm", Robots::basicParse, nullptr);
    rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
    rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
    rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);

    rs.addCmd("gus", GoStair::parseGoUpStair,GoStair::GoUpStair);
    rs.addCmd("gds", GoStair::parseGoDownStair,GoStair::GoDownStair);

    rs.addCmd("vwk", visionWalkParse, visionWalk);
    rs.addCmd("swk", stopVisionWalkParse, visionWalk);
    rs.addCmd("record",visionRecordParse,mapShot);

    rs.addCmd("climb",Rofo::rofoParse,Rofo::rofoGait);
    rs.addCmd("edcl", Rofo::rofoEndParse,Rofo::rofoEndGait);
    rs.addCmd("ay",Rofo::ayParse,Rofo::ayGait);

    //liujimu's gaits
    rs.addCmd("cwf", CWFParse, CWFGait);
    rs.addCmd("cwfs", CWFStopParse, CWFGait);
    rs.addCmd("pr", pushRecoveryParse, pushRecoveryGait);
    rs.addCmd("prs", pushRecoveryStopParse, pushRecoveryGait);
    rs.addCmd("cmb", parseContinuousMoveBegin, continuousMove);
    rs.addCmd("cmj", parseContinuousMoveJudge, continuousMove);
    rs.addCmd("tw", twistWaistParse, twistWaistGait);
    rs.addCmd("sh", sayHelloParse, sayHelloGait);
    rs.addCmd("ph", pegInHoleParse, pegInHoleGait);
    //zhaoyue
    rs.addCmd("sw", swingParse, swingGait);
    //sunqiao
    rs.addCmd("psd", PushDoorSimple::PushDoorSimpleWrapper::ParseCmds,
              PushDoorSimple::PushDoorSimpleWrapper::GaitFunction);
    rs.addCmd("cpp", CrowdPassing::CrowdPassingGaitWrapper::ParseCmds,
              CrowdPassing::CrowdPassingGaitWrapper::GaitFunction);


    //slope walking
    rs.addCmd("gsv",VersatileGait::parseGoSlopeVision,VersatileGait::GoSlopeByVision);
    rs.addCmd("adj",VersatileGait::parseAdjustSlope,VersatileGait::GoSlopeByVision);
    rs.addCmd("gsh",VersatileGait::parseGoSlopeHuman,VersatileGait::GoSlopeByHuman);

    Rofo::RofoWalkInit();
    rs.open();

    rs.setOnExit([&]()
    {
        aris::core::XmlDocument xml_doc;
        xml_doc.LoadFile(xml_address.c_str());
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)
            throw std::runtime_error("can't find Model element in xml file");
        rs.model().saveXml(*model_xml_ele);

        aris::core::stopMsgLoop();
    });

    aris::core::runMsgLoop();

    return 0;

}
