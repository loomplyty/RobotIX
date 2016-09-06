#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <map>
#include <string>
#include <eigen3/Eigen/Dense>

using namespace std;

#include <aris.h>
#include <Basic_Gait.h>
#include <Robot_Type_III.h>
//#include <Robot_Gait.h>
//#include <Robot_Base.h>
#include <Robot_Type_I.h>
#include "Vision_Gait0.h"
#include "Kinect2.h"
#include "Kinect2Test.h"

#include "rtdk.h"
#include "unistd.h"


//tianyuan
#include"GaitGenerator.h"

using namespace aris::core;



Kinect2Sensor::KINECT2 kinect2;





int main(int argc, char *argv[])
{
   //kinect2.Start();

   // VersatileGait::startLogDataThread();
    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/Desktop/RobotIX/resource/Robot_IX.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/Desktop/RobotIII/resource/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
        xml_address = "/home/hex/Kinect2/Robot_VIII.xml";
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

    auto &rs = aris::server::ControlServer::instance();

    rs.createModel<Robots::RobotTypeIII>();
    rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::Gait::basicParse, nullptr);
    rs.addCmd("ds", Robots::Gait::basicParse, nullptr);
    rs.addCmd("hm", Robots::Gait::basicParse, nullptr);
    rs.addCmd("rc", Robots::Gait::recoverParse, Robots::Gait::recoverGait);
    rs.addCmd("wk", Robots::Gait::walkParse, Robots::Gait::walkGait);
    rs.addCmd("ro", Robots::Gait::resetOriginParse, Robots::Gait::resetOriginGait);
    rs.addCmd("hmsw", Robots::Gait::basicParse, nullptr);
    rs.addCmd("ec",Robots::Gait::extendChainParse,Robots::Gait::extendChainGait);
    //wasit
    rs.addCmd("rcw",Robots::Gait::recoverWaistParse,Robots::Gait::recoverWaistGait);
    rs.addCmd("aw",Robots::Gait::adjustWaistParse,Robots::Gait::adjustWaistGait);
    //    rs.addCmd("ca", visionCalibrateParse, visionCalibrate);
    //    rs.addCmd("vwk", visionWalkParse, visionWalk);
    //    rs.addCmd("swk", stopVisionWalkParse, visionWalk);

    //slope walking
    rs.addCmd("adj",VersatileGait::parseAdjustSlope,VersatileGait::GoSlopeByVisionFast2);
    rs.addCmd("frc",VersatileGait::parseForce,VersatileGait::GoSlopeByVisionFast2);
    rs.addCmd("imu",VersatileGait::parseIMU,VersatileGait::GoSlopeByVisionFast2);
    rs.addCmd("vis",VersatileGait::parseVision,VersatileGait::GoSlopeByVisionFast2);

    rs.addCmd("gsvf2",VersatileGait::parseGoSlopeVisionFast2,VersatileGait::GoSlopeByVisionFast2);
    rs.addCmd("pitch",VersatileGait::parsePitch,VersatileGait::GoSlopeByVisionFast2);
    rs.addCmd("gs35",VersatileGait::parseGoSlope35,VersatileGait::GoSlope35);

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
