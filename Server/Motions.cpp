#include <cstring>
#include <cmath>
#include <algorithm>
#include <memory>

#include "Motions.h"
#include "rtdk.h"


#define GoUpStairLength_Enough 120000
static double Gait_GoUpStair[18*GoUpStairLength_Enough];
static int GaitStairCount;
 void GoStair::parseGoUpStair(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg)
{
    GoStairParam param;
    for (auto &i:params)
    {
        if(i.first=="file")
        {


                //插入步态*/
                std::ifstream file;
                std::string FileName = i.second;
                cout<<"file name:"<<FileName<<endl;

               int gaitNum{ -1 };

                file.open(FileName);
                if (!file) throw std::logic_error("gostair file not exist");
                for (double tem; !file.eof(); file >> tem) ++gaitNum;
                if (gaitNum % 18 != 0) throw std::logic_error("gostair file invalid, because the num of numbers is not valid");
                gaitNum /= 18;
                file.close();

                param.gaitCount = gaitNum;
                GaitStairCount=gaitNum;

               // std::unique_ptr<double> p(new double[gaitNum* 18]);


               // Gait_GoUpStair=p.get();


                file.open(FileName);
                for (int i = 0; !file.eof(); file >> Gait_GoUpStair[i++]);
                file.close();

                cout<<"gaitnum:"<<gaitNum<<endl;

        }

        else
        {
            throw std::logic_error("internal error happened, because invalid params in parseFastWalk");
        }
    }

    msg.copyStruct(param);

 }

 void GoStair::parseGoDownStair(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg)
{
     GoStairParam param;
     param.gaitCount=GaitStairCount;
     msg.copyStruct(param);
 }

int GoStair::GoUpStair(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{

    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &pSP=static_cast<const GoStair::GoStairParam &>(param_in);
    if(pSP.count==0)
    {
        cout<<"gostair begins"<<"gait length:"<<pSP.gaitCount<<endl;
        cout<<"pin value"<<Gait_GoUpStair[0]<<" "<<Gait_GoUpStair[1]<<" "<<Gait_GoUpStair[17]<<endl;
    }


    if(pSP.count < pSP.gaitCount)
    {
         robot.SetPin(&Gait_GoUpStair[pSP.count*18]);
    }

   return pSP.gaitCount-pSP.count-1;

}


int GoStair::GoDownStair(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)
{

     auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &pSP=static_cast<const GoStair::GoStairParam &>(param_in);
    if(pSP.count==0)
    {
        cout<<"go down stair begins"<<"gait length:"<<pSP.gaitCount<<endl;
        cout<<"pin value"<<Gait_GoUpStair[0]<<" "<<Gait_GoUpStair[1]<<" "<<Gait_GoUpStair[17]<<endl;
    }


    if(pSP.count < pSP.gaitCount)
    {
         robot.SetPin(&Gait_GoUpStair[(pSP.gaitCount-pSP.count-1)*18]);
    }

   return pSP.gaitCount-pSP.count-1;

}



