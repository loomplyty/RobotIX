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
     /*       static std::map<std::string, std::tuple<int, std::unique_ptr<double> > > walkFileMap;

            const auto found = walkFileMap.find(i.second);
            if (found != walkFileMap.end())
            {
                //步态已经存在
                std::tie(param.gaitCount, std::ignore) = found->second;
                param.pIn = std::get<3>(found->second).get();
                std::cout << "gostair count:" << param.count << std::endl;

            }
            else
            {
                if (walkFileMap.size() > 1)
                {
                    throw std::runtime_error("only one path is allowed");
                }

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
               // cout<<"pin 1: "<<param.pIn[0]<<" "<<param.pIn[17]<<" "<<param.pIn[18]<<endl;
               // cout<<"pin end: "<<param.pIn[gaitNum*18-17]<<" "<<param.pIn[gaitNum*18-16]<<" "<<param.pIn[gaitNum*18-1]<<endl;
               // cout<<"pin 1 global poiter: "<<Gait_GoUpStair[0]<<" "<<Gait_GoUpStair[17]<<" "<<Gait_GoUpStair[18]<<endl;

              //  walkFileMap.insert(std::make_pair(i.second, std::make_tuple(gaitNum, std::move(p))));
            //}
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



