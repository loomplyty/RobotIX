#include "Client.h"
#include <iostream>
#include <cstring>

using namespace std;

using namespace Aris::Core;


CONN ControlSysClient;

//CONN callback functions
int OnConnDataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data)
{
	if(data.GetMsgID()==0)
 		 Aris::Core::PostMsg(Aris::Core::MSG(ControlCommandNeeded));

	else if (data.GetMsgID()==10000)
	{
		Aris::Core::MSG msg(data);
		data.SetMsgID(RTDataGet);
		Aris::Core::PostMsg(msg);
	}
	return 0;
}

int OnConnectLost(Aris::Core::CONN *pConn)
{
	PostMsg(Aris::Core::MSG(SystemLost));
	return 0;
}
//MSG callback functions
int OnControlCommandNeeded(Aris::Core::MSG &msg)
{
	int cmd;
	cout<<"Commands:"<<endl;
	cout<<"1.Enable"<<endl<<"2.Running"<<endl<<"3.Disable"<<endl<<"5.Gohome_1"<<endl<<"6.Gohome_2"<<endl<<"7.HOme2start_1"<<endl<<"8.Home2start_2"<<endl<<"9. BackToStandstill"<<endl<<"10.Forward"<<endl<<"11.Backward"<<endl<<"12.Fast_Forward"<<endl<<"13.Fast_Backward"<<endl<<"14. TurnRight"<<endl<<"15.TurnLeft"<<endl<<"16.LegUp"<<endl;

	cout<<"Please enter your command: ";
	 cin>>cmd;
	while(cmd<1||cmd>16){
	  cout<<"Not valid command ID,enter again : ";
	  cin>>cmd;
	}
 	MSG data;
	data.SetMsgID(cmd);
	ControlSysClient.SendData(data);
	return 0;
}



int OnSystemLost(Aris::Core::MSG &msg)
{
	cout<<"Control system lost"<<endl;
	return 0;
}

 
