#include "Client.h"
#include <iostream>
#include <cstring>

using namespace std;

using namespace Aris::Core;


CONN ControlSysClient;

//CONN callback functions
int OnConnDataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data)
{
	Aris::Core::PostMsg(Aris::Core::MSG(ControlCommandNeeded));
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
	cout<<"1.PowerOff"<<endl<<"2.Stop"<<endl<<"3.Enable"<<endl<<"4.Running"<<endl<<"5.Gohome_1"<<endl<<"6.Gohome_2"<<endl<<"7.HOme2start_1"<<endl<<"8.Home2start_2"<<endl<<"9.Forward"<<endl<<"10.Backward"<<endl<<"11.Fast_Forward"<<endl<<"12.Fast_Backward"<<endl<<"13.Legup"<<endl<<"14.Turnleft"<<endl<<"15.TurnRight"<<endl<<"16.Online Gait"<<endl;

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

 
