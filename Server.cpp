#include "Server.h"
#include "iostream"
#include <cstring>

using namespace std;

using namespace Aris::Core;



//static int LastCMD=-1;

CONN ControlSystem, VisualSystem;

//#include "Hexapod_Robot.h"

// CONN call back functions
int On_CS_ConnectionReceived(Aris::Core::CONN *pConn, const char* addr,int port)
{
	Aris::Core::MSG msg;

		msg.SetMsgID(CS_Connected);
		msg.SetLength(sizeof(port));
		msg.Copy(&port,sizeof(port));
		msg.CopyMore(addr,strlen(addr));
		PostMsg(msg);
	return 0;

}
int On_CS_DataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data)
{
	    int cmd=data.GetMsgID();
		//RobotCMD_Msg CMD;
        MSG CMD=CS_CMD_Received;
       // int ID=CMD.GetMsgID();
      //cout<<"MSG ID is :"<<ID<<endl;
        CMD.SetLength(sizeof(int));
		//add control matrix
        bool IsCMDExecutable=true;

        CMD.Copy(&cmd,sizeof(int));
		cout<<"received CMD is"<<cmd<<endl;
		PostMsg(CMD);
	//	LastCMD=data.GetMsgID();
		return 0;
}
int On_CS_ConnectionLost(Aris::Core::CONN *pConn)
{
	PostMsg(Aris::Core::MSG(CS_Lost));
	return 0;
}

//MSG call back functions
int On_CS_Connected(Aris::Core::MSG &msg)
{
	cout<<"Received Connection from Control System:"<<endl;
	cout<<"   Remote IP is: "<<msg.GetDataAddress()+sizeof(int)<<endl;
	cout<<"   Port is     : "<<*((int*)msg.GetDataAddress()) << endl << endl;

	Aris::Core::MSG data(0,0);
	ControlSystem.SendData(data);
	return 0;
}

int On_CS_CMD_Received(Aris::Core::MSG &msg)
{

	MSG Command(msg);
	Command.SetMsgID(GetControlCommand);
	PostMsg(Command);
	int cmd;
//	Command.Paste(&cmd,sizeof(int));
//	cout<<"received CMD in cmd received is"<<cmd<<endl;

	MSG data(0,0);
	ControlSystem.SendData(data);
	return 0;
}

int On_CS_Lost(Aris::Core::MSG &msg)
{
		cout << "Control system connection lost" << endl;
		ControlSystem.StartServer("5690");
		return 0;
}


