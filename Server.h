#ifndef SERVER_H_
#define SERVER_H_

#include "Aris_Message.h"
#include "Aris_Socket.h"
#include "Gait.h"
 

extern bool Is_CS_Connected;

enum Client_Msg

{
	CS_Connected=0,
	CS_CMD_Received=1,

//	TRAJ_Finished,

	CS_Lost=2,
	VS_Connected=3,
	VS_MAP_Received=4,
	VS_Lost=5,
};

//register is in ac_test
enum RobotCMD_Msg
{
	GetControlCommand=100,
	RT_Data_Received=10000,

};

/*enum COMMAND
{
	Enable,
	Disable,
	Home,
	Standstill,
	Startgait,
	Back,
	Stop,
    AdaptiveWalk,
    BeginDiscover,
    EndDiscover,
    SIT,
};*/

//CS
//MSG callback
int On_CS_Connected(Aris::Core::MSG &msg);
int On_CS_CMD_Received(Aris::Core::MSG &msg);
int On_CS_Lost(Aris::Core::MSG &msg);
int On_RT_DataReceived(Aris::Core::MSG &data);

// CONN callback
int On_CS_ConnectionReceived(Aris::Core::CONN *pConn, const char* addr,int port);
int On_CS_DataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data);
int On_CS_ConnectionLost(Aris::Core::CONN *pConn);

//VS
//MSG callback
int On_VS_Connected(Aris::Core::MSG &msg);
int ON_VS_Map_Received(Aris::Core::MSG &msg);
int On_VS_Lost(Aris::Core::MSG &msg);

// CONN callback
int On_VS_ConnectionReceived(Aris::Core::CONN *pConn, const char* addr,int port);
int On_VS_DataReceived(Aris::Core::CONN *pConn, Aris::Core::MSG &data);
int On_VS_ConnectionLost(Aris::Core::CONN *pConn);
 

extern Aris::Core::CONN ControlSystem;
extern Aris::Core::CONN VisualSystem;
#endif
