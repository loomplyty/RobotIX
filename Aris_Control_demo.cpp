/*
 * ac_test.cpp

 *
 *  Created on: Nov 26, 2014
 *      Author: leo
 */
#include <iostream>
//#include "Aris_ControlData.h"
#include "Aris_Control.h"
#include "Aris_Message.h"
#include "Aris_Thread.h"
#include "Aris_Socket.h"
#include "Server.h"


#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <xcommunication/legacydatapacket.h>
#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/enumerateusbdevices.h>
#include <xcommunication/mtwsdidata.h>

#include "deviceclass.h"

#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>


#ifdef __GNUC__
#include "conio.h" // for non ANSI _kbhit() and _getch()
#else
#include <conio.h>
#endif

using namespace std;
using namespace Aris::RT_CONTROL;

static CGait gait;
static EGAIT gaitcmd[AXIS_NUMBER];
static EGAIT gaitcmdtemp[AXIS_NUMBER];

Aris::RT_CONTROL::ACTUATION cs;
//CSysBase sysbase;
Aris::RT_CONTROL::CSysInitParameters initParam;
int Count;

 enum MACHINE_CMD
 {
     NOCMD=1000,
	 POWEROFF=1001,
	 STOP=1002,
	 ENABLE=1003,
	 RUNNING=1004,
	 GOHOME_1=1005,
	 GOHOME_2=1006,
	 HOME2START_1=1007,
	 HOME2START_2=1008,
 	 FORWARD=1009,
	 BACKWARD=1010,
	 FAST_FORWARD=1011,
	 FAST_BACKWARD=1012,
	 LEGUP=1013,
	 TURNLEFT=1014,
	 TURNRIGHT=1015,
	 ONLINEGAIT=1016,
	 TOSTANDSTILL=1017
 };


/*
 * Trajectory Generator
 */

 void* Thread2(void *)
 {
	 cout<<"running msgloop"<<endl;
 	Aris::Core::RunMsgLoop();
 	return NULL;
 };


void* Thread_IMU(void *)
{
	DeviceClass device;

	try
	{
		// Scan for connected USB devices
		std::cout << "Scanning for USB devices..." << std::endl;
		XsPortInfoArray portInfoArray;
		xsEnumerateUsbDevices(portInfoArray);
 		if (!portInfoArray.size())
		{
			std::string portNumber="/dev/ttyUSB0";
			int baudRate=921600;

			std::cout << "No USB Motion Tracker found." << std::endl << std::endl
					<< "COM port name set as "<<portNumber <<std::endl
					<< "Baud rate set as "<<baudRate<<std::endl<<std::endl;

			XsPortInfo portInfo(portNumber, XsBaud::numericToRate(baudRate));
			portInfoArray.push_back(portInfo);
		}

		// Use the first detected device
		XsPortInfo mtPort = portInfoArray.at(0);

		// Open the port with the detected device
		std::cout << "Opening port..." << std::endl;
		if (!device.openPort(mtPort))
			throw std::runtime_error("Could not open port. Aborting.");

		// Put the device in configuration mode
		std::cout << "Putting device into configuration mode..." << std::endl;
		if (!device.gotoConfig()) // Put the device into configuration mode before configuring the device
		{
			throw std::runtime_error("Could not put device into configuration mode. Aborting.");
		}

		// Request the device Id to check the device type
		mtPort.setDeviceId(device.getDeviceId());

		// Check if we have an MTi / MTx / MTmk4 device
		if (!mtPort.deviceId().isMt9c() && !mtPort.deviceId().isMtMk4())
		{
			throw std::runtime_error("No MTi / MTx / MTmk4 device found. Aborting.");
		}
		std::cout << "Found a device with id: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << std::endl;

		try
		{
			// Print information about detected MTi / MTx / MTmk4 device
			std::cout << "Device: " << device.getProductCode().toStdString() << " opened." << std::endl;

			// Configure the device. Note the differences between MTix and MTmk4
			std::cout << "Configuring the device..." << std::endl;
			if (mtPort.deviceId().isMt9c())
			{
				XsOutputMode outputMode = XOM_Orientation; // output orientation data
				XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion; // output orientation data as quaternion

				// set the device configuration
				if (!device.setDeviceMode(outputMode, outputSettings))
				{
					throw std::runtime_error("Could not configure MT device. Aborting.");
				}
			}
			else if (mtPort.deviceId().isMtMk4())
			{
				XsOutputConfiguration quat_Q(XDI_Quaternion, 100);
				XsOutputConfiguration quat_DQ(XDI_DeltaQ, 100);
				XsOutputConfiguration quat_DV(XDI_DeltaV, 100);
				XsOutputConfiguration quat_ACC(XDI_Acceleration, 100);
				XsOutputConfigurationArray configArray;
				configArray.push_back(quat_Q);
				configArray.push_back(quat_DQ);
				configArray.push_back(quat_DV);
				configArray.push_back(quat_ACC);
				if (!device.setOutputConfiguration(configArray))
				{

					throw std::runtime_error("Could not configure MTmk4 device. Aborting.");
				}
			}
			else
			{
				throw std::runtime_error("Unknown device while configuring. Aborting.");
			}

			// Put the device in measurement mode
			std::cout << "Putting device into measurement mode..." << std::endl;
			if (!device.gotoMeasurement())
			{
				throw std::runtime_error("Could not put device into measurement mode. Aborting.");
			}

			std::cout << "\nMain loop (press any key to quit)" << std::endl;
			std::cout << std::string(79, '-') << std::endl;

			XsByteArray data;
			XsMessageArray msgs;
			while (!_kbhit())
			{
				device.readDataToBuffer(data);
				device.processBufferedData(data, msgs);
				for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
				{
					// Retrieve a packet
					XsDataPacket packet;
					if ((*it).getMessageId() == XMID_MtData) {
						LegacyDataPacket lpacket(1, false);
						lpacket.setMessage((*it));
						lpacket.setXbusSystem(false, false);
						lpacket.setDeviceId(mtPort.deviceId(), 0);
						lpacket.setDataFormat(XOM_Orientation, XOS_OrientationMode_Quaternion,0);	//lint !e534
						XsDataPacket_assignFromXsLegacyDataPacket(&packet, &lpacket, 0);
					}
					else if ((*it).getMessageId() == XMID_MtData2) {
						packet.setMessage((*it));
						packet.setDeviceId(mtPort.deviceId());
					}

					// Get the quaternion data
					bool a0=packet.containsOrientation();
					//std::cout<<"a0:"<<a0;
					XsQuaternion quaternion = packet.orientationQuaternion();
/*
					std::cout << "\r"
						<< "W:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.m_w
						<< ",X:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.m_x
						<< ",Y:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.m_y
						<< ",Z:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.m_z
					;
*/
					// Get deltaQ
					bool a1=packet.containsSdiData();
					//std::cout<<"a1:"<<a1;
					XsSdiData deltaParam=packet.sdiData();
					XsQuaternion deltaQ=deltaParam.orientationIncrement();
					XsVector deltaV=deltaParam.velocityIncrement();

				//	std::cout<<"\r"
				//		<< "dQ W:" << std::setw(5) << std::fixed << std::setprecision(2) << deltaQ.m_w
				//		<< ",dQ X:" << std::setw(5) << std::fixed << std::setprecision(2) << deltaQ.m_x
				//		<< ",dQ Y:" << std::setw(5) << std::fixed << std::setprecision(2) << deltaQ.m_y
				//		<< ",dQ Z:" << std::setw(5) << std::fixed << std::setprecision(2) << deltaQ.m_z
				//	;
					/*
					std::cout<<"\r"
						<< "dV X:" << std::setw(5) << std::fixed << std::setprecision(2) << deltaV[0]
						<< ",dV Y:" << std::setw(5) << std::fixed << std::setprecision(2) << deltaV[1]
						<< ",dV Z:" << std::setw(5) << std::fixed << std::setprecision(2) << deltaV[2]
					;
*/
					// Convert packet to euler
					XsEuler euler = packet.orientationEuler();
					double angle[3];
					angle[0]=euler.m_x/180*PI;
					angle[1]=euler.m_y/180*PI;
					angle[2]=euler.m_z/180*PI;

/*
					std::cout
						<< ",Roll:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.m_roll
						<< ",Pitch:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.m_pitch
						<< ",Yaw:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.m_yaw
					;
*/
					// Get angular velocity

					double angularVelocity[3];
					angularVelocity[0] = deltaQ[1] / 0.01 * 2.0;
					angularVelocity[1] = deltaQ[2] / 0.01 * 2.0;
					angularVelocity[2] = deltaQ[3] / 0.01 * 2.0;


				//	std::cout
				//		<< "aVel X:" << std::setw(7) << std::fixed << std::setprecision(2) << angularVelocity[0]
				//		<< ",aVel Y:" << std::setw(7) << std::fixed << std::setprecision(2) << angularVelocity[1]
				//		<< ",aVel Z:" << std::setw(7) << std::fixed << std::setprecision(2) << angularVelocity[2]
				//	;


					// Get acceleration
					//bool a2=packet.containsCalibratedAcceleration();
					//std::cout<<"a2:"<<a2<<std::endl;
					XsVector acceleration=packet.calibratedAcceleration();
					double linearAcc[3];
					XsVector(acceleration,linearAcc,3);



 		 	 	//gait.GetIMUData(angle,angularVelocity,linearAcc);



//					printf("before POST MSG\n");

				 	Aris::Core::MSG m;
					m.SetMsgID(1035);
					m.SetLength(sizeof(double)*9);
					m.CopyAt(angle,sizeof(double)*3,0);
					m.CopyAt(angularVelocity,sizeof(double)*3,sizeof(double)*3);
					m.CopyAt(linearAcc,sizeof(double)*3,sizeof(double)*6);

 					cs.NRT_PostMsg(m);

	           //  cout<<"ANgle of IMU"<<angle[0]<<"  "<<angle[1]<<"  "<<angle[2]<<endl;
/*
					std::cout
							<< "Acc X:" << std::setw(7) << std::fixed << std::setprecision(2) << linearAcc[0]
							<< ",Acc Y:" << std::setw(7) << std::fixed << std::setprecision(2) << linearAcc[1]
							<< ",Acc Z:" << std::setw(7) << std::fixed << std::setprecision(2) << linearAcc[2]
					;
*/
					std::cout << std::flush;
				}

				msgs.clear();
				XsTime::msleep(0);
			}
			_getch();
			std::cout << "\n" << std::string(79, '-') << "\n";
			std::cout << std::endl;
		}
		catch (std::runtime_error const & error)
		{
			std::cout << error.what() << std::endl;
		}
		catch (...)
		{
			std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		}

		// Close port
		std::cout << "Closing port..." << std::endl;
		device.close();
	}
	catch (std::runtime_error const & error)
	{
		std::cout << error.what() << std::endl;
	}
	catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
	}

	std::cout << "Successful exit." << std::endl;

	std::cout << "Press [ENTER] to continue." << std::endl; std::cin.get();

	return 0;
}

int initFun(Aris::RT_CONTROL::CSysInitParameters& param)
{
	gait.InitGait(param);
	return 0;
};
int tg(Aris::RT_CONTROL::CMachineData& machineData,Aris::Core::RT_MSG& msgRecv,Aris::Core::RT_MSG& msgSend)
{

	static int tg_count;
	tg_count++;
	if(tg_count%1000==000)
	{
 		    msgSend.SetMsgID(RT_Data_Received);

	    	msgSend.SetLength(sizeof(int)*18*5);
	    	//states , mode, pos, vel, current

	    	for(int i=0;i<18;i++)
	    	{
	    		msgSend.CopyAt(&machineData.motorsStates[i],sizeof(int),i*5);
	    		msgSend.CopyAt(&machineData.motorsModesDisplay[i],sizeof(int),i*5+1);
		    	msgSend.CopyAt(&machineData.feedbackData[0].Position,sizeof(int),i*5+2);
		    	msgSend.CopyAt(&machineData.feedbackData[0].Position,sizeof(int),i*5+3);
		    	msgSend.CopyAt(&machineData.feedbackData[0].Position,sizeof(int),i*5+4);
	    	}


		 //     rt_printf("ty give msg id %d data length %d \n",msgSend.GetMsgID(),msgSend.GetLength());
 		//  rt_machine_msg.SetMsgID(1000);
		//  rt_machine_msg.Copy(&machineData,sizeof(machineData));

	         cs.RT_PostMsg(msgSend);
	}


  	if(tg_count==2000)
	{
 	 /*  ofstream input_static;
	    input_static.open("./static.txt");

	 	CTrotGait trot;
	 	double foot_pos[18];
	 	double screw_pos[18];
	 	double body[6]={0, 0, 0, 0, 0, 0};

	 	trot.LoadRobot();
	 	double a,b,c,d,e;
	 	a=1;
	 	b=3;
	 	c=0.1;
	 	d=0.1;
	 	e=0.5;
	 	trot.SetGaitParas(a,b,c,d,e);
	 	int N=1000*(trot.m_raiseMidLegsTime*2+trot.m_period*2);
	 	cout<<"gait length"<<N<<endl;
	 	for(int i=1;i<=N;i++)
	 	{
	 		trot.CalPee(i,foot_pos,body);
	 		trot.CalPin(screw_pos);


	 		input_static<<foot_pos[0]<<"\t";
	 		input_static<<foot_pos[1]<<"\t";
	 		input_static<<foot_pos[2]<<"\t";

	 		input_static<<foot_pos[12]<<"\t";
	 		input_static<<foot_pos[13]<<"\t";
	 		input_static<<foot_pos[14]<<"\t";

	 		input_static<<body[3]<<"\t";
	 		input_static<<body[4]<<"\t";
	 		input_static<<body[5]<<"\t";



	 		input_static<<endl;

	 	}
	 	input_static.close();*/




 	  /*   ofstream input;
 	    input.open("./input.txt");

		 for(int i=0;i<8000;i++)
		{
			input<<i<<"\t";
 			gait.online_DoPID(i,machineData);
 			input<<gait.online_ideal_foot_pos_before_PID[0]<<"\t";
			input<<gait.online_ideal_foot_pos_before_PID[1]<<"\t";
			input<<gait.online_ideal_foot_pos_before_PID[2]<<"\t";
			input<<gait.online_ideal_foot_pos[0]<<"\t";
			input<<gait.online_ideal_foot_pos[1]<<"\t";
			input<<gait.online_ideal_foot_pos[2]<<"\t";

		 	input<<gait.online_ideal_screw_pos_before_PID[0]<<"\t";
		 	input<<gait.online_ideal_screw_pos_before_PID[1]<<"\t";
		 	input<<gait.online_ideal_screw_pos_before_PID[2]<<"\t";
		 	input<<gait.online_ideal_screw_pos[0]<<"\t";
		 	input<<gait.online_ideal_screw_pos[1]<<"\t";
		 	input<<gait.online_ideal_screw_pos[2]<<"\t";

		 //	input<<gait.online_ideal_screw_pos[15]<<"\t";
		 //	input<<gait.online_ideal_screw_pos[16]<<"\t";
		 //	input<<gait.online_ideal_screw_pos[17]<<"\t";
		//	input<<gait.online_ideal_foot_pos_before_PID[6]<<"\t";
		//	input<<gait.online_ideal_foot_pos_before_PID[7]<<"\t";
		//	input<<gait.online_ideal_foot_pos_before_PID[8]<<"\t";
		//	input<<gait.online_ideal_foot_pos[6]<<"\t";
		//	input<<gait.online_ideal_foot_pos[7]<<"\t";
		//	input<<gait.online_ideal_foot_pos[8]<<"\t";

		// 	input<<gait.online_ideal_screw_pos_before_PID[6]<<"\t";
		 //	input<<gait.online_ideal_screw_pos_before_PID[7]<<"\t";
		 //	input<<gait.online_ideal_screw_pos_before_PID[8]<<"\t";
		 //	input<<gait.online_ideal_screw_pos[6]<<"\t";
		 //	input<<gait.online_ideal_screw_pos[7]<<"\t";
		// 	input<<gait.online_ideal_screw_pos[8]<<"\t";

		 	//input<<gait.online_angle[0]<<"\t";
		 //	input<<gait.online_angle[1]<<"\t";
		 //	input<<gait.online_angle[2]<<"\t";
		 	//input<<gait.online_angleVel[0]<<"\t";
		 //	input<<gait.online_angleVel[1]<<"\t";
		 	//input<<gait.online_angleVel[2]<<"\t";
		 //	input<<gait.online_IMU_313[0]<<"\t";
		//	input<<gait.online_IMU_313[1]<<"\t";
		 //	input<<gait.online_IMU_313[2]<<"\t";



		 //	input<<gait.online_ideal_screw_vel[0]<<"\t";
		 //	input<<gait.online_ideal_screw_vel[1]<<"\t";
		 //	input<<gait.online_ideal_screw_vel[2]<<"\t";

		 //	input<<gait.online_last_ideal_screw_vel[0]<<"\t";
		 //	input<<gait.online_last_ideal_screw_vel[1]<<"\t";
		 //	input<<gait.online_last_ideal_screw_vel[2]<<"\t";


			input<<endl;
		}



	//	input.close();*/

	}

	//rt_printf("angle 313 %f %f %f\n",gait.online_angle[0],gait.online_angle[1],gait.online_angle[2]);
	// for(int i = 0; i < 3; i++)
	// rt_printf("Linear Acc[%d] = %.3lf   ", machineData.IMUData.LinearAccleration[i]);
	// rt_printf("\n");

	const int MapAbsToPhy[18]=
	{
			10,	11,	9,
			12,	14,	13,
			17,	15,	16,
			6,	8,	7,
			3,	5,	4,
			0,	2,	1
	};
	const int MapPhyToAbs[18]=
	{
			15,	17,	16,
			12,	14,	13,
			9,	11,	10,
			2,	0,	1,
			3,	5,	4,
			7,	8,	6
	};

 	int CommandID;

	 CommandID=msgRecv.GetMsgID();
 	switch(CommandID)
	{
	case NOCMD:
		for(int i=0;i<18;i++)
		{
	    	machineData.motorsCommands[i]=EMCMD_NONE;

		}
		rt_printf("NONE Command Get in NRT\n" );

	break;

	case ENABLE:
		 for(int i=0;i<18;i++)
		 {
			machineData.motorsCommands[i]=EMCMD_ENABLE;
		 }
 		rt_printf("ENABLE Command Get in NRT\n" );

		break;
	case POWEROFF:
		 for(int i=0;i<18;i++)
		 {
			machineData.motorsCommands[i]=EMCMD_POWEROFF;
			gait.IfReadytoSetGait(false,i);
		 }
		rt_printf("POWEROFF Command Get in NRT\n" );

		break;
	case STOP:
		 for(int i=0;i<18;i++)
		{
			machineData.motorsCommands[i]=EMCMD_STOP;
 		}
 		 rt_printf("STOP Command Get in NRT\n" );

		break;
	case RUNNING:
	 	for(int i=0;i<18;i++)
	 	{
			machineData.motorsCommands[i]=EMCMD_RUNNING;
			gait.IfReadytoSetGait(true,i);
	 	}
		rt_printf("RUNNING Command Get in NRT\n" );
		break;

	case GOHOME_1:

			machineData.motorsCommands[MapAbsToPhy[0]]=EMCMD_GOHOME;
			machineData.motorsCommands[MapAbsToPhy[1]]=EMCMD_GOHOME;
			machineData.motorsCommands[MapAbsToPhy[2]]=EMCMD_GOHOME;
			machineData.motorsCommands[MapAbsToPhy[6]]=EMCMD_GOHOME;
			machineData.motorsCommands[MapAbsToPhy[7]]=EMCMD_GOHOME;
			machineData.motorsCommands[MapAbsToPhy[8]]=EMCMD_GOHOME;
			machineData.motorsCommands[MapAbsToPhy[12]]=EMCMD_GOHOME;
			machineData.motorsCommands[MapAbsToPhy[13]]=EMCMD_GOHOME;
			machineData.motorsCommands[MapAbsToPhy[14]]=EMCMD_GOHOME;

 			gaitcmd[MapAbsToPhy[0]]=EGAIT::GAIT_HOME;
 			gaitcmd[MapAbsToPhy[1]]=EGAIT::GAIT_HOME;
 			gaitcmd[MapAbsToPhy[2]]=EGAIT::GAIT_HOME;
 			gaitcmd[MapAbsToPhy[6]]=EGAIT::GAIT_HOME;
 			gaitcmd[MapAbsToPhy[7]]=EGAIT::GAIT_HOME;
 			gaitcmd[MapAbsToPhy[8]]=EGAIT::GAIT_HOME;
 			gaitcmd[MapAbsToPhy[12]]=EGAIT::GAIT_HOME;
 			gaitcmd[MapAbsToPhy[13]]=EGAIT::GAIT_HOME;
 			gaitcmd[MapAbsToPhy[14]]=EGAIT::GAIT_HOME;

		rt_printf("GOHOME_1 Command Get in NRT\n" );

		break;

	case GOHOME_2:

		machineData.motorsCommands[MapAbsToPhy[3]]=EMCMD_GOHOME;
		machineData.motorsCommands[MapAbsToPhy[4]]=EMCMD_GOHOME;
		machineData.motorsCommands[MapAbsToPhy[5]]=EMCMD_GOHOME;
		machineData.motorsCommands[MapAbsToPhy[9]]=EMCMD_GOHOME;
		machineData.motorsCommands[MapAbsToPhy[10]]=EMCMD_GOHOME;
		machineData.motorsCommands[MapAbsToPhy[11]]=EMCMD_GOHOME;
		machineData.motorsCommands[MapAbsToPhy[15]]=EMCMD_GOHOME;
		machineData.motorsCommands[MapAbsToPhy[16]]=EMCMD_GOHOME;
		machineData.motorsCommands[MapAbsToPhy[17]]=EMCMD_GOHOME;

			gaitcmd[MapAbsToPhy[3]]=EGAIT::GAIT_HOME;
			gaitcmd[MapAbsToPhy[4]]=EGAIT::GAIT_HOME;
			gaitcmd[MapAbsToPhy[5]]=EGAIT::GAIT_HOME;
			gaitcmd[MapAbsToPhy[9]]=EGAIT::GAIT_HOME;
			gaitcmd[MapAbsToPhy[10]]=EGAIT::GAIT_HOME;
			gaitcmd[MapAbsToPhy[11]]=EGAIT::GAIT_HOME;
			gaitcmd[MapAbsToPhy[15]]=EGAIT::GAIT_HOME;
			gaitcmd[MapAbsToPhy[16]]=EGAIT::GAIT_HOME;
			gaitcmd[MapAbsToPhy[17]]=EGAIT::GAIT_HOME;


		rt_printf("GOHOME_2 Command Get in NRT\n" );

		break;

	case HOME2START_1:

		if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
		{
		    for(int i=0;i<18;i++)
		    {
				machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
		    }
				gaitcmd[MapAbsToPhy[0]]=EGAIT::GAIT_HOME2START;
				gaitcmd[MapAbsToPhy[1]]=EGAIT::GAIT_HOME2START;
				gaitcmd[MapAbsToPhy[2]]=EGAIT::GAIT_HOME2START;
				gaitcmd[MapAbsToPhy[6]]=EGAIT::GAIT_HOME2START;
				gaitcmd[MapAbsToPhy[7]]=EGAIT::GAIT_HOME2START;
				gaitcmd[MapAbsToPhy[8]]=EGAIT::GAIT_HOME2START;
				gaitcmd[MapAbsToPhy[12]]=EGAIT::GAIT_HOME2START;
				gaitcmd[MapAbsToPhy[13]]=EGAIT::GAIT_HOME2START;
				gaitcmd[MapAbsToPhy[14]]=EGAIT::GAIT_HOME2START;

			rt_printf("HOME2START_1 Command Get in NRT\n" );
		}

		break;

	case HOME2START_2:

		if(gait.m_gaitState[MapAbsToPhy[3]]==GAIT_STOP)
		{
		    for(int i=0;i<18;i++)
		    {
				machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
		    }

			gaitcmd[MapAbsToPhy[3]]=EGAIT::GAIT_HOME2START;
			gaitcmd[MapAbsToPhy[4]]=EGAIT::GAIT_HOME2START;
			gaitcmd[MapAbsToPhy[5]]=EGAIT::GAIT_HOME2START;
			gaitcmd[MapAbsToPhy[9]]=EGAIT::GAIT_HOME2START;
			gaitcmd[MapAbsToPhy[10]]=EGAIT::GAIT_HOME2START;
			gaitcmd[MapAbsToPhy[11]]=EGAIT::GAIT_HOME2START;
			gaitcmd[MapAbsToPhy[15]]=EGAIT::GAIT_HOME2START;
	 		gaitcmd[MapAbsToPhy[16]]=EGAIT::GAIT_HOME2START;
			gaitcmd[MapAbsToPhy[17]]=EGAIT::GAIT_HOME2START;

			rt_printf("HOME2START_2 Command Get in NRT\n" );
		}

		break;

 	case FORWARD:
	    for(int i=0;i<18;i++)
		 {
 			machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
			gaitcmdtemp[i]=EGAIT::GAIT_MOVE;
			machineData.motorsCommands[i]=EMCMD_RUNNING;


		 if(gait.m_gaitState[i]!=GAIT_STOP)
		 {
     	   gait.Gait_iter[i]=gait.Gait_iter[i]+1;
		 }
		 else
		 {
				gaitcmd[i]=gaitcmdtemp[i];
				gait.Gait_iter[i]=1;
			}

		 }

	   // rt_printf("driver 0 gaitcmd:%d\n",gaitcmd[0]);


		break;
 	case BACKWARD:
	    for(int i=0;i<18;i++)
		 {
 			machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
			gaitcmdtemp[i]=EGAIT::GAIT_MOVE_BACK;
			machineData.motorsCommands[i]=EMCMD_RUNNING;


		 if(gait.m_gaitState[i]!=GAIT_STOP)
		 {
     	   gait.Gait_iter[i]=gait.Gait_iter[i]+1;
		 }
		 else
		 {
				gaitcmd[i]=gaitcmdtemp[i];
				gait.Gait_iter[i]=1;
			}

		 }

	   // rt_printf("driver 0 gaitcmd:%d\n",gaitcmd[0]);

		break;

 	case FAST_FORWARD:
	    for(int i=0;i<18;i++)
		 {
 			machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
			gaitcmdtemp[i]=EGAIT::GAIT_FAST_MOVE;
			machineData.motorsCommands[i]=EMCMD_RUNNING;


		 if(gait.m_gaitState[i]!=GAIT_STOP)
		 {
     	   gait.Gait_iter[i]=gait.Gait_iter[i]+1;
		 }
		 else
		 {
				gaitcmd[i]=gaitcmdtemp[i];
				gait.Gait_iter[i]=1;
			}

		 }

	   // rt_printf("driver 0 gaitcmd:%d\n",gaitcmd[0]);


		break;
 	case FAST_BACKWARD:
	    for(int i=0;i<18;i++)
		 {
 			machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
			gaitcmdtemp[i]=EGAIT::GAIT_FAST_MOVE_BACK;
			machineData.motorsCommands[i]=EMCMD_RUNNING;


		 if(gait.m_gaitState[i]!=GAIT_STOP)
		 {
     	   gait.Gait_iter[i]=gait.Gait_iter[i]+1;
		 }
		 else
		 {
				gaitcmd[i]=gaitcmdtemp[i];
				gait.Gait_iter[i]=1;
			}

		 }

		break;
	case LEGUP:

		if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
		{
		    for(int i=0;i<18;i++)
		    {
				machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
				gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_LEGUP;
				machineData.motorsCommands[i]=EMCMD_RUNNING;

		    }
		}
		break;
	case TURNLEFT:

		if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
		{
		    for(int i=0;i<18;i++)
		    {
				machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
				gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_TURN_LEFT;
				machineData.motorsCommands[i]=EMCMD_RUNNING;

		    }
		}
		break;
	case TURNRIGHT:

		if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
		{
		    for(int i=0;i<18;i++)
		    {
				machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
				gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_TURN_RIGHT;
				machineData.motorsCommands[i]=EMCMD_RUNNING;

		    }
		}
		break;
	case ONLINEGAIT:

	    for(int i=0;i<18;i++)
		 {
 			machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
			gaitcmdtemp[i]=EGAIT::GAIT_ONLINE;
			machineData.motorsCommands[i]=EMCMD_RUNNING;


		 if(gait.m_gaitState[i]!=GAIT_STOP)
		 {
     	   gait.Gait_iter[i]=gait.Gait_iter[i]+1;
		 }
		 else
		 {
				gaitcmd[i]=gaitcmdtemp[i];
				gait.Gait_iter[i]=1;
		}

		 }

	   // rt_printf("driver 0 gaitcmd:%d\n",gaitcmd[0]);


		break;
	case TOSTANDSTILL:

		if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
		{
		    for(int i=0;i<18;i++)
		    {
				machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
				gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_TOSTANDSTILL;
				machineData.motorsCommands[i]=EMCMD_RUNNING;

		    }
		}
		break;

	case 1035:
		//rt_printf("IMU\n\n\n\n");
		msgSend.PasteAt(gait.online_angle,sizeof(double)*3,0);
	    msgSend.PasteAt(gait.online_angleVel,sizeof(double)*3,sizeof(double)*3);
		msgSend.PasteAt(gait.online_linearAcc,sizeof(double)*3,sizeof(double)*6);
	   // rt_printf("IMU raw data %f %f %f\n",gait.online_angle[0],gait.online_angle[1],gait.online_angle[2]);

		break;

		/*if(gait.m_gaitState[MapAbsToPhy[0]]==GAIT_STOP)
		{
		    for(int i=0;i<18;i++)
		    {
				machineData.motorsModes[i]=EOperationMode::OM_CYCLICVEL;
				gaitcmd[MapAbsToPhy[i]]=EGAIT::GAIT_ONLINE;
				machineData.motorsCommands[i]=EMCMD_RUNNING;
		    }
		}
		break;*/

	default:
		//DO NOTHING, CMD AND TRAJ WILL KEEP STILL
 		break;
	}
    // gait.IfReadytoSetGait(machineData.isMotorHomed[0]);
    // rt_printf("driver 0 gaitcmd:%d\n",gaitcmd[0]);

     gait.RunGait(gaitcmd,machineData);

return 0;

};

//offsets driver order
static int HEXBOT_HOME_OFFSETS_RESOLVER[18] =
{
		-15849882+349000,	 -16354509+349000,	 -16354509+349000,
		-15849882+349000,	 -16354509+349000,	 -16354509+349000,
		-15849882+349000,	 -16354509+349000,	 -16354509+349000,
		-16354509+349000,	 -15849882+349000,	 -16354509+349000,
		-15849882+349000,	 -16354509+349000,	 -16354509+349000,
		-16354509+349000,	 -16354509+349000,  -15849882+349000
};

int OnGetControlCommand(Aris::Core::MSG &msg)
{
    int CommandID;
    msg.Paste(&CommandID,sizeof(int));
    Aris::Core::MSG data;

    switch(CommandID)
    {
    case 1:
		data.SetMsgID(POWEROFF);
		int k;
		k=123;
		data.Copy(&k,sizeof(int));
		cs.NRT_PostMsg(data);
		break;
    case 2:
    	data.SetMsgID(STOP);
		cs.NRT_PostMsg(data);
    	break;
    case 3:
    	data.SetMsgID(ENABLE);
		cs.NRT_PostMsg(data);
    	break;
    case 4:
    	data.SetMsgID(RUNNING);
		cs.NRT_PostMsg(data);
    	break;
    case 5:
    	data.SetMsgID(GOHOME_1);
		cs.NRT_PostMsg(data);
    	break;
    case 6:
    	data.SetMsgID(GOHOME_2);
		cs.NRT_PostMsg(data);
    	break;
    case 7:
    	data.SetMsgID(HOME2START_1);
		cs.NRT_PostMsg(data);
    	break;
    case 8:
    	data.SetMsgID(HOME2START_2);
		cs.NRT_PostMsg(data);
    	break;
    case 9:
    	data.SetMsgID(FORWARD);
		cs.NRT_PostMsg(data);
    	break;
    case 10:
    	data.SetMsgID(BACKWARD);
		cs.NRT_PostMsg(data);
    	break;
    case 11:
    	data.SetMsgID(FAST_FORWARD);
		cs.NRT_PostMsg(data);
    	break;
    case 12:
    	data.SetMsgID(FAST_BACKWARD);
		cs.NRT_PostMsg(data);
    	break;
    case 13:
    	data.SetMsgID(LEGUP);
		cs.NRT_PostMsg(data);
    	break;
    case 14:
    	data.SetMsgID(TURNLEFT);
		cs.NRT_PostMsg(data);
    	break;
    case 15:
    	data.SetMsgID(TURNRIGHT);
		cs.NRT_PostMsg(data);
    	break;
    case 16:
    	data.SetMsgID(ONLINEGAIT);
    	cs.NRT_PostMsg(data);
    	break;
    case 17:
    	data.SetMsgID(TOSTANDSTILL);
    	cs.NRT_PostMsg(data);
    	break;


    default:
		printf("Hi! I didn't get validate cmd\n");
        break;

    }
    return CommandID;
};

int On_RT_DataReceived(Aris::Core::MSG &data)
{
	if(Is_CS_Connected==true)
	{
	    printf("Sending data to client,data length: %d\n",data.GetLength());
		ControlSystem.SendData(data);
	}
}
//static int driverIDs[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};

int main(int argc, char** argv)
{

    Aris::Core::RegisterMsgCallback(CS_Connected,On_CS_Connected);
    Aris::Core::RegisterMsgCallback(CS_CMD_Received,On_CS_CMD_Received);
    Aris::Core::RegisterMsgCallback(CS_Lost,On_CS_Lost);
    Aris::Core::RegisterMsgCallback(GetControlCommand,OnGetControlCommand);
    Aris::Core::RegisterMsgCallback(RT_Data_Received,On_RT_DataReceived);

//   CONN call back
	/*设置所有CONN类型的回调函数*/
   ControlSystem.SetCallBackOnReceivedConnection(On_CS_ConnectionReceived);
    //VisualSys.SetCallBackOnReceivedConnection(OnConnectReceived_VS);

	ControlSystem.SetCallBackOnReceivedData(On_CS_DataReceived);
      //  VisualSys.SetCallBackOnReceivedData(OnSysDataReceived_VS);

    ControlSystem.SetCallBackOnLoseConnection(On_CS_ConnectionLost);
      //  VisualSys.SetCallBackOnLoseConnection(OnConnectionLost_VS);

    ControlSystem.StartServer("5690");
     // VisualSys.StartServer("5691");
	Aris::Core::THREAD T1,T2,T3 ;
 //	 T1.SetFunction(Thread1);
	 T2.SetFunction(Thread2);
  // cs.Load_XML_PrintMessage();
     T2.Start(0);

     T3.SetFunction(Thread_IMU);


     //sleep(1);//waiting for IMU uploading
    // if (gait.online_angle[0]!=0||gait.online_angle[1]!=0||gait.online_angle[2]!=0)
      //    {
			cs.SetSysInitializer(initFun);

			cs.SetTrajectoryGenerator(tg);

			//cs.SetModeCycVel();

			initParam.motorNum=18;
			initParam.homeHighSpeed=280000;
			initParam.homeLowSpeed=80000;
			initParam.homeAccel=8000;
			initParam.homeMode=-1;
			initParam.homeTorqueLimit=1000;

			////necessary steps
			initParam.homeOffsets=HEXBOT_HOME_OFFSETS_RESOLVER;
			cs.SysInit(initParam);

			cs.SysInitCommunication();

			cs.SysStart();
		    T3.Start(0);
		    sleep(1);


	        //printf("old %f %f %f\n",gait.online_angle[0],gait.online_angle[1],gait.online_angle[2]);

		//    if (gait.online_angle[0]==0&&gait.online_angle[1]==0&&gait.online_angle[2]==0)
		  //  	cs.SysStop();

			printf("Will start\n");

			/*Aris::Core::MSG msg;
			msg.SetMsgID(100035);
			while(!cs.IsSysStopped())
			{
				if(Count%1000==0)
					cs.NRT_PostMsg(msg);
			}*/
			while(!cs.IsSysStopped())
			{


				Count++;
				sleep(1);
			}
        //  }
       // else
    	    // cout<<"IMU processing data failed !"<<endl;

	 return 0;

};


