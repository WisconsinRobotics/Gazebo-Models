#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <stdio.h>
#include <gazebo/sensors/sensors.hh>
#include <vector>
#include "UdpSocket.hpp"
#include "CommonJAUS.hpp"
#include "Sensors/LRF/LaserRangeFinder.h"

typedef int  _socket_t;

LaserRangeFinder lrf;
Socket::UdpSocket port(20000);

uint8_t testing = 1; 


namespace gazebo
{


class InsomniaPlugin : public ModelPlugin
{

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{

	// Store the pointer to the model
	this->model = _parent;


    // initialize the lrf
    if (!lrf.Initialize())
    {
        std::cout << "Failed to open the LRF!" << std::endl;
        return;
    }

		
	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&InsomniaPlugin::OnUpdate, this, _1));

	if(!port.Open())
	{
		std::cout << "Failed to open port!" << std::endl;
	}
	
}

// Called by the world update start event
public: void OnUpdate(const common::UpdateInfo & /*_info*/)
{
	//Check for data
		//no? use old commands
		//yes? get new commands

	//Recieve from port (from awake)
	std::vector<uint8_t> buffer;

    if(!testing)
    {
	    //if(!ReadJAUSMessage(port, buffer))
	    //{
	    //	return;
	    //}
	    if(port.Read(buffer, sizeof(buffer), nullptr) != JAUS_DRIVE_MESSAGE_TOTAL_SIZE)
	    {
	    	return;
	    }
    }
    else
    {
        TestJAUSPacket(buffer)
    }

	//unpack JAUS packet into gazebo movements
    MoveRobot(buffer);

    std::vector<uint8_t> lrfData;
    // grab the data, serialize, and send it out
	lrf.RefreshData();
    lrf.DataPacker(lrfData);
	if(!port.Write(lrfData,sizeof(lrfData), nullptr))
    {
        std::cout << "Failed to send buffer to listener!" << std::endl;
        break;
    }
}

private: void TestJAUSPacket(std::vector<uint8_t>& buffer)
{
	buffer.push_back(DLE_BYTE);
	buffer.push_back(MOTOR_CONTROLLER_ID);
	buffer.push_back(0x0B);
	buffer.push_back(SET_SPEED_CMD);
	for(int k = 4; k < 10; k++)
	{
		buffer.push_back(0x05);
	}
	buffer.push_back(ETX_BYTE);
}

private: bool ReadJAUSMessage(Socket::UdpSocket port, std::vector<uint8_t>& buffer)
{
	//buffer.reserve(JAUS_DRIVE_MESSAGE_TOTAL_SIZE);
	if(port.Read(buffer, sizeof(buffer), nullptr) != JAUS_DRIVE_MESSAGE_TOTAL_SIZE)
	{
		return false;
	}

    return true;
}

private: void MoveRobot(std::vector<uint8_t> buffer)
{
	int lfWlVel = 0;
	int rtWlVel = 0;
	int armTrnTblVel = 0;
	int armShldrVel = 0;
	int armElbwVel = 0;
	int armWrstVel = 0;
	int armClwRotVel = 0;
	int armClwGrpVel = 0;

	if(buffer[0] == DLE_BYTE && buffer[1] == MOTOR_CONTROLLER_ID)
	{
		switch(buffer[3])
		{
			case SET_SPEED_CMD:
			{
				uint8_t ltMag = buffer[WHEEL_LEFT_INDEX] & 0x7F;
				uint8_t ltDir = buffer[WHEEL_LEFT_INDEX] >> 7;
				uint8_t rtMag = buffer[WHEEL_RIGHT_INDEX] & 0x7F;
				uint8_t rtDir = buffer[WHEEL_RIGHT_INDEX] >> 7;

				lfWlVel = ltDir ? -ltMag : ltMag;
				rtWlVel = rtDir ? rtMag : -rtMag;

				break;
			}
			case SET_ACTUATORS_CMD:
            {
			    uint8_t trnTblMag = buffer[ ARM_TURNTABLE_INDEX ] & 0x7F;
				uint8_t shldrMag  = buffer[ ARM_SHOULDER_INDEX  ] & 0x7F;
				uint8_t elbwMag   = buffer[ ARM_ELBOW_INDEX     ] & 0x7F;
				uint8_t wrstMag   = buffer[ ARM_WRIST_INDEX     ] & 0x7F;
				uint8_t clwRotMag = buffer[ ARM_CLAWROT_INDEX   ] & 0x7F;
				uint8_t clwGrpMag = buffer[ ARM_CLAWGRIP_INDEX  ] & 0x7F;

				uint8_t trnTblDir = buffer[ ARM_TURNTABLE_INDEX ] >> 7;
				uint8_t shldrDir  = buffer[ ARM_SHOULDER_INDEX  ] >> 7;
				uint8_t elbwDir   = buffer[ ARM_ELBOW_INDEX     ] >> 7;
				uint8_t wrstDir   = buffer[ ARM_WRIST_INDEX     ] >> 7;
				uint8_t clwRotDir = buffer[ ARM_CLAWROT_INDEX   ] >> 7;
				uint8_t clwGrpDir = buffer[ ARM_CLAWGRIP_INDEX  ] >> 7;
			
				armTrnTblVel = trnTblDir ? trnTblMag : -trnTblMag;
				armShldrVel  = shldrDir ? shldrMag: -shldrMag;
				armElbwVel   = elbwDir ? elbwMag : -elbwMag;
				armWrstVel   = wrstDir ? wrstMag : -wrstMag;
				armClwRotVel = clwRotDir ? clwRotMag : -clwRotMag;
				armClwGrpVel = clwGrpDir ? -clwGrpMag : clwGrpMag;

				break;
			}
			default:
                // do nothing
				break;

		}
	}

	//torque to be constant

	this->model->GetJoint("BogieLeft-Wheel0")->SetVelocity(0,lfWlVel);	//left	pos=forward
	this->model->GetJoint("BogieLeft-Wheel1")->SetVelocity(0,lfWlVel);	//left
	this->model->GetJoint("RockerLeft-Wheel2")->SetVelocity(0,lfWlVel);	//left
	this->model->GetJoint("BogieRight-Wheel3")->SetVelocity(0,rtWlVel);	//right	neg=forward
	this->model->GetJoint("BogieRight-Wheel4")->SetVelocity(0,rtWlVel);	//right
	this->model->GetJoint("RockerRight-Wheel5")->SetVelocity(0,rtWlVel);	//right

	this->model->GetJoint("Frame-ArmBase")->SetVelocity(0,armTrnTblVel);  //negative = clockwise
	this->model->GetJoint("ArmBase-Humerus")->SetVelocity(0,armShldrVel); //positive = up
	this->model->GetJoint("Humerus-Forearm")->SetVelocity(0,armElbwVel);  //positive = up
	this->model->GetJoint("Forearm-Wrist")->SetVelocity(0,armWrstVel);    //positive = up
	this->model->GetJoint("Wrist-Claw")->SetVelocity(0,armClwRotVel);     //negative = clockwise
	this->model->GetJoint("Claw-Jaw")->SetVelocity(0,armClwGrpVel);       //positive = closed

}


// Pointer to the model
private: physics::ModelPtr model;

//pointer to the laserSensor
private: sensors::RaySensorPtr raySensor;

// Pointer to the update event connection
private: event::ConnectionPtr updateConnection;
};




// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(InsomniaPlugin)
}
