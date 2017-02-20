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
#include <math.h>

using ignition::math::Angle;

typedef int  _socket_t;

Socket::UdpSocket client_port(10000);
Socket::UdpSocket lrf_port(20000);
Socket::UdpSocket gps_port(20002);

uint8_t testing = 0; 
int lrf_en = 1;
int gps_en = 0;

int lrf_count = 0;

struct sockaddr_in robot_addr;    

namespace gazebo
{


class InsomniaPlugin : public ModelPlugin
{

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // Store the pointer to the model
    this->model = _parent;

    if ((this->lrf = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(
         gazebo::sensors::SensorManager::Instance()->GetSensor("laser"))) == NULL)
    {
        std::cout << "COULD NOT FIND LASER SENSOR" << std::endl;
        return;
    }

	if ((this->gps = std::dynamic_pointer_cast<gazebo::sensors::GpsSensor>(
		 gazebo::sensors::SensorManager::Instance()->GetSensor("gps"))) == NULL)
	{
		std::cout << "COULD NOT FIND GPS SENSOR" << std::endl;
		return;
	}
		
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&InsomniaPlugin::OnUpdate, this, _1));

    if(!client_port.Open())
    {
        std::cout << "Failed to open client_port!" << std::endl;
    }
	
	if(!lrf_port.Open())
    {
        std::cout << "Failed to open lrf_port!" << std::endl;
    }


    memset(&robot_addr, 0, sizeof(struct sockaddr_in));    
    inet_pton(AF_INET, "192.168.1.21", &(robot_addr.sin_addr));    
    robot_addr.sin_family = AF_INET;    
    robot_addr.sin_port = htons(20001); 		
	
}

// Called by the world update start event
public: void OnUpdate(const common::UpdateInfo & /*_info*/)
{
    uint8_t buffer[256];
	memset(buffer, 0, 256);
    int rd = 0;

    if(!testing)
    {
		
		// drive command
        rd = client_port.Read(&buffer[0], 256, nullptr);
		printf("rd = %d\n", rd);
        //if (rd != 12)
	    //	return;
    }
    else
    {
		// fake drive command
        for(int k = 0; k < 12; k++)
        {
        	buffer[k] = 0x0;
        }
    }

	// move robot
    Drive(buffer);

//    if(lrf_count == 300)
	if(true)
	{
    	std::vector<double> lrfData;
    	this->lrf->Ranges(lrfData);

    	if(lrfData.size() == 0)
        	return;

	    //std::vector<char> payload;

		int len = lrfData.size()*2;

		char payload[len];


		int m = 0;
		for(int l = 0; l < lrfData.size(); l++)
		{
			lrfData.at(l) = lrfData.at(l) * 1000;
			float f = (float)lrfData.at(l);
			uint32_t uint = (uint32_t)(*(uint32_t*)&f);
			uint16_t tmp = (uint16_t)lrfData.at(l);
			//payload[m++] = (char)(tmp >> 56);
			//payload[m++] = (char)(tmp >> 48);
			//payload[m++] = (char)(tmp >> 40);
			//payload[m++] = (char)(tmp >> 32);
			//payload[m++] = (char)(tmp >> 24);
			//payload[m++] = (char)(tmp >> 16);
			payload[m++] = (char)(tmp >> 8);
			payload[m++] = (char)tmp;
			if(l < 6 || l > lrfData.size() - 6)
				printf("%d:%f ",l,lrfData.at(l));
		}
		printf("\n");
/*
		printf("%d %d %d %d %d | %d %d %d %d %d\n", 
			payload[len-5],payload[len-4],payload[len-3],payload[len-2],payload[len-1],
			payload[0],payload[1],payload[2],payload[3],payload[4]);
*/
		if(lrf_en == 1)
		{
			if(!lrf_port.Write(&payload[0], len, (struct sockaddr*)&robot_addr))
			{
				fprintf(stderr, "Failed to send lrf to listener!\n");
				return;
			}
		}

		double latitude_dec = this->gps->Latitude().Degree();
		double longitude_dec = this->gps->Longitude().Degree();

		char latitude_deg = (char)floor(latitude_dec);
		char latitude_min = (char)((latitude_dec - floor(latitude_dec)) * 60.0);
		char latitude_sec = (char)((latitude_min - floor(latitude_min)) * 60.0);
 
		char longitude_deg = (char)(floor(longitude_dec));
		char longitude_min = (char)((longitude_dec - floor(longitude_dec)) * 60.0);
		char longitude_sec = (char)((longitude_min - floor(longitude_min)) * 60.0);
   
		char gpspayload[6] = { latitude_deg, latitude_min, latitude_deg,
		                       longitude_deg, longitude_deg, longitude_deg };

   		if(gps_en == 1)
		{
			if(!gps_port.Write(&gpspayload[0], sizeof(gpspayload), nullptr))
			{
				fprintf(stderr, "Failed to send gps to listener!\n");
				return;
			}
		}

    	lrf_count = 0;
    //for(int i = 0; i < lrfData.size(); i += 20)
    //{
    //    for(int j = 0; j < 5; j++)
    //        std::cout << lrfData.at(i+j) << " | ";
    //    std::cout << std::endl;
    //}
    //std::cout << std::endl << std::endl;
    }


    //lrf_count++;
}

private: void Drive(uint8_t *buffer)
{

    //buffer[0] = buffer[0] < 0xA ? 0 : 0xA;
    //buffer[3] = buffer[3] < 0xA ? 0 : 0xA;
    int lfWlVel = (int)((int8_t)buffer[0]);
    //int lfWlVel = buffer[6] ? (-1 * (int)buffer[0]) : (int)buffer[0];
    //int rtWlVel = buffer[9] ? (int)buffer[3] : (-1 * (int)buffer[3]);
    int rtWlVel = (-1*(int)((int8_t)buffer[1]));
	//printf("lfWlVel = %d | rtWlVel = %d\n",lfWlVel,rtWlVel);
    int armTrnTblVel = 0;
    int armShldrVel = 0;
    int armElbwVel = 0;
    int armWrstVel = 0;
    int armClwRotVel = 0;
    int armClwGrpVel = 0;

	//printf("lf = %d | rt = %d\n", lfWlVel, rtWlVel);

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

private: void MoveRobot(uint8_t *buffer, int length)
{
    buffer[0] = buffer[0] < 0xA ? 0 : 0xA;
    buffer[3] = buffer[3] < 0xA ? 0 : 0xA;
	int lfWlVel = buffer[6] ? (-1 * (int)buffer[0]) : (int)buffer[0];
	int rtWlVel = buffer[9] ? (int)buffer[3] : (-1 * (int)buffer[3]);
	int armTrnTblVel = 0;
	int armShldrVel = 0;
	int armElbwVel = 0;
	int armWrstVel = 0;
	int armClwRotVel = 0;
	int armClwGrpVel = 0;

        if(length != JAUS_DRIVE_MESSAGE_TOTAL_SIZE)
        	return;
/*
	if(buffer[0] == DLE_BYTE && buffer[1] == MOTOR_CONTROLLER_ID)
	{
		switch(buffer[3])
		{
			case SET_SPEED_CMD:
			{
				//uint8_t ltMag = buffer[WHEEL_LEFT_INDEX] & 0x7F;
				//uint8_t ltDir = buffer[WHEEL_LEFT_INDEX] >> 7;
				//uint8_t rtMag = buffer[WHEEL_RIGHT_INDEX] & 0x7F;
				//uint8_t rtDir = buffer[WHEEL_RIGHT_INDEX] >> 7;

				//lfWlVel = ltDir ? -ltMag : ltMag;
				//rtWlVel = rtDir ? rtMag : -rtMag;

                                buffer[4] = buffer[4] < 0xA ? 0x0 : buffer[4];
                                buffer[5] = buffer[5] < 0xA ? 0x0 : buffer[5];

                                lfWlVel = buffer[6] ? (-1 * (int)buffer[4]) : (int)buffer[4];
                                rtWlVel = buffer[7] ? (int)buffer[5] : (-1 * (int)buffer[5]);

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
*/
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
private: sensors::RaySensorPtr lrf;

//pointer to the gps sensor
private: sensors::GpsSensorPtr gps;

// Pointer to the update event connection
private: event::ConnectionPtr updateConnection;
};




// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(InsomniaPlugin)
}
