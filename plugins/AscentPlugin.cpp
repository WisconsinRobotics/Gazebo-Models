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
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <Packet.h>
#include <BclStatus.h>
#include <MechanicalControlPackets.h>
#include <SensorControlPackets.h>
#include "../utils/UdpSocket.hpp"
#include <calculateAngles.h>

static Socket::UdpSocket client_port(10000);
static Socket::UdpSocket lrf_port(20000);
static Socket::UdpSocket gps_port(20001);
static Socket::UdpSocket imu_port(20002);

// only ever have one or no test flag set at a time
static int testing = 0;
static int testing_ik = 1;

static int lrf_en = 0;
static int gps_en = 0;
static int imu_en = 0;

static struct sockaddr_in robot_addr;    

namespace gazebo
{


class AscentPlugin : public ModelPlugin
{

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // Store the pointer to the model
    this->model = _parent;

	// Initialize sensors
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

	if ((this->imu = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(
		 gazebo::sensors::SensorManager::Instance()->GetSensor("imu"))) == NULL)
	{
		std::cout << "COULD NOT FIND IMU SENSOR" << std::endl;
		return;
	}
		
	// Open Udp ports 
	if(testing == 0)
	{
    	if(!client_port.Open())
    	{
        	std::cout << "Failed to open client_port!" << std::endl;
    	}
	}
	
	if(lrf_en == 1)
	{
		if(!lrf_port.Open())
    	{
        	std::cout << "Failed to open lrf_port!" << std::endl;
   		 }
	}
	
	if(gps_en == 1)
	{
		if(!gps_port.Open())
    	{
        	std::cout << "Failed to open gps_port!" << std::endl;
   		 }
	}
	
	if(imu_en == 1)
	{
		if(!imu_port.Open())
    	{
        	std::cout << "Failed to open imu_port!" << std::endl;
   		 }
	}

	// setup outgoing udp address
    memset(&robot_addr, 0, sizeof(struct sockaddr_in));    
    inet_pton(AF_INET, "192.168.1.144", &(robot_addr.sin_addr));    
    robot_addr.sin_family = AF_INET;    
    robot_addr.sin_port = htons(20001); 		
		
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&AscentPlugin::OnUpdate, this, _1));
}

// Called by the world update start event
public: void OnUpdate(const common::UpdateInfo & /*_info*/)
{
	BCL_STATUS status;
    uint8_t buffer[255];
	int ik_buf[3];
	uint8_t bytes_written;

	memset(buffer, 0, 255);

	BclPacket tank_drive_pkt;
	TankDrivePayload tank_drive_payload;


    if(testing)
    {	
		// fake drive command
        for(int k = 0; k < 12; k++)
        {
        	buffer[k] = 0x5;
        }

    }
	else if(testing_ik)
	{
		int x = 10;
		int y = 10;
		int z = 10;
/*
		if(!calculateAngles(ik_buf, x, y, z)
		{
			fprintf(stderr, "Can't get there!!\n");
		}
*/		
	}
    else
    {
		// drive command
		InitializeSetTankDriveSpeedPacket(&tank_drive_pkt, &tank_drive_payload);
        client_port.Read(&buffer[0], 255, nullptr);
		//status = DeserializeBclPacket(&tank_drive_pkt, buffer, 255);
		//if(status != BCL_OK)
		//{
		//	fprintf(stderr, "Failed to deserialize Tank Drive!\n");
		//	return;
		//}

    }

	// move robot
	if(testing_ik)
		Drive_ik(ik_buf);
	else
   	 	Drive(buffer);

	if(lrf_en == 1)
	{
   		std::vector<double> lrfData;
   		this->lrf->Ranges(lrfData);

	   	if(lrfData.size() == 0)
    	   	return;

		// LRF data is flipped, so reverse the array
		std::reverse(std::begin(lrfData), std::end(lrfData));

		int len = lrfData.size()*2;

		uint8_t payload[len];


		int m = 0;
		for(int l = 0; l < lrfData.size(); l++)
		{
			lrfData.at(l) = lrfData.at(l) * 1000;
			float f = (float)lrfData.at(l);
			uint32_t uint = (uint32_t)(*(uint32_t*)&f);
			uint16_t tmp = (uint16_t)lrfData.at(l);
			payload[m++] = (uint8_t)(tmp >> 8);
			payload[m++] = (uint8_t)tmp;
		}

		printf("lrf at 0 = %f\n", lrfData.at(0));



		if(!lrf_port.Write(&payload[0], len, (struct sockaddr*)&robot_addr))
		{
			fprintf(stderr, "Failed to send lrf to listener!\n");
			return;
		}
	}

	if(gps_en == 1)
	{
		BclPacket gps_pkt;
		GpsPayload gps_payload;

		InitializeReportGPSPacket(&gps_pkt, &gps_payload);

		double latitude_dec = this->gps->Latitude().Degree();
		double longitude_dec = this->gps->Longitude().Degree();

		uint16_t latitude_deg = (uint16_t)floor(latitude_dec);
		uint16_t latitude_min = (uint16_t)((latitude_dec - floor(latitude_dec)) * 60.0);
		uint16_t latitude_sec = (uint16_t)((latitude_min - floor(latitude_min)) * 60.0);
 
		uint16_t longitude_deg = (uint16_t)(floor(longitude_dec));
		uint16_t longitude_min = (uint16_t)((longitude_dec - floor(longitude_dec)) * 60.0);
		uint16_t longitude_sec = (uint16_t)((longitude_min - floor(longitude_min)) * 60.0);

		gps_payload.lat_degrees = latitude_deg;
		gps_payload.lat_minutes = latitude_min;
		gps_payload.lat_seconds = latitude_sec;
  
		gps_payload.long_degrees = longitude_deg;
		gps_payload.long_minutes = longitude_min;
		gps_payload.long_seconds = longitude_sec;

		memset(buffer, 0, 255);
		status = SerializeBclPacket(&gps_pkt, buffer, 255, &bytes_written);

		if(status != BCL_OK)
		{
			fprintf(stderr, "Failed to serialize gps packet!\n");
			return;
		}

		if(!gps_port.Write(&buffer[0], bytes_written, nullptr))
		{
			fprintf(stderr, "Failed to send gps to listener!\n");
			return;
		}
	}

	if(imu_en == 1)
	{	
		BclPacket imu_pkt;
		ImuPayload imu_payload;

		InitializeReportIMUPacket(&imu_pkt, &imu_payload);

		math::Quaternion orientation = this->imu->Orientation();

		double yaw_rad = orientation.GetYaw();
		double yaw_degree = yaw_rad * 180 / M_PI;  

		uint16_t z_orient = (uint16_t)yaw_degree;

		imu_payload.z_orient = z_orient;

		memset(buffer, 0, 255);
		status = SerializeBclPacket(&imu_pkt, buffer, 255, &bytes_written);

		if(status != BCL_OK)
		{
			fprintf(stderr, "Failed to serialize imu packet!\n");
			return;
		}

		if(!imu_port.Write(&buffer[0], bytes_written, nullptr))
		{
			fprintf(stderr, "Failed to send imu to listener!\n");
			return;
		}
	}
}

private: void Drive_ik(int* ik_buf)
{

	int lfWheels = 0;
	int rtWheels = 0;

	// arm
    int turntable = 0;
    int shoulder = 0;
    int elbow = 0;
    int wristPitch = 0;
    int wristRot = 0;
    int jaw = 0;

    this->model->GetJoint("Wheel0")->SetVelocity(0,lfWheels);	//positive = forward
    this->model->GetJoint("Wheel1")->SetVelocity(0,lfWheels);	
    this->model->GetJoint("Wheel2")->SetVelocity(0,lfWheels);	
    this->model->GetJoint("Wheel3")->SetVelocity(0,rtWheels);	//negative = forward
    this->model->GetJoint("Wheel4")->SetVelocity(0,rtWheels);	
    this->model->GetJoint("Wheel5")->SetVelocity(0,rtWheels);	

	// buffer[0] = turntable
	// buffer[1] = elbow
	// buffer[2] = shoulder
		
	ik_buf[0] = ik_buf[0] * M_PI / 180;
	ik_buf[1] = ik_buf[1] * M_PI / 180;
	ik_buf[2] = ik_buf[2] * M_PI / 180;

	math::Angle tt((double)ik_buf[0]);
	math::Angle shld((double)ik_buf[2]);
	math::Angle elb((double)ik_buf[1]);

	//negative = clockwise
    //this->model->GetJoint("Turntable")->SetAngle(0, tt);
	this->model->GetJoint("Turntable")->SetPosition(0, 0.2);

	//positive = up
	//this->model->GetJoint("Shoulder")->SetAngle(0, shld);

	//positive = up
    //this->model->GetJoint("Elbow")->SetAngle(0, elb);

	this->model->GetJoint("WristPitch")->SetVelocity(0,wristPitch); //positive = up
	this->model->GetJoint("WristRot")->SetVelocity(0,wristRot);     //negative = clockwise
	this->model->GetJoint("Jaw0")->SetVelocity(0,jaw);       		//positive = closed
	this->model->GetJoint("Jaw1")->SetVelocity(0,jaw);       		//positive = closed
	
}

private: void Drive(uint8_t* buffer)
{
	// wheels
    int lfWheels = (-1 * (int)((int8_t)buffer[0]));
    int rtWheels = (int)((int8_t)buffer[1]);
	//printf("lfWheels = %d | rtWheels = %d\n", lfWheels, rtWheels);

	// arm
    int turntable = 0;
    int shoulder = 0;
    int elbow = 0;
    int wristPitch = 0;
    int wristRot = 0;
    int jaw = 0;

    this->model->GetJoint("Wheel0")->SetVelocity(0,lfWheels);	//positive = forward
    this->model->GetJoint("Wheel1")->SetVelocity(0,lfWheels);	
    this->model->GetJoint("Wheel2")->SetVelocity(0,lfWheels);	
    this->model->GetJoint("Wheel3")->SetVelocity(0,rtWheels);	//negative = forward
    this->model->GetJoint("Wheel4")->SetVelocity(0,rtWheels);	
    this->model->GetJoint("Wheel5")->SetVelocity(0,rtWheels);	

	this->model->GetJoint("Turntable")->SetVelocity(0,turntable);  	//negative = clockwise
    this->model->GetJoint("Shoulder")->SetVelocity(0,shoulder); 	//positive = up
   	this->model->GetJoint("Elbow")->SetVelocity(0,elbow);  			//positive = up
    this->model->GetJoint("WristPitch")->SetVelocity(0,wristPitch); //positive = up
	this->model->GetJoint("WristRot")->SetVelocity(0,wristRot);     //negative = clockwise
    this->model->GetJoint("Jaw0")->SetVelocity(0,jaw);       		//positive = closed
    this->model->GetJoint("Jaw1")->SetVelocity(0,jaw);       		//positive = closed
}

// Pointer to the model
private: physics::ModelPtr model;

//pointer to the laserSensor
private: sensors::RaySensorPtr lrf;

//pointer to the gps sensor
private: sensors::GpsSensorPtr gps;

//pointer to the imu sensor
private: sensors::ImuSensorPtr imu;

// Pointer to the update event connection
private: event::ConnectionPtr updateConnection;
};


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AscentPlugin)
} 
