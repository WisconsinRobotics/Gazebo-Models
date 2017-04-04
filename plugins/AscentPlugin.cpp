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
#include <crc8.h>
#include <Packet.h>
#include <BclStatus.h>
#include <MechanicalControlPackets.h>
#include <SensorControlPackets.h>
#include "../utils/UdpSocket.hpp"
#include <calculateAngles.h>
#include <pthread.h>
#include <mutex>

const char* IP_ENDPOINT = "192.168.1.50";
const int LRF_PORT = 20001;
const int SENSORS_PORT = 15000;

Socket::UdpSocket client_port(10000);
Socket::UdpSocket lrf_port(20000);
Socket::UdpSocket gps_port(20002);
Socket::UdpSocket imu_port(20003);

// flag to indicate whether to use a dummy drive command 
int testing = 0;

// enable flags for sensors
int lrf_en = 1;
int gps_en = 1;
int imu_en = 1;

// debug flags
uint8_t lrf_debug = 0;
uint8_t imu_debug = 0;
uint8_t gps_debug = 0;
uint8_t drive_cmd_debug = 0;

// sensor endpoint addresses
struct sockaddr_in lrf_addr;   
struct sockaddr_in sensors_addr;

// drive command data structure
BclPacket tank_drive_pkt;
TankDrivePayload tank_drive_payload;

// async drive command reads
pthread_t thread;
std::mutex mtx;
uint8_t thread_running = 0;

namespace gazebo
{

class AscentPlugin : public ModelPlugin
{

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // Store the pointer to the model
    this->model = _parent;

	// used to setup all the sensors based on the enable flags above
	Initialize();
    
	// Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			boost::bind(&AscentPlugin::OnUpdate, this, _1));
}

private: static void* GetDriveCommand(void* threadid)
{
	BCL_STATUS status;
	uint8_t buffer[255];

	while(1)
	{
		memset(buffer, 0, 255);
		uint8_t bytes_read = client_port.Read(&buffer[0], 255, nullptr);
		mtx.lock();
		status = DeserializeBclPacket(&tank_drive_pkt, buffer, bytes_read);
		printf("lfWheels = %d | rtWheels = %d\n", tank_drive_payload.left, tank_drive_payload.right);
		mtx.unlock();
		if(status != BCL_OK)
		{
			fprintf(stderr, "Failed to deserialize Tank Drive!\n");
		}
	}
}

// Called by the world update start event
public: void OnUpdate(const common::UpdateInfo & /*_info*/)
{
	BCL_STATUS status;
    uint8_t buffer[255];
	uint8_t bytes_written;

	memset(buffer, 0, 255);

    if(testing)
    {	
		// fake drive command
		tank_drive_payload.left = 50;
		tank_drive_payload.right = 50;
    }
	else if(!thread_running)
	{
		InitializeSetTankDriveSpeedPacket(&tank_drive_pkt, &tank_drive_payload);
		/*
		if(pthread_create(&thread, NULL, GetDriveCommand, NULL))
		{
			fprintf(stderr, "Error: unable to create thread\n");
			return;
		}
		thread_running = 1;
		*/
		uint8_t bytes_read = client_port.Read(&buffer[0], 255, nullptr);
		status = DeserializeBclPacket(&tank_drive_pkt, buffer, bytes_read);
		printf("lfWheels = %d | rtWheels = %d\n", tank_drive_payload.left, tank_drive_payload.right);
		if(status != BCL_OK)
		{
			fprintf(stderr, "Failed to deserialize Tank Drive!\n");
		}
	}

	mtx.lock();
	TankDrivePayload tmp = tank_drive_payload;
	mtx.unlock();

	// move robot
   	Drive(tmp);

	if(lrf_en)
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

		if(lrf_debug)
			printf("lrf at 0 = %f\n", lrfData.at(0));

		if(!lrf_port.Write(&payload[0], len, (struct sockaddr*)&lrf_addr))
		{
			fprintf(stderr, "Failed to send lrf to listener!\n");
			return;
		}
	}

	if(gps_en)
	{
		BclPacket gps_pkt;
		GpsPayload gps_payload;

		InitializeReportGPSPacket(&gps_pkt, &gps_payload);

		double latitude_dec = this->gps->Latitude().Degree() * 1000;
		double longitude_dec = this->gps->Longitude().Degree() * 1000;

		double latitude_deg = floor(latitude_dec);
		double latitude_min = (latitude_dec - floor(latitude_dec)) * 60.0;
		double latitude_sec = (latitude_min - floor(latitude_min)) * 60.0;
 
		double longitude_deg = floor(longitude_dec);
		double longitude_min = (longitude_dec - floor(longitude_dec)) * 60.0;
		double longitude_sec = (longitude_min - floor(longitude_min)) * 60.0;

		if(gps_debug) 
		{
			printf("gps: latitude = %f | longitude = %f\n", latitude_dec, longitude_dec);
			printf("gps: latitude = %fdeg %f\'%f\" | longitude = %fdeg %f\'%f\"\n",
																				latitude_deg,
																				latitude_min,
																				latitude_sec,
																				longitude_deg,
																				longitude_min,
																				longitude_sec );
		}

		gps_payload.lat_degrees = (uint16_t)latitude_deg;
		gps_payload.lat_minutes = (uint16_t)latitude_min;
		gps_payload.lat_seconds = (uint16_t)latitude_sec;
  
		gps_payload.long_degrees = (uint16_t)longitude_deg;
		gps_payload.long_minutes = (uint16_t)longitude_min;
		gps_payload.long_seconds = (uint16_t)longitude_sec;

		memset(buffer, 0, 255);
		status = SerializeBclPacket(&gps_pkt, buffer, 255, &bytes_written);

		if(status != BCL_OK)
		{
			fprintf(stderr, "Failed to serialize gps packet!\n");
			return;
		}

		if(!gps_port.Write(&buffer[0], bytes_written, (struct sockaddr*)&sensors_addr))
		{
			fprintf(stderr, "Failed to send gps to listener!\n");
			return;
		}
	}

	if(imu_en)
	{	
		BclPacket imu_pkt;
		ImuPayload imu_payload;

		InitializeReportIMUPacket(&imu_pkt, &imu_payload);

		math::Quaternion orientation = this->imu->Orientation();

		double yaw_rad = orientation.GetYaw();
		yaw_rad = yaw_rad < 0 ? yaw_rad + 2*M_PI : yaw_rad;

		double yaw_degree = yaw_rad * 180 / M_PI;

		uint16_t z_orient = (uint16_t)yaw_degree;

		if(imu_debug)
			printf("imu: z_orient = %d\n", z_orient);

		imu_payload.z_orient = z_orient;

		memset(buffer, 0, 255);
		status = SerializeBclPacket(&imu_pkt, buffer, 255, &bytes_written);

		if(status != BCL_OK)
		{
			fprintf(stderr, "Failed to serialize imu packet!\n");
			return;
		}

		if(!imu_port.Write(&buffer[0], bytes_written, (struct sockaddr*)&sensors_addr))
		{
			fprintf(stderr, "Failed to send imu to listener!\n");
			return;
		}
	}
}

private: void Drive(TankDrivePayload tank_drive_payload)
{
	// wheels
    int lfWheels = (int)tank_drive_payload.left / 10; // scale to 0 - 10 speed
    int rtWheels = (int)tank_drive_payload.right / 10;

	// arm
    int turntable = 0;
    int shoulder = 0;
    int elbow = 0;
    int wristPitch = 0;
    int wristRot = 0;
    int jaw = 0;

	if(drive_cmd_debug)
		printf("lfWheels = %d | rtWheels = %d\n", lfWheels, rtWheels);


    this->model->GetJoint("Wheel0")->SetVelocity(0,lfWheels);	//positive = forward
    this->model->GetJoint("Wheel1")->SetVelocity(0,lfWheels);	
    this->model->GetJoint("Wheel2")->SetVelocity(0,lfWheels);	
    this->model->GetJoint("Wheel3")->SetVelocity(0,rtWheels);	//negative = forward
    this->model->GetJoint("Wheel4")->SetVelocity(0,rtWheels);	
 	this->model->GetJoint("Wheel5")->SetVelocity(0,rtWheels);	
	
	this->model->GetJoint("Turntable")->SetVelocity(0,turntable);  	//negative = clockwise
    this->model->GetJoint("Shoulder")->SetVelocity(0,0); 			//positive = up
   	this->model->GetJoint("Elbow")->SetVelocity(0,elbow);  			//positive = up
    this->model->GetJoint("WristPitch")->SetVelocity(0,wristPitch); //positive = up
	this->model->GetJoint("WristRot")->SetVelocity(0,wristRot);     //negative = clockwise
    this->model->GetJoint("Jaw0")->SetVelocity(0,jaw);       		//positive = closed
    this->model->GetJoint("Jaw1")->SetVelocity(0,jaw);       		//positive = closed

// experimenting; sometimes helps when arm continually moves down, still don't know for sure tho
/*
	this->model->GetJoint("Turntable")->SetPosition(0,turntable);  	//negative = clockwise
    this->model->GetJoint("Shoulder")->SetPosition(0,shoulder); 	//positive = up
   	this->model->GetJoint("Elbow")->SetPosition(0,elbow);  			//positive = up
    this->model->GetJoint("WristPitch")->SetPosition(0,wristPitch); //positive = up
	this->model->GetJoint("WristRot")->SetPosition(0,wristRot);     //negative = clockwise
    this->model->GetJoint("Jaw0")->SetPosition(0,jaw);       		//positive = closed
    this->model->GetJoint("Jaw1")->SetPosition(0,jaw);       		//positive = closed
*/
}

private: void Initialize()
{
	// port for drive commands
	if(!testing)
	{
    	if(!client_port.Open())
    	{
        	std::cout << "Failed to open client_port!" << std::endl;
    	}
	}

	if(lrf_en)
	{
    	if ((this->lrf = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(
        	gazebo::sensors::SensorManager::Instance()->GetSensor("laser"))) == NULL)
    	{
        	std::cout << "COULD NOT FIND LASER SENSOR" << std::endl;
        	return;
    	}

		if(!lrf_port.Open())
    	{
        	std::cout << "Failed to open lrf_port!" << std::endl;
    	}
		
		memset(&lrf_addr, 0, sizeof(struct sockaddr_in));

    	int status = inet_pton(AF_INET, IP_ENDPOINT, &(lrf_addr.sin_addr));

		if(status == 0)
		{
			std::cout << "src does not contain a character string representing ";
			std::cout << "a valid network address in the specified address family" << std::endl;
		}
		if(status == -1)
		{
			std::cout << "network error" << std::endl;
		}

    	lrf_addr.sin_family = AF_INET;
    	lrf_addr.sin_port = htons(LRF_PORT);
	}

	if(gps_en)
	{
		if ((this->gps = std::dynamic_pointer_cast<gazebo::sensors::GpsSensor>(
			gazebo::sensors::SensorManager::Instance()->GetSensor("gps"))) == NULL)
		{
			std::cout << "COULD NOT FIND GPS SENSOR" << std::endl;
			return;
		}

		if(!gps_port.Open())
    	{
        	std::cout << "Failed to open gps_port!" << std::endl;
   		}
	}

	if(imu_en)
	{
		if ((this->imu = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(
			gazebo::sensors::SensorManager::Instance()->GetSensor("imu"))) == NULL)
		{
			std::cout << "COULD NOT FIND IMU SENSOR" << std::endl;
			return;
		}
		
		if(!imu_port.Open())
    	{
        	std::cout << "Failed to open imu_port!" << std::endl;
   		}
	}
	
	if(gps_en | imu_en)
	{
		memset(&sensors_addr, 0, sizeof(struct sockaddr_in));
    	inet_pton(AF_INET, IP_ENDPOINT, &(sensors_addr.sin_addr));
    	sensors_addr.sin_family = AF_INET;
    	sensors_addr.sin_port = htons(SENSORS_PORT);
	}
}

// Pointer to the model
private: physics::ModelPtr model;

// Pointer to the LRF
private: sensors::RaySensorPtr lrf;

// Pointer to the gps sensor
private: sensors::GpsSensorPtr gps;

// Pointer to the imu sensor
private: sensors::ImuSensorPtr imu;

// Pointer to the update event connection
private: event::ConnectionPtr updateConnection;

};


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(AscentPlugin)
} 
