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
#include <math.h>
#include "../utils/UdpSocket.hpp"
#include <Packet.h>
#include <BclStatus.h>
#include <MechanicalControlPackets.h>
#include <SensorControlPackets.h>

static Socket::UdpSocket client_port(10000);
static Socket::UdpSocket lrf_port(20000);
static Socket::UdpSocket imu_port(20001);
static Socket::UdpSocket gps_port(20002);

static uint8_t testing = 1; 
static uint8_t lrf_en = 0;
static uint8_t imu_en = 0; 
static uint8_t gps_en = 0;

static int lrf_count = 0;

// sensor endpoint addresses
static struct sockaddr_in lrf_addr;
static struct sockaddr_in sensors_addr;
static struct sockaddr_in robot_addr;    

// drive command data structure
BclPacket tank_drive_pkt;
TankDrivePayload tank_drive_payload;

// async drive command reads
pthread_t thread;
std::mutex mtx;
uint8_t thread_running = 0;

namespace gazebo
{

class InsomniaPlugin : public ModelPlugin
{

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    // Store the pointer to the model
    this->model = _parent;
		
		// used to setup all the sensors based on the enable flags above

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

private: static void* GetDriveCommand(void* threadid)
{
	BCL_STATUS status;
	uint8_t buffer[255];

	while(1)
	{
		memset(buffer, 0, 255);
		uint8_t bytes_read = client_port.Read(&buffer[0],255, nullptr);
		mtx.lock();
		status = DeserializeBclPacket(&tank_drive_pkt, buffer, bytes_read);
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

	memset(buffer,0,255);
	int rd = 0;

	if(testing)
	{
		//fake drive command
		tank_drive_payload.left = 50;
		tank_drive_payload.right = 50;
	}

   	else if (!thread_running)
	{
			InitializeSetTankDriveSpeedPacket(&tank_drive_pkt, &tank_drive_payload);
			if(pthread_create(&thread, NULL, GetDriveCommand, NULL))
			{
				fprintf(stderr, "Error: unalbe to create thread\n");
				return;
			}
			thread_running = 1;
	}	
   else
    {
		// fake drive command
        for(int k = 0; k < 12; k++)
        {
        	buffer[k] = 0xA;
        }
    }

   mtx.lock();
   TankDrivePayload tmp = tank_drive_payload;
   mtx.unlock();
   
   // move robot
   Drive(tmp);
		
	if(imu_en == 1)
	{

		BclPacket imu_pkt;
		ImuPayload imu_payload;

		InitializeReportIMUPacket(&imu_pkt, &imu_payload);

		math::Quaternion orientation = this->imu->Orientation();

		uint8_t yaw = orientation.GetYaw();
		uint8_t yaw_degree = yaw * 180 / M_PI;  
	    //printf("Yaw is: %f\n", yaw);

		uint16_t z_orient = (uint16_t)yaw_degree;

		imu_payload.z_orient = z_orient;

		memset(buffer, 0, 255);
		status = SerializeBclPacket(&imu_pkt, buffer, 255, &bytes_written);
		

		if(status != BCL_OK)
		{
			fprintf(stderr, "Failed to serialize imu packet!\n");
			return;
		}
			
		if(!imu_port.Write(&buffer[0], bytes_written , (struct sockaddr*)&robot_addr))
		{
				fprintf(stderr, "Failed to send imu to listener!\n");
				return;
		}
	}

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
			payload[m++] = (char)(tmp >> 8);
		 	payload[m++] = (char)tmp;
		}	
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

		uint8_t latitude_dec = this->gps->Latitude().Degree();
		uint8_t longitude_dec = this->gps->Longitude().Degree();

		uint8_t latitude_deg = (uint8_t)floor(latitude_dec);
		uint8_t latitude_min = (uint8_t)((latitude_dec - floor(latitude_dec)) * 60.0);
		uint8_t latitude_sec = (uint8_t)((latitude_min - floor(latitude_min)) * 60.0);

		uint8_t longitude_deg = (uint8_t)(floor(longitude_dec));
		uint8_t longitude_min = (uint8_t)((longitude_dec- floor(longitude_dec)) * 60.0);
		uint8_t longitude_sec = (uint8_t)((longitude_min - floor(longitude_min)) * 60.0);
	

		gps_payload.lat_degrees = (uint16_t)latitude_deg;
		gps_payload.lat_minutes = (uint16_t)latitude_min;
		gps_payload.lat_seconds = (uint16_t)latitude_sec;

		gps_payload.long_degrees = (uint16_t)longitude_deg;
		gps_payload.long_minutes = (uint16_t)longitude_min;
		gps_payload.long_seconds = (uint16_t)longitude_sec;


		memset(buffer, 0, 255);
		status = SerializeBclPacket(&gps_pkt, buffer, 255, &bytes_written);

		if (status != BCL_OK)
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
}

private: void Drive(TankDrivePayload tank_drive_payload)
{

    //buffer[0] = buffer[0] < 0xA ? 0 : 0xA;
    //buffer[3] = buffer[3] < 0xA ? 0 : 0xA;
    int lfWlVel = (int8_t)(tank_drive_payload.left / 10);
    //int lfWlVel = buffer[6] ? (-1 * (int)buffer[0]) : (int)buffer[0];
    //int rtWlVel = buffer[9] ? (int)buffer[3] : (-1 * (int)buffer[3]);
    int rtWlVel = (-1*(int)((int8_t)tank_drive_payload.right / 10));
		//arm
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
GZ_REGISTER_MODEL_PLUGIN(InsomniaPlugin);
}

