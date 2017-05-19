
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

uint8_t tennisball_debug = 0;

namespace gazebo
{

	class TennisBallPlugin : public ModelPlugin
	{

		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{

		    // Store the pointer to the model
		    this->model = _parent;

			// used to setup all the sensors based on the enable flags above
			//Initialize();
		    
		if ((this->gps = std::dynamic_pointer_cast<gazebo::sensors::GpsSensor>(
			gazebo::sensors::SensorManager::Instance()->GetSensor("gps"))) == NULL)
		{
			std::cout << "COULD NOT FIND GPS SENSOR" << std::endl;
			return;
		}
			// Listen to the update event. This event is broadcast every
		    // simulation iteration.
		  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&TennisBallPlugin::OnUpdate, this, _1));

		}

		// Called by the world update start event
		public: void OnUpdate(const common::UpdateInfo & /*_info*/)
		{
			
			double prevlatitude_deg = 0;
			double prevlatitude_min = 0;
			double prevlatitude_sec = 0;
			double prevlongitude_deg = 0;
			double prevlongitude_min = 0;
			double prevlongitude_sec = 0;


			double latitude_dec = this->gps->Latitude().Degree();
			double longitude_dec = this->gps->Longitude().Degree();

			double latitude_deg = floor(latitude_dec);
			double latitude_min = (latitude_dec - floor(latitude_dec)) * 60.0;
			double latitude_sec = (latitude_min - floor(latitude_min)) * 60.0;
	 
			double longitude_deg = floor(longitude_dec);
			double longitude_min = (longitude_dec - floor(longitude_dec)) * 60.0;
			double longitude_sec = (longitude_min - floor(longitude_min)) * 60.0;
				
				
				if(tennisball_debug){

					printf("Tennisball: latitude = %f | longitude = %f\n", latitude_dec, longitude_dec);
			printf("Tennisball: latitude = %fdeg %f\'%f\" | longitude = %fdeg %f\'%f\"\n",
				latitude_deg, latitude_min, latitude_sec, longitude_deg, longitude_min, longitude_sec );

			prevlongitude_deg = longitude_deg;
			prevlongitude_min = longitude_min;
			prevlongitude_sec = longitude_sec;
			prevlatitude_deg = latitude_deg;
			prevlatitude_min = latitude_min;
			prevlatitude_sec = latitude_sec;
}
			
		
				
		}
		// Pointer to the model
		private: physics::ModelPtr model;
		
		// Pointer to the gps sensor
		private: sensors::GpsSensorPtr gps;
		
		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;
	};
	GZ_REGISTER_MODEL_PLUGIN(TennisBallPlugin)
}
