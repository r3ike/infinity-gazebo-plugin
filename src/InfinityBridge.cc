#include "InfinityBridge.hh"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <iostream>

using namespace infinity_plugin;

InfinityBridge::InfinityBridge()
{
}

void InfinityBridge::Configure(
	const gz::sim::Entity &_entity,
	const std::shared_ptr<const sdf::Element> &_sdf,
	gz::sim::EntityComponentManager &_ecm,
	gz::sim::EventManager &/*_eventMgr*/)
{
	std::string imuTopic = "/imu";

	if (_sdf->HasElement("imu_topic"))
	{
		std::cout << "topic trovato" << std::endl;
		imuTopic = _sdf->Get<std::string>("imu_topic");
	}

	// Subscribe ai topic

	this->node_.Subscribe(imuTopic, &InfinityBridge::OnImuData, this);
		
  	std::cout << "[Infinity-autopilot] Plugin configurato con successo !"<< std::endl;
}

void InfinityBridge::PreUpdate(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
  	// _info contiene informazioni come _info.simTime e _info.dt
  	if (!_info.paused)
  	{
		// Legge i valori della imu in base al periodo 
		if (this->newImuSample_)
		{
			ReadAndSendSensors();
			std::cout << "[Infinity-autopilot] [] Sample letto!" << std::endl;
			
		}
  	}		
}



void InfinityBridge::ReadAndSendSensors()
{
	std::lock_guard<std::mutex> lock(this->imuMutex_);
	double accelX = this->latestImuSample_.linear_acceleration().x();
	double accelY = this->latestImuSample_.linear_acceleration().y();
	double accelZ = this->latestImuSample_.linear_acceleration().z();

	std::cout << "[Infinity-autopilot] AccelX: " << accelX << std::endl;

	double gyroX = this->latestImuSample_.angular_velocity().x();
	double gyroY = this->latestImuSample_.angular_velocity().y();
	double gyroZ = this->latestImuSample_.angular_velocity().z();
	
	this->newImuSample_ = false;
}

/**
 * 
 */
void InfinityBridge::OnImuData(const gz::msgs::IMU &msg)
{
	std::lock_guard<std::mutex> lock(this->imuMutex_);
    this->latestImuSample_ = msg;
    this->newImuSample_ = true;
}

void InfinityBridge::OnGpsData(gz::msgs::NavSat &msg)
{
}

void InfinityBridge::OnBaroData(gz::msgs::FluidPressure &msg)
{
}

void InfinityBridge::OnMagData(gz::msgs::Magnetometer &msg)
{
}

// Registrazione del plugin tramite le macro di Gazebo
GZ_ADD_PLUGIN(
	infinity_plugin::InfinityBridge,
	gz::sim::System,
	infinity_plugin::InfinityBridge::ISystemConfigure,
	infinity_plugin::InfinityBridge::ISystemPreUpdate
)

// Alias opzionale per semplificare il caricamento nel file SDF
GZ_ADD_PLUGIN_ALIAS(infinity_plugin::InfinityBridge, "infinity_plugin::InfinityBridge")
