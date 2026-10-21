#include "InfinityBridge.hh"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <iostream>

using namespace infinity_bridge;

InfinityBridge::InfinityBridge()
{
}

void InfinityBridge::Configure(
	const gz::sim::Entity &_entity,
	const std::shared_ptr<const sdf::Element> &_sdf,
	gz::sim::EntityComponentManager &_ecm,
	gz::sim::EventManager &/*_eventMgr*/)
{

	this->imuEntity_ = _ecm.EntityByComponents(
		gz::sim::components::Name("imu"),
      	gz::sim::components::Sensor()
	);

	if (this->imuEntity_ != gz::sim::kNullEntity)
	{
		auto sensorComp = _ecm.Component<gz::sim::components::Sensor>(this->imuEntity_);

		if (sensorComp)
		{
			double updateRate = 100.0; // Valore di default (Hz)

			if (_sdf->HasElement("update_rate"))
			{
				updateRate = _sdf->Get<double>("update_rate");
			}
			std::cout << "[Infinity-autopilot] Frequenza IMU letta da SDF: " << updateRate << " Hz" << std::endl;

			// Calcola l'intervallo di campionamento necessario per il tuo loop
			if (updateRate > 0.0)
			{
				this->imuPeriod_ = std::chrono::duration<double>(1.0 / updateRate);
			}
		}
		
	}
	

  	std::cout << "[Infinity-autopilot] Plugin configurato con successo !"<< std::endl;
}

void InfinityBridge::PreUpdate(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
  	// _info contiene informazioni come _info.simTime e _info.dt
  	if (!_info.paused && this->imuEntity_ != gz::sim::kNullEntity)
  	{
		auto imuComp = _ecm.Component<gz::sim::components::Imu>(this->imuEntity_);
		if (!imuComp)
		{
			return;
		}
		// Legge i valori della imu in base al periodo 
		if (_info.simTime - lastImuRead_ >= this->imuPeriod_)
		{
			ReadAndSendSensors(imuComp);
			std::cout << "[Infinity-autopilot] [] Sample letto!" << std::endl;
			lastImuRead_ = _info.simTime;
		}
  	}		
}

void infinity_bridge::InfinityBridge::ReadAndSendSensors(gz::sim::components::Imu *imuComp)
{
	const gz::msgs::IMU &imuData = imuComp->Data();

	double accelX = imuData.linear_acceleration().x();
	double accelY = imuData.linear_acceleration().y();
	double accelZ = imuData.linear_acceleration().z();

	std::cout << "[Infinity-autopilot] AccelX: " << accelX << std::endl;

	double gyroX = imuData.angular_velocity().x();
	double gyroY = imuData.angular_velocity().y();
	double gyroZ = imuData.angular_velocity().z();
	
}

// Registrazione del plugin tramite le macro di Gazebo
GZ_ADD_PLUGIN(
	infinity_bridge::InfinityBridge,
	gz::sim::System,
	infinity_bridge::InfinityBridge::ISystemConfigure,
	infinity_bridge::InfinityBridge::ISystemPreUpdate
)

// Alias opzionale per semplificare il caricamento nel file SDF
GZ_ADD_PLUGIN_ALIAS(infinity_bridge::InfinityBridge, "infinity_bridge::InfinityBridge")
