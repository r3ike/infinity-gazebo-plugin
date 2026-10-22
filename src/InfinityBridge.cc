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

	ReadSdfElements(_sdf);

	// Inizializzazione socket
	autopilot_sock_.init(this->simPort_, this->autopilotAddr_.c_str(),this->autopilotPort_)

	// Subscribe ai topic
	this->node_.Subscribe(imuTopic, &InfinityBridge::OnImuData, this);
		
  	std::cout << "[Infinity-autopilot] Plugin configurato con successo !"<< std::endl;
}


void InfinityBridge::ReadSdfElements(const std::shared_ptr<const sdf::Element> &_sdf){
	
	if (_sdf->HasElement("imu_topic"))
	{
		std::cout << "imu_topic trovato" << std::endl;
		this->imuTopic_ = _sdf->Get<std::string>("imu_topic");
	}

	if (_sdf->HasElement("sim_port"))
	{
		std::cout << "sim_port trovato" << std::endl;
		this->simPort_ = _sdf->Get<int>("sim_port");
	}

	if (_sdf->HasElement("autopilot_port"))
	{
		std::cout << "autopilot_port trovato" << std::endl;
		this->autopilotPort_ = _sdf->Get<int>("autopilot_port");
	}

	if (_sdf->HasElement("autopilot_address"))
	{
		std::cout << "autopilot_address trovato" << std::endl;
		this->autopilotAddr_ = _sdf->Get<std::string>("autopilot_address");
	}
}


void InfinityBridge::PreUpdate(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
  	// _info contiene informazioni come _info.simTime e _info.dt
  	if (!_info.paused)
  	{
		auto simTimeUs = std::chrono::duration_cast<std::chrono::microseconds>(_info.simTime);
    	uint64_t simTimeUint64 = static_cast<uint64_t>(simTimeNs.count());
		// Legge i valori di IMU, Mag e Baro ad ogni nuovo sample imu
		// è necessario fare tutto insieme perchè mavlink prevede un'unico messaggio
		// HIL_SENSORS, lato firmware bisognerà verificare quali dati sono stati aggiornati
		// mediante gli appositi flag presenti nel pacchetto
		if (this->newImuSample_)
		{
			ReadAndSendSensors(simTimeUint64);
			std::cout << "[Infinity-autopilot] [] Sample letto!" << std::endl;
			
		}

		// Invia un messaggio HIL_GPS ogni nuovo messaggio gps di gazebo
		if (this->newGpsSample_)
		{
			ReadAndSendGps(simTimeUint64);
		}
		

		/**
		 * LOCKSTEP
		 * metti in pausa la simulazione (mediante la chiamata socket bloccate) e aspetti
		 * che l'autopilot calcoli i comandi per gli attuatori.
		 */
  	}		
}



void InfinityBridge::ReadAndSendSensors(uint64_t simTime)
{
	std::lock_guard<std::mutex> lock(this->imuMutex_);
	float accelX = static_cast<float>(this->latestImuSample_.linear_acceleration().x());
	float accelY = static_cast<float>(this->latestImuSample_.linear_acceleration().y());
	float accelZ = static_cast<float>(this->latestImuSample_.linear_acceleration().z());

	float gyroX = static_cast<float>(this->latestImuSample_.angular_velocity().x());
	float gyroY = static_cast<float>(this->latestImuSample_.angular_velocity().y());
	float gyroZ = static_cast<float>(this->latestImuSample_.angular_velocity().z());
	
	this->newImuSample_ = false;

	uint32_t updatedMaskMag = this->newMagSample_ ? (HIL_SENSOR_UPDATED_XMAG | HIL_SENSOR_UPDATED_YMAG | HIL_SENSOR_UPDATED_ZMAG) : 0;
	uint32_t updatedMaskBaro = this->newBaroSample_ ? (HIL_SENSOR_UPDATED_ABS_PRESSURE | HIL_SENSOR_UPDATED_DIFF_PRESSURE | HIL_SENSOR_UPDATED_PRESSURE_ALT) : 0;	


	uint32_t updatedMask =  HIL_SENSOR_UPDATED_XACC |
							HIL_SENSOR_UPDATED_YACC |
							HIL_SENSOR_UPDATED_ZACC |
							HIL_SENSOR_UPDATED_XGYRO |
							HIL_SENSOR_UPDATED_YGYRO |
							HIL_SENSOR_UPDATED_ZGYRO |
							updatedMaskMag |
							updatedMaskBaro |
							HIL_SENSOR_UPDATED_TEMPERATURE;
	
	float magX = 0.0;
	float magY = 0.0;
	float magZ = 0.0;
	if(this->newMagSample_){
		magX = static_cast<float>(this->latestMagSamples_.field_tesla().x());
		magY = static_cast<float>(this->latestMagSamples_.field_tesla().y());
		magZ = static_cast<float>(this->latestMagSamples_.field_tesla().z());

		// Conversione in gauss
		magX *= 1e4;  
		magY *= 1e4;  
		magZ *= 1e4;  
		this->newMagSample_ = false;
	}

	float abs_pressure = 0.0;
	float pressure_alt = 0.0;
	float diff_pressure = 0.0;
	float temperature = 0.0;
	if (this->newBaroSample_)
	{
		this->newBaroSample_ = false;
	}
	

	mavlink_message_t new_msg;
	mavlink_msg_hil_sensor_pack(
		1, MAV_COMP_ID_AUTOPILOT1, new_msg, simTime, 
		accelX, accelY, accelZ,
		gyroX, gyroY, gyroZ,
		magX, magY, magZ,
		abs_pressure, diff_pressure, pressure_alt,
		temperature, updatedMask, 0
	);

	sendMavlinkMsg(new_msg);
}

void InfinityBridge::ReadAndSendGps(uint64_t simTime){
	std::lock_guard<std::mutex> lock(this->gpsMutex_);


	mavlink_message_t new_msg;
	mavlink_msg_hil_gps_pack();

	sendMavlinkMsg(new_msg);
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

void InfinityBridge::sendMavlinkMsg(mavlink_message_t &msg)
{
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	int len = mavlink_msg_to_send_buffer(buffer, &msg);
	if (len > 0)
	{
		autopilot_sock_.send(buffer, len);
	}
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
