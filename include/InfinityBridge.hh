#ifndef MY_SYSTEM_HH_
#define MY_SYSTEM_HH_

#include <gz/transport/Node.hh>
#include <gz/sim/System.hh>

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Imu.hh>

#include "mavlink/common/mavlink.h"
#include "UdpSocket.hh"

#include <memory>
#include <mutex>

namespace infinity_plugin
{
    class InfinityBridge :
        public gz::sim::System,
        public gz::sim::ISystemConfigure,
        public gz::sim::ISystemPreUpdate
    {
        public: InfinityBridge();
        public: ~InfinityBridge() override = default;

        // Chiamato all'avvio della simulazione per configurare il plugin
        public: void Configure(
                const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_eventMgr) override;

        // Chiamato a ogni step prima dell'aggiornamento della fisica
        public: void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override;

        private:
			int simPort_{14550};
			int autopilotPort_{14551};

			std::string imuTopic_;
			std::string autopilotAddr_;

			UdpSocket autopilot_sock_;
			gz::transport::Node node_;

			void ReadSdfElements(const std::shared_ptr<const sdf::Element> &_sdf);

			std::mutex imuMutex_;
			std::mutex gpsMutex_;
			std::mutex magMutex_;
			std::mutex baroMutex_;

			bool newImuSample_{false};
			bool newGpsSample_{false};
			bool newBaroSample_{false};
			bool newMagSample_{false};

			gz::msgs::IMU latestImuSample_;
			gz::msgs::NavSat latestGpsSample_;
			gz::msgs::FluidPressure latestBaroSample_;
			gz::msgs::Magnetometer latestMagSample_;

			// callback topic gz
			void OnImuData(const gz::msgs::IMU& msg);
			void OnGpsData(gz::msgs::NavSat& msg);
			void OnBaroData(gz::msgs::FluidPressure& msg);
			void OnMagData(gz::msgs::Magnetometer& msg);
			//void OnLidarData();

			void sendMavlinkMsg(mavlink_message_t& msg);


			void ReadAndSendSensors(uint64_t simTime);
			void ReadAndSendGps(uint64_t simTime);
    };
}

#endif
