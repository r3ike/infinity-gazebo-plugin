#ifndef MY_SYSTEM_HH_
#define MY_SYSTEM_HH_

#include <gz/sim/System.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/Imu.hh>
#include <memory>

namespace infinity_bridge
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
			gz::sim::Entity imuEntity_{gz::sim::kNullEntity};
			std::chrono::duration<double> imuPeriod_{0.0};		// Refresh period of the imu
			std::chrono::steady_clock::duration lastImuRead_{0};

            double customFactor_{1.0};

			void ReadAndSendSensors(gz::sim::components::Imu* imuComp);
    };
}

#endif
