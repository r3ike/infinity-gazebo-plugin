#ifndef MY_SYSTEM_HH_
#define MY_SYSTEM_HH_

#include <gz/sim/System.hh>
#include <memory>

namespace my_plugin
{
  class MySystem :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    public: MySystem();
    public: ~MySystem() override = default;

    // Chiamato all'avvio della simulazione per configurare il plugin
    public: void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) override;

    // Chiamato a ogni step prima dell'aggiornamento della fisica
    public: void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

    private:
      double customFactor_{1.0};
  };
}

#endif