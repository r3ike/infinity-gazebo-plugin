#include "MySystem.hh"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <iostream>

using namespace my_plugin;

MySystem::MySystem()
{
}

void MySystem::Configure(
	const gz::sim::Entity &_entity,
	const std::shared_ptr<const sdf::Element> &_sdf,
	gz::sim::EntityComponentManager &_ecm,
	gz::sim::EventManager &/*_eventMgr*/)
{
  // Lettura parametri dal file SDF (se presenti)
  if (_sdf->HasElement("custom_factor"))
  {
	this->customFactor_ = _sdf->Get<double>("custom_factor");
  }

  // Ottenere il nome dell'entità a cui è attaccato il plugin
  auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
  std::string name = nameComp ? nameComp->Data() : "Sconosciuta";

  std::cout << "[MySystem] Plugin configurato con successo sull'entità: " 
			<< name << " (Fattore: " << this->customFactor_ << ")" << std::endl;
}

void MySystem::PreUpdate(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
  // _info contiene informazioni come _info.simTime e _info.dt
  if (!_info.paused)
  {
	// Esempio: log stampato ogni secondo di simulazione
	static auto lastPrint = _info.simTime;
	if (_info.simTime - lastPrint >= std::chrono::seconds(1))
	{
	  std::cout << "[MySystem] Tempo di simulazione: " 
				<< std::chrono::duration<double>(_info.simTime).count() << "s" << std::endl;
	  lastPrint = _info.simTime;
	}
  }
}

// Registrazione del plugin tramite le macro di Gazebo
GZ_ADD_PLUGIN(
	my_plugin::MySystem,
	gz::sim::System,
	my_plugin::MySystem::ISystemConfigure,
	my_plugin::MySystem::ISystemPreUpdate
)

// Alias opzionale per semplificare il caricamento nel file SDF
GZ_ADD_PLUGIN_ALIAS(my_plugin::MySystem, "my_plugin::MySystem")