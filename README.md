# infinity-gazebo-plugin
Questo plugin sviluppato appositamente per infinity-autopilot si occupa di inviare all'autopilot i messaggi mavlink HIL contenenti i dati del simulatore, inoltre riceve i comandi degli attuatori e li applica al simulatore.
La comunicazione con l'autopilot avviene mediante socket UDP.

## Build

Build the plugin
```bash
mkdir build && cd build
cmake ..
make
```

Export the path to the build
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

Set the plugin in the world SDF file
```yaml
<plugin
    filename="libInfinityBridge.so"
    name="infinity_plugin::InfinityBridge">
    <imu_topic>/imu</imu_topic>
</plugin>
```
