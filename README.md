# infinity-gazebo-plugin

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
    filename="libMySystem.so"
    name="my_plugin::MySystem">
    <custom_factor>2.5</custom_factor>
</plugin>
```