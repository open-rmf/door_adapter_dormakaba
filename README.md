# door_adapter_dormakaba
An RMF adapter for integrating smart doors provided by [Dormakaba](https://www.dormakaba.com/)
This adapter is tested with the Dormakaba EL301 and ED250 models.
It requires the Dormakaba IoT connector to be fitter to the above door operators.
For more information on the hardware, please contact Dormakaba.

## Requirements
* [Open-RMF](https://github.com/open-rmf/rmf)

## Setup
1. Clone this repository into the RMF workspace
2. Source Open-RMF
3. `colcon build --packages-up-to door_adapter_dormakaba --cmake-args -DCMAKE_BUILD_TYPE=Release`

## Run the door adapter

Make sure to source the workspace first.

```
ros2 run door_adapter_dormakaba door_adapter_dormakaba -c CONFIG_FILE

```

The schema for the config file is provided [here](door_adapter_dormakaba/config.yaml).
Credentials to access the door's cloud API will be provided by Dormakaba.
