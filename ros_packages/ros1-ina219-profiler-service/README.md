# ros1-ina219-profiler-service
Provides a ROS1 server and client for profiling power consumption on the robotic system, orchestrated from a remote pc.
## Usage
### Robotic system
Run the profiling server:
```
rosrun ina219_profiler_service ina219_profiler_server.py
```
**Note:** make sure the correct device address (e.g. `/dev/ttyUSB1`) for your system is configured in `src/ina219_profiler_service/ina219_profiler_server.py `.
### Arduino
Load the profiler code at `src/ina219_profiler_service/ina219_profiler.ino` onto the Arduino, making sure to update the `ledPin` value to the correct pin for your circuit.
This can be either an external LED (e.g. at pin 10) or a built-in one (at pin 13).
The LED will be lit while profiling is in progress.
### Remote PC
Initialise the profiler client and call the services to start or stop measurements:
```
from ina219_profiler_service.ina219_profiler_client import INA219ProfilerClient

# Initialise the profiler
ina219_profiler = INA219ProfilerClient()

# Start measurements
ina219_profiler.start_measurement(id)

# Stop and obtain measurements
response = ina219_profiler.stop_measurement(id)
```
The `response` will be of the form:
```
{
  'timestamps': [t_1, t_2, t_3, ...],
  'power_mw': [power_mw_t1, power_mw_t2, power_mw_t3, ...],
}
```
where each of `t_1` until `t_n` are epoch timestamps in milliseconds, corresponding to the power consumption (in milliWatts) at the same index.
