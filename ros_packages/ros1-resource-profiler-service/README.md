# ros1-resource-profiler-service
Provides a ROS1 server and client for profiling resource utilisation on the robotic system, orchestrated from a remote pc.
## Usage
### Robotic system
Run the profiling server:
```
rosrun resource_profiler_service resource_profiler_server.py
```
### Remote PC
Initialise the profiler client and call the services to start or stop measurements:
```
from resource_profiler_service.resource_profiler_client import ResourceProfilerClient

# Initialise the profiler
resource_profiler = ResourceProfilerClient()

# Start measurements, writing to file 'id' (on the robotic system)
resource_profiler.start_measurement(id)

# Stop and obtain measurements
response = resource_profiler.stop_measurement(id)
```
The `response` will be of the form:
```
{
  'timestamps': [t_1, t_2, t_3, ...],
  'cpu': [cpu_t1, cpu_t2, cpu_t3, ...],
  'mem': [mem_t1, mem_t2, mem_t3, ...],
}
```
where each of `t_1` until `t_n` are epoch timestamps in milliseconds, corresponding to the cpu and memory measurements at the same index.
