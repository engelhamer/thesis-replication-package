![cover_img](https://user-images.githubusercontent.com/39912581/132099655-fa2975ce-99ab-409f-b9b7-560f4fad554f.PNG)

## Repository structure
This repository is organised as follows:
- **/robot_runner**: Contains the experiment configuration;
- **/data**: Contains the experiment's run table (with aggregated measurements per run as well as their configuration);
- **/analysis**: Contains all scripts used for data analysis;
- **/figures**: Contains all figures created during data analysis;
- **/ros_packages**: Contains all custom ROS packages for measuring power consumption, CPU utilisation and memory utilisation.

## Robot Runner
The experiment is designed and executed using [Robot Runner](https://github.com/s2-group/robot-runner): a software framework for orchestration and (partial) automation of measurement-based robotics experiments. The structure is as follows:

    /robot_runner
     |--- /documentation                                Robot Runner documentation
     |--- /robot-runner                                 Robot Runner core
     |--- /experiments                                  
     |     |--- /slam_experiment                        All experiment scripts
     |     |     |--- /turtlebot                        The mission package running on the Turtlebot
     |     |     |     |--- /config                     Configuration files for all algorithms and parameter settings in the experiment
     |     |     |     |--- /launch                     Launch files for all evaluated algorithms
     |     |     |     |--- /rosbags                    Movement instructions for traversing the arenas
     |     |     |--- config.py                         Definition of all steps in the experiment associated with event hooks in Robot Runner

## Data
The full dataset of all measurements is too large for GitHub, but can be obtained from [Google Drive](https://drive.google.com/drive/folders/1wDQUXRsSxRiEIMGsi2PvDu5d8a4pqcsM?usp=sharing). Each subdirectory of contains the measurement data related to one of the 320 experiment runs. The structure is as follows:

    /data
     |--- /run_<i>                                      The results for the ith experiment run
     |     |--- <i>.bag                                 Topic data recorded during the experiment
     |     |--- <i>.pgm                                 A map created using map_server after completing traversal of the arena
     |     |--- <i>.txt                                 Log of all terminal output for the current run
     |     |--- cpu_mem_measurements.csv                Raw measurements for CPU and memory utilisation
     |     |--- energy_measurements.csv                 Raw measurements for power consumption
     |     |--- final_map.csv                           Final occupancy grid, used for analysis
     |     |--- rosbag_measurements.csv                 Time, topic and size for all ROS messages
     |     |--- trimmed_cpu_mem_measurements.csv        Measurements trimmed to 1 second after the last nonzero velocity, to avoid the influence of profiler timing issues
     |     |--- trimmed_energy_measurements.csv         Measurements trimmed to 1 second after the last nonzero velocity, to avoid the influence of profiler timing issues
     |
     |--- run_table.csv                                 Aggregated experiment results
     |--- run_table_map_data.csv                        Aggregated experiment results including map analysis metrics (computed after the experiment was performed)

## Analysis
Each subdirectory of */analysis* contains R and Jupyter notebooks for a particular metric in the experiment:

    /analysis
     |--- /<metric>                                     
     |     |--- <metric>_arena_circular.Rmd             R notebook containing the statistical analysis for the circular arena
     |     |--- <metric>_arena_circular.nb.html         HTML export of the above R notebook
     |     |--- <metric>_arena_point_to_point.Rmd       R notebook containing the statistical analysis for the point to point arena     
     |     |--- <metric>_arena_point_to_point.nb.html   HTML export of the above R notebook
     |
     |--- correlations_analysis.Rmd                     R notebook containing the statistical analysis of correlation between metrics (using Spearman's correlation coefficient)
     |--- correlations_analysis.nb.html                 HTML export of the above R notebook
     |--- result_boxes_overview.Rmd                     R notebook containing the difference computations for all result boxes in the thesis document
     |--- result_boxes_overview.nb.html                 HTML export of the above R notebook
     |--- data_exploration.ipynb                        Jupyter notebook containing all scripts for plotting measurements over time
      
For each of the metric and arena combinations, the analysis starts by obtaining descriptive statistics and plotting the data as bar and box plots. Then, the assumptions for ANOVA are tested and (PERM)ANOVA is carried out. As post hoc tests, Tukey's HSD or Dunn's test are used. 

## ROS packages
The profilers used in this experiment are documented in their respective folders (`ros1-ina219-profiler-service` and `ros1-resource-profiler-service`).
