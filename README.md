# Scripts and Instructions

These scripts and instructions were written for the dem scm calibration paper. 

The data and figures obtained in the paper were calculated and created by the scripts above.

Clone the scripts and follow the instructions to reproduce results in the paper.

We ran the DEM demos using NVIDIA RTX 4090 GPU, ran SCM demos using Intel 14900K CPU and drew figures with the local Spyder software in Anaconda Navigator.

The calculation process were all accomplished on a linux system.


## Step 1 - Clone Chrono and scripts for the paper, then build Chrono software and run DEM simulation，containing the plate sinkage test, annulus shear test, 3 types of single wheel test and the full rover test

Create a work directory in your home directory like below: 

```~/dem_scm_calibration```

Go to this work directory, clone the Chrono repository and checkout to release 8.0 version: 

```git clone https://github.com/projectchrono/chrono.git --recursive -b release/8.0```

Clone the scripts repository for the paper: 

```git clone https://github.com/Zym13430941761/dem_scm_calibration_scripts.git```

Make some necessary changes to the Chrono source files by replacing them with the provided files: 

```cp dem_scm_calibration_scripts/Demos/DEM_demos/. chrono/src/demos/gpu/```

Create a directory to build Chrono software and go to it:

```mkdir build_GPU```

```cd build_GPU```

Build Chrono in this directory:

```ccmake ../chrono```

Configure it, open the ```GPU_module``` and generate it. Compile the demos using:

```make -j```

Update the json files and the obj files:

```cd ..```

```unzip dem_scm_calibration_scripts/Demos/models.zip```

```cp dem_scm_calibration_scripts/Demos/models/. build_GPU/data/models/```

```cp dem_scm_calibration_scripts/Demos/json file/. build_GPU/data/gpu/```

Copy the shell programs to the bin directory:

```cp dem_scm_calibration_scripts/Demos/Shell/. build_GPU/bin/```

Create the data storage by copying prepared folder:

```cp dem_scm_calibration_scripts/Data collection/. build_GPU/bin/DEMO_OUTPUT/GPU/```

Enter the bin directory and first create the terrain environment:

```cd build_GPU```

```cd bin```

```sudo chmod +x terrain.sh```

```./terrain.sh```

Copy the created checkpoint files into the corresponding checkpoint files:

```sudo chmod +x copy_checkpoint_file.sh```

```./copy_checkpoint_file.sh```

Run the simulations:

```sudo chmod +x simulation.sh```

```./simulation.sh```

## Step 2 - Run the calibration demos to calibrate the six parameters for the SCM  model

Copy the "plt demos" into the "Figure" directory for convinience:

```cp dem_scm_calibration_scripts/Demos/plt_demos/. dem_scm_calibration_scripts/Figure/```

Locally use a python software (like Anaconda) to run the demos(it is noted that we need first download a Pymc3 and activate into python):

First run the demo ```Sinkage.py``` and ```Annulus.py```, copy the data in ```01_plot_plate_sinkage/plate_selected_force_sinkage_points_dem.txt```， ```02_plot_annulus_shear/annulus_max_torque_vs_load_dem_vs_scm.txt``` and ```02_plot_annulus_shear/annulus_selected_torque_points_for_each_load.txt``` into demo ```Sinkage_parameter.py```, ```Annulus_parameter.py``` and ```K_s.py``` seperately. 

Then run the three demos ```Sinkage_parameter.py```, ```Annulus_parameter.py``` and ```K_s.py``` to calibrate the six parameters. 

We are supposed to change the number of chains in the 3 demos in line 87, 71, 91 so that the calibration would be more accurate with the growth of "nchains". 

Then, write the calibrated parameters into ```Sinkage.py``` and ```Annulus.py``` so that we can get the comparison figure between DEM and SCM.

Run the demos ```Sinkage.py``` and ```Annulus.py``` again to gain the figures.

Figures about absolute error of the SCM results compared with the DEM solution at 1 s, 2 s, and 3 s in demo ```Annulus.py``` requires us to change the "t" in line 112 from 1s to 2s and 3s.

Follow the instructions in line 226, 233, 244, 262, 273.

According to the ncfiles created by the Pymc3 software, we can optimize the figures by running ```plot.py```.

## Step 3 - Run the SCM demos using pychrono

Create a directory to build Pychrono software and go to it:

```mkdir build_py```

```cd build_py```

Build Pychrono in this directory:

```ccmake ../chrono```

Configure it, open the ```ENABLE_MODULE_PYTHON```, ```ENABLE_MODULE_VEHICLE``` and generate them. Compile the demos using:

```make -j 12```

Copy the SCM demos into the bin directory:

```cd ..```

```cp dem_scm_calibration_scripts/Demos/SCM_demos/. build_py/bin/```

Copy the six calibrated parameters into ```Singlewheel.py``` and ```Rover_test.py```

Run the SCM simulation under the bin directory:

```cd build_py```

```cd bin```

```python3 Rover_test.py```

```python3 Singlewheel.py```

## Step 4 - Draw the singlewheel test/full rover test results 

Copy the TXT files from ```py_demos/bin``` into ```dem_scm_calibration_scripts/Figure/04_plot_single_wheel/```

Run the ```Singlewheel_Test.py``` and change the ```HMMWV Wheel``` symbols to ```Cylinder Wheel``` and  ```Rover Wheel```, run the demo two more times

Then copy the data calculated in  ```Rover_test.py``` using SCM into demo ```Rover_new.py```, the data were in the files ```Rover_DBP_Torque.txt``` and ```DBP_Torque_vs_Time_ROVER_Wheel_SCM.txt```

By running the demo ```Rover_new.py```, we can gain the comparison figures.






