# Scripts and Instructions

These scripts and instructions were written for the dem scm calibration paper. 

The data and figures obtained in the paper were calculated and created by the scripts above.

Clone the scripts and follow the instructions to reproduce results in the paper.

We ran the DEM demos using NVIDIA RTX 4090 GPU, ran SCM demos using Intel 14900K CPU and drew figures with the local Spyder software in Anaconda Navigator.

The calculation process were all accomplished on a linux system.


## Step 1 - Clone Chrono and scripts for the paper, then build Chrono software and run DEM simulation

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

```make -j 12```

Update the json files and the obj files:

```cd ..```

```cp dem_scm_calibration_scripts/Demos/models/. build_GPU/data/models/```

```cp dem_scm_calibration_scripts/Demos/json file/. build_BPU/data/gpu/```

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


## Step 2 - Run the SCM demos using pychrono





