# Overview
This project is part of the Udacity Self-Driving Car Nanodegree.

Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project I implemented a 2 dimensional particle filter in C++. This particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter also get observation and control data.

[//]: # (Image References)
[success]:   ./success.png

## Brief description of functions
The code was developed by Udacity and I modified src/particle_filter.cpp to implement the particle filter.
In ```ParticleFilter::init``` I initialize the particles (I found that 500 particles yield good accuracy and speed) with weight of 1 and an initial position randomly distributed around the given x-y coordinates. This way, all the 500 Particles are in the vicinity of the vehicle (assuming a reasonably precise GPS) rather than all over the map.

In ```ParticleFilter::prediction``` each particle moves using a kinematic bicycle model, with velocity and yaw_rate measurements as they would come from a noisy sensor, in addition gaussian noise is added to simulate the model uncertainty.

In ```ParticleFilter::dataAssociation``` I associate landmarks with observations, basically each observation will me assigned a landmark_id belonging to the closest match (geometrically) observation-landmark.

In ```ParticleFilter::updateWeights``` (The meat of the algorithm) I update particle weights based on how accurately the particle represent the vehicle, by using the observations coming from the sensors. 
Weights are calculated as the product of probability distributions depending on observations/landmark: matched observations closer to the real landmark will produce a high probability, hence a higher weight. The product of each individual probability gives the total particle weight.
The final result is that the particle with the highest weight is the best candidate to approximate the position of the vehicle.

The ```main.cpp``` takes care of the resampling step, that weeds-out over time particles with low weight and increase the importance of particles with higher weight (essentially from a bucket of 500 particles we have after many iterations still 500 particles, but many are instances of the same high-weight particles). The resampling routine is in ```ParticleFilter::resample```.
This is then proved using the simulator previously described, which automatically detects if the accuracy is high enough to "pass" the detection, and the project objective. Also, the algorithm should be efficient enough to take less than 100seconds to fully complete the simulation. The simulator shows the best particle position, represented as blue circle that ideally overlaps with the car which is the ground truth. Smaller circles are landmarks and green lines represent the observation vectors coming from the sensors.

![alt text][success]






## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)





## Other project instructions (useful during development)
Note that the programs that need to be written to accomplish the project are src/particle_filter.cpp, and particle_filter.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


Your job is to build out the methods in `particle_filter.cpp` until the simulator output says:

```
Success! Your particle filter passed!
```

# Implementing the Particle Filter
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

The only file you should modify is `particle_filter.cpp` in the `src` directory. The file contains the scaffolding of a `ParticleFilter` class and some associated methods. Read through the code, the comments, and the header file `particle_filter.h` to get a sense for what this code is expected to do.

If you are interested, take a look at `src/main.cpp` as well. This file contains the code that will actually be running your particle filter and calling the associated methods.

## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.

## Success Criteria
If your particle filter passes the current grading code in the simulator (you can make sure you have the current version at any time by doing a `git pull`), then you should pass!

The things the grading code is looking for are:


1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.
