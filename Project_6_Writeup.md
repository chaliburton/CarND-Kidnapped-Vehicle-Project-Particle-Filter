# [Localization with Particle Filter Project Writeup]
Udacity Self-Driving Car Engineer Nanodegree Program
Chris Haliburton
12/19/2019

## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)
Please review the associated readme with this directory for more information.

## Writeup
This project can also be found [here](https://github.com/chaliburton/CarND-Kidnapped-Vehicle-Project-Particle-Filter)
In this project I was provided with initial code for reading in GPS and sensor data converted to 2D landmarks in vehicle coordinates.  The code interfaces with a websocket where a vehicle moves in a map with known landmarks.
The starter code precluded functionality of the following code files:
1. particle_filter.cpp

Additionally, a function was added to helper_functions.h

[image1]: ./images/Success.jpg "Successful Screenshot of Particle Filter"

## [From the Project Rubric:]
Does your particle filter localize the vehicle to within the desired accuracy?
  Yes, the automatic grader outputs: "Success! Your particle filter passed!" see ![alt text][image1]

Does your particle run within the specified time of 100 seconds?
  Yes, the automatic grader outputs: "Success! Your particle filter passed!"

Does your code use a particle filter to localize the robot?
  Yes, this implements a particle filter to localize the robot given initial GPS coordinates.

## [particle_filter.cpp Writeup]
This code defines the initial particle filter functions needed for the program to run.
Lines 8-20 contain libraries, dependencies and declarations.

Lines 22-40 contain the initialization function for the particle filter.  The number of particles in the filter is set and then each particle is randomly initialized from the GPS coordinates provided by utilizing the sensor's standard deviation.

Lines 42-66 contain the prediction step of the particle filter.  Depending on the turning rate (yaw_rate) each particle's position is updated based on either a straightline motion equation to avoid dividing by zero or a curved motion equation.

Lines 68-83 contain the dataAssociation step of the particle filter which conducts the nearest neighbour pairing for all sensor observations with landmarks predicted to be in range of the vehicle's sensor.  This function is called from the updateWeights function.

Lines 85-147 contain the updateWeights step of the particle filter.  In this step the weights of each particle are updated based on performing a transform from particle-coordinates to map-coordinates, landmarks outside of the sensor range are filtered for the timestep and passed to dataAssociation, the new probability for each particle is calculated based on the associated landmarks and the particle's weight is then normalized using the total weight of each particle.

Lines 149-159 contain the resample function that uses std::discrete_distribution to resample given the vector of particle weights

## [helper_functions.h Writeup]
A function "multiv_prob" was borrowed from the lectures and added to help calculate the weight using the mult-variate Gaussian distribution.
