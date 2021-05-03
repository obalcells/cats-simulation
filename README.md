# CATS Simulation

<img src="https://github.com/catsystems/cats-docs/blob/main/logo/PNG/logo_with_smile.png" alt = "CATS Logo" width="300" height="300">

*Always land on your paws*

## Open Source
All CATS code is open source and can be used free of charge without warranty. 

## Introduction

This repo includes all flight simulation and testing for the kalman filter.
The *Analysis* folder includes analysis of sensor data as well as 3D plotting of the orientation kalman filter which is currently in development.
The *sim* folder includes a python simulation where sensor data is loaded and where the attitude and velocity kalman filter can be tested.

## Next Steps

This repo will include, in a later step

1. A simulator in C++ which can feed simulation and true sensor data to the CATS FC.
2. A simple simulator for rockets to generate trajectories and fake sensor data.
