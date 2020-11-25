# Implementation of IT&E on the RARL Hexapod

This project enables gait adaptation of the University of Cape Town's Robotic Agents Research Lab Hexapod platform using the Intelligent Trial & Error (IT&E) algorithm ([Cully et al. 2015](https://doi.org/10.1038/nature14422)) as part of my final year undergraduate project. This project was supervised by Ms Leanne Raw and co-supervised by Dr Geoff Nitschke.

`Code` contains the hexapod simulation, MAP-Elites, M-BOA, generated maps, and results\
`Hexapod Control` contains the modified onboard C++ control code for the robotic platform\
`Hexapod SolidWorks` contains the updated SolidWorks files for the hexapod platform with the required coordinate frames

# Dependencies
- numpy
- pybullet
- matplotlib
- sklearn
- mpi4py
- GPy
- scipy