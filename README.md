# Hexapod Gait Adaptation to Leg Failure

This code in this repository enables gait adaptation to failure on a custom Hexapod robot shown below using the [Intelligent Trial & Error (IT&E) algorithm](https://doi.org/10.1038/nature14422), and investigates the impact of behaviour-performance map size on adaptation performance.

<p align="center">
  <img src="cover_image.png" width="400"/>
</p>

<p align="center">
  <i> Hexapod platform on a generated behaviour-performance map </i>
</p>

`Code` contains the hexapod simulation, MAP-Elites, M-BOA, generated maps, and results\
`Hexapod Control` contains the modified onboard C++ gait controller code for the robot\
`Hexapod SolidWorks` contains the updated SolidWorks model of the robot

## Videos
[Adapting to reality gap](https://youtu.be/6fp-Spu_-Wc)\
[Adapting to Failure Scenario S1](https://youtu.be/4rsNQu46i6c)\
[Adapting to Failure Scenario S2](https://youtu.be/6fp-Spu_-Wc)

## Dependencies
- numpy
- pybullet
- matplotlib
- sklearn
- mpi4py
- GPy
- scipy
