# Implementation of IT&E on the RARL Hexapod

This code in this repository enables gait adaptation to failure on a custom Hexapod robot shown below using the [Intelligent Trial & Error (IT&E) algorithm](https://doi.org/10.1038/nature14422), and investigates the impact of behaviour-performance map size on adaptation perfromance.

<p align="center">
  <img src="cover_image.png" width="400"/>
</p>

<p align="center">
  <i> Hexapod platform on a generated behaviour-performance map </i>
</p>

`Code` contains the hexapod simulation, MAP-Elites, M-BOA, generated maps, and results\
`Hexapod Control` contains the modified onboard C++ control code for the robotic platform\
`Hexapod SolidWorks` contains the updated SolidWorks model of the robot

Videos of the project can be found [here](https://drive.google.com/drive/folders/18nBqK6PnA0IYt2r0Ebi3O82trxsxnqot?usp=sharing)

# Dependencies
- numpy
- pybullet
- matplotlib
- sklearn
- mpi4py
- GPy
- scipy
