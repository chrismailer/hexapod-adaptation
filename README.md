# Implementation of IT&E on the RARL Hexapod

This project enables gait adaptation of the University of Cape Town's [Robotic Agents Research Lab](http://www.rarl.uct.ac.za) Hexapod platform using the [Intelligent Trial & Error (IT&E) algorithm](https://doi.org/10.1038/nature14422) as part of my final year undergraduate project. This project was supervised by Ms Leanne Raw and co-supervised by Dr Geoff Nitschke.

<p align="center">
  <img src="cover_image.png" width="400"/>
</p>

<p align="center">
  <i> Hexapod platform on a generated behaviour-performance map (Image by Alexandra Mailer) </i>
</p>

`Code` contains the hexapod simulation, MAP-Elites, M-BOA, generated maps, and results\
`Hexapod Control` contains the modified onboard C++ control code for the robotic platform\
`Hexapod SolidWorks` contains the updated SolidWorks files for the hexapod platform

Videos of the project can be found [here](https://drive.google.com/drive/folders/18nBqK6PnA0IYt2r0Ebi3O82trxsxnqot?usp=sharing)

# Dependencies
- numpy
- pybullet
- matplotlib
- sklearn
- mpi4py
- GPy
- scipy
