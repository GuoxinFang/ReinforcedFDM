# Reinforced FDM: 
## Multi-Axis Filament Alignment with Controlled Anisotropic Strength

Updated： July-02-2021

This is the code to slice curved 3D printing layer and generate toolpath based on the stress distribution. The generated toolpath can be printed by multi-axis 3D printer, which results a higher strength in fabricated model.

[Reinforced FDM: Multi-Axis Filament Alignment with Controlled Anisotropic Strength]
* Guoxin Fang, Tianyu Zhang, Sikai Zhong, Xiangjia Chen, Zichun Zhong, and Charlie C.L. Wang. 2020. 
[ACM Transactions on Graphics (Proceedings of SIGGRAPH Asia 2020)], vol.39, no.6, article no.204, 2020.

## Installation

Please compline the code with QMake file 'ReinforcedFDM.pro'.

Tested platform:

MacOS: QT Creator

Windows (recommand): Visual Studio + QT-plugin (tested QT version: 5.12.10 + msvc2017_64)

Remark: if you are using Visual Studio, after using QT VS Tool to open the .pro file and generate the project,
(1) Set 'shapeLab' as the start up project
(2) Enable OpenMP to get best performace at:
ShapeLab Project Property -> 'Configuration Proerties' -> c/c++ -> Language -> Open MP Support -> Select 'Yes (/openmp)'
(3) Open Console at: ShapeLab Project Property -> 'Configuration Proerties' -> Linker -> System -> Select 'Console (/SUBSYSTEM:CONSOLE)' in 'SubSystem'

## Usage

First input tetrahedral mesh into the system. Two sample model is given 'Bunnyhead.tet' and 'topopt_new.tet'. You can either drag the file into the blank area of the UI or open the panel "file -> open".

#Step 1: 
Input FEA simulation result and compute principle stress direction as vector field by clicking bottom 'Step 1: Input FEA Result'.

The color map on the mesh visulize the stress distribution. You can draw the stress vector field by using function "View -> Profile" or direct click the "Profile" bottom.
You can also change the percentage of the critical region by changing the value below 'Step 1: Input FEA Result' bottom and click 'Change ratio' to redraw the field. Notice the parameter used in our paper is already set as the initial value in UI.

#Step 2: 
Compute Field (for both vector field and scalar field) by clicking bottom 'Step 2: Field Computing'. This process may take some time.

After finish computing the field, you can draw the scalar field at by using function "View -> Node" or direct click the "Node" bottom.
The vector field can be draw by using function "View -> Profile" or direct click the "Profile" bottom. (the perivious stress field will be replaced).

#Step 2: 
