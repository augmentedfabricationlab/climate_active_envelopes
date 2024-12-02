# Climate Active Envelopes - Wall Design

**Quick links:** [COMPAS](https://compas.dev/compas/latest/index.html) [Ladybug Tools](https://www.food4rhino.com/en/app/ladybug-tools)

## Requirements

* Operating System: **Windows 10 Pro** <sup>(1)</sup>.
* [Rhinoceros 3D 8.0](https://www.rhino3d.com/)
* [Anaconda Python Distribution](https://www.anaconda.com/download/): 3.x
* [Visual Studio Code](https://code.visualstudio.com/)
* [GitHub Desktop](https://desktop.github.com/)
* [Ladybug Tools](https://www.food4rhino.com/en/app/ladybug-tools)

## Dependencies

* [Assembly Information Model](https://github.com/augmentedfabricationlab/assembly_information_model)

## Getting Started

### 1. Setting up the Anaconda environment with all dependencies

Execute the commands below in Anaconda Prompt:

#### Install Compas and Compas Fab

    (base) conda create -n cae -c conda-forge compas_fab
    (base) conda activate cae

#### Verify Installation

    (cae) python -m compas
            Yay! COMPAS is installed correctly!

#### Install on Rhino (as admin)

    (cae) python -m compas_rhino.install -v 8.0

#### Install compas and compas_fab for python3 in the following folder directory

    (cae) cd .rhinocode\py39-rh8
    (cae) python.exe -m pip install compas compas_fab

## Install Dependencies

    (cae) conda install git
    
## Assembly Information Model

    (cae) python -m pip install git+https://github.com/augmentedfabricationlab/assembly_information_model@master#egg=assembly_information_model 
    (cae) python -m compas_rhino.install -p assembly_information_model -v 8.0
    
### 2. Cloning and installing the repository

#### Repository Cloning
* Create a workspace directory: C:\Users\YOUR_USERNAME\workspace
* Open Github Desktop and clone [this repository](https://github.com/augmentedfabricationlab/climate_active_envelopes) into you workspace folder.

## Credits

This package was created by `Julia Fleckenstein` <julia.fleckenstein@tum.de> [@ augmentedfabricationlab](https://github.com/augmentedfabricationlab)
