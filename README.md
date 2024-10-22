# Climate Active Envelopes

**Quick links:** [COMPAS](https://compas.dev/compas/latest/index.html)

## Requirements

* Operating System: **Windows 10 Pro** <sup>(1)</sup>.
* [Rhinoceros 3D 8.0](https://www.rhino3d.com/)
* [Anaconda Python Distribution](https://www.anaconda.com/download/): 3.x
* [Visual Studio Code](https://code.visualstudio.com/)
* [GitHub Desktop](https://desktop.github.com/)

## Dependencies

* [Assembly Information Model](https://github.com/augmentedfabricationlab/assembly_information_model)
* [ABB Fabrication Control](https://compas-rrc.github.io/compas_rrc/latest/)
* [Fabtory Fabrication Control](https://github.com/augmentedfabricationlab/fabtory_fabrication_control)

## Getting Started

### 1. Setting up the Anaconda environment with all dependencies

Execute the commands below in Anaconda Prompt:

#### Install Compas and Compas Fab

    (base) conda create -n cae -c conda-forge compas_fab
    (base) conda activate cae
    
#### Verify Installation

    (cae) pip show compas
    or
    (cae) python -m compas
    
    Name: COMPAS
    Version: 2.0.0a2
    Summary: The COMPAS framework
    ....

#### Install compas for python3 in the following folder directory

    (cae) cd .rhinocode/py39-rh8
    (cae) ..../rhinocode/py39-rh8: python.exe -m pip install compas compas_fab

## Install Dependencies

    (cae) conda install git
    
#### Assembly Information Model
    
    (cae) python -m pip install git+https://github.com/augmentedfabricationlab/assembly_information_model@master#egg=assembly_information_model
    (cae) python -m compas_rhino.install -p assembly_information_model -v 8.0

#### ABB Fabrication Control

    (cae) conda install compas_rrc 
    (cae) python -m compas_rhino.install -p compas_rrc
    
#### Fabrication Manager

    (cae) python -m pip install git+https://github.com/augmentedfabricationlab/fabrication_manager@master#egg=fabrication_manager
    (cae) python -m compas_rhino.install -p fabrication_manager -v 8.0
    
#### Install on Rhino

    (cae) python -m compas_rhino.install -v 8.0

### 2. Cloning and installing the repository

#### Repository Cloning
* Create a workspace directory: C:\Users\YOUR_USERNAME\workspace
* Open Github Desktop and clone [this repository](https://github.com/augmentedfabricationlab/climate_active_envelopes) into you workspace folder.

#### Install the repository in editable mode
    (cae) python -m pip install -e <your_path>/<your_repository_name>
    (cae) python -m compas_rhino.install -p climate_active_envelopes -v 7.0

## Credits

This package was created by Julia Fleckenstein <julia.fleckenstein@tum.de> at `@augmentedfabricationlab`
