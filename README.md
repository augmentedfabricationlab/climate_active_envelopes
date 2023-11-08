# Climate Active Envelopes

**Quick links:** [COMPAS](https://compas.dev/) | [COMAS FAB](https://gramaziokohler.github.io/compas_fab/latest/)

## Requirements

* Operating System: **Windows 10 Pro** <sup>(1)</sup>.
* [Rhinoceros 3D 7.0](https://www.rhino3d.com/)
* [Anaconda Python Distribution](https://www.anaconda.com/download/): 3.x
* [Visual Studio Code](https://code.visualstudio.com/)
* [GitHub Desktop](https://desktop.github.com/)

## Dependencies
#### [Fabtory Fabrication Control](https://github.com/augmentedfabricationlab/fabtory_fabrication_control)
    (cae) conda install compas_rrc
    (cae) python -m compas_rhino.install -v 7.0
  
#### [Assembly Information Model](https://github.com/augmentedfabricationlab/assembly_information_model) 


## Getting Started

### 1. Setting up the Anaconda environment with all dependencies

Execute the commands below in Anaconda Prompt:

#### Install Compas & Compas Fab
 
    (base) conda config --add channels conda-forge
    (base) conda create -n cae compas_fab --yes
    (base) conda activate cae
    
#### Install on Rhino
    
    (cae) python -m compas_rhino.install -v 7.0
    
#### Verify Installation

    (cae) pip show compas_fab
    
    Name: compas-fab
    Version: 0.XX.X
    Summary: Robotic fabrication package for the COMPAS Framework
    ....
### 2. Cloning and installing the repository

#### Repository Cloning
* Create a workspace directory: C:\Users\YOUR_USERNAME\workspace
* Open Github Desktop and clone [this repository](https://github.com/augmentedfabricationlab/climate_active_envelopes) into you workspace folder.

#### Install the repository in editable mode
    (cae) python -m pip install -e <your_path>/<your_repository_name>
    (cae) python -m compas_rhino.install -p climate_active_envelopes -v 7.0

    (cae) python -m pip install git+https://github.com/augmentedfabricationlab/assembly_information_model@master#egg=assembly_information_model
    (cae) python -m compas_rhino.install -p climate_active_envelopes -v 7.0

## Credits

This package was created by Julia Fleckenstein at `@augmentedfabricationlab`
