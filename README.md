# Climate Active Envelopes

**Quick links:** [COMPAS](https://compas.dev/)

## Requirements

* Operating System: **Windows 10 Pro** <sup>(1)</sup>.
* [Rhinoceros 3D 7.0](https://www.rhino3d.com/)
* [Anaconda Python Distribution](https://www.anaconda.com/download/): 3.x
* [Visual Studio Code](https://code.visualstudio.com/)
* [GitHub Desktop](https://desktop.github.com/)

## Dependencies

* [Assembly Information Model](https://github.com/augmentedfabricationlab/assembly_information_model)
* [Climate_Active_Envelopes_Fabrication](https://github.com/augmentedfabricationlab/climate_active_envelopes_fabrication)

## Getting Started

### 1. Setting up the Anaconda environment with all dependencies

Execute the commands below in Anaconda Prompt:

#### Install Compas 

    (base) conda create -n cae -c conda-forge compas
    (base) pip install compas==2.0.0-alpha.1
    (base) conda activate cae
    
#### Install on Rhino
    
    (cae) python -m compas_rhino.install -v 7.0
    
#### Verify Installation

    (cae) pip show compas
    
    Name: COMPAS
    Version: 2.0.0a1
    Summary: The COMPAS framework
    ....

## Install Dependencies

    (cae) conda install git
    
#### Assembly Information Model
    
    (cae) python -m pip install git+https://github.com/augmentedfabricationlab/assembly_information_model@master#egg=assembly_information_model
    (cae) python -m compas_rhino.install -p assembly_information_model -v 7.0
    
    
#### Install on Rhino

    (cae) python -m compas_rhino.install -v 7.0

### 2. Cloning and installing the repository

#### Repository Cloning
* Create a workspace directory: C:\Users\YOUR_USERNAME\workspace
* Open Github Desktop and clone [this repository](https://github.com/augmentedfabricationlab/climate_active_envelopes) into you workspace folder.

#### Install the repository in editable mode
    (cae) python -m pip install -e <your_path>/<your_repository_name>
    (cae) python -m compas_rhino.install -p climate_active_envelopes -v 7.0

## Credits

This package was created by Julia Fleckenstein at `@augmentedfabricationlab`
