---
layout: page
title: Getting Started
---


**Quick links:** [COMPAS](https://compas.dev/compas/latest/index.html) [Ladybug Tools](https://www.food4rhino.com/en/app/ladybug-tools) [EPW Maps](https://www.ladybug.tools/epwmap/)

## Requirements

* Operating System: **Windows 10 Pro** <sup>(1)</sup>.
* [Rhinoceros 3D 8.0](https://www.rhino3d.com/)
* [Anaconda Python Distribution](https://www.anaconda.com/download/): 3.x
* [Visual Studio Code](https://code.visualstudio.com/)
* [GitHub Desktop](https://desktop.github.com/)

## Dependencies

* [Assembly Information Model](https://github.com/augmentedfabricationlab/assembly_information_model)

### 1. Setting up the Anaconda environment

Execute the commands below in Anaconda Prompt:

#### Install Compas

    (base) conda create -n cae -c conda-forge compas_fab
    (base) conda activate cae

#### Verify Installation

    (cae) python -m compas
            Yay! COMPAS is installed correctly!

#### Install compas using the file path of the Rhino 8 Python executable

Find the Rhino 8 Python executable by running the following in a terminal or command prompt:

    (cae) python -m compas_rhino.print_python_path

Your Rhino 8 Python path should look something like this:

    C:\Users\your_user_name\.rhinocode\py39-rh8\python.exe

Then you can pip install using the file path of the Rhino 8 Python executable:
    
    (cae) your_py39-rh8_path -m pip install compas compas_fab
    
#### Install Ladybug Tools to run the climate simulation
To get the latest Ladybug Tools and its simulations download and install:
* [Ladybug Tools](https://www.food4rhino.com/en/app/ladybug-tools)
* [Radiance](https://github.com/LBNL-ETA/Radiance/releases/tag/27dbb0e0) 

### 2. Cloning and installing the repositories

#### Repository Cloning
* Create a workspace directory: C:\Users\YOUR_USERNAME\workspace
* Open Github Desktop and clone [this repository](https://github.com/augmentedfabricationlab/climate_active_envelopes.git)) and the [Assembly Information Model](https://github.com/augmentedfabricationlab/assembly_information_model) into you workspace folder.

#### Make the Assembly Information Model accessible in Rhino 8

    (cae) your_py39-rh8_path -m pip install your_filepath_to_assembly_information_model  


**Voilà! You can now go to VS Code, Rhino or Grasshopper to run the example files!**