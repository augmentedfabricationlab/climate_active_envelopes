---
layout: page
title: Getting Started
---

**Quick links:** [COMPAS](https://compas.dev/compas/latest/index.html), [Ladybug Tools](https://www.ladybug.tools/)

## Requirements
* Operating System: **Windows 10 Pro** <sup>(1)</sup>.
* [Rhinoceros 3D 8.0](https://www.rhino3d.com/)
* [Anaconda Python Distribution](https://www.anaconda.com/download/): 3.x
* [Visual Studio Code](https://code.visualstudio.com/)
* [GitHub Desktop](https://desktop.github.com/)
* [Ladybug Tools](https://www.food4rhino.com/en/app/ladybug-tools)
* [Radiance](https://github.com/LBNL-ETA/Radiance/releases/tag/27dbb0e0)
* [OpenStudio 3.7.0](https://github.com/NREL/OpenStudio/releases/tag/v3.7.0) **Mac only**

## Dependencies
* [compas 2.6.1](https://compas.dev/index.html)
* [Shapely](https://pypi.org/project/shapely/) 

### 1. Getting Started

After installing Rhino 8, open Rhino and run **ScriptEditor** on the Rhino command line to **initialize Python**.

Execute the commands below in Anaconda Prompt:

#### Install dependencies with Anaconda Prompt on Windows

    (base) conda create -n rfa -c conda-forge compas_fab
    (base) conda activate rfa
    
#### Verify Installation

    (rfa) python -m compas
            Yay! COMPAS is installed correctly!

#### Install dependencies on Rhino 8 Python 3 with Rhinocode

It is good practice to ensure that you are using the latest version of pip. To update pip, run the following command:

    (rfa) python -m pip install --upgrade pip

Then run the installation in the rhinocode folder

    (rfa) cd .rhinocode\py39-rh8  
    (rfa) python.exe -m pip install compas==2.6.1 compas_robots==0.6.0 roslibpy pyserial
    (rfa) python.exe -m pip install --no-deps compas_fab
    (rfa) python.exe -m pip install shapely 

#### Install dependencies with Rhino PowerShell on Mac

Open **ScriptEditor** and navigate to **Tools** > **Advanced** > **Open Python3 Shell**

Execute the commands below:

    (base) conda create -n rfa -c conda-forge compas_fab
    (base) conda activate rfa
    (rfa) python -m pip install --upgrade pip

Then run the installation in the rhinocode folder:
    
    (rfa) cd .rhinocode/py39-rh8/ 
    (rfa) python -m pip install compas==2.6.1 compas_robots==0.6.0 roslibpy pyserial 
    (rfa) python -m pip install --no-deps compas_fab
    (rfa) python -m pip install shapely

#### Install Ladybug Tools to run the climatic simulations
* Download and Install [Ladybug Tools](https://www.food4rhino.com/en/app/ladybug-tools) and [Radiance](https://github.com/LBNL-ETA/Radiance/releases/tag/27dbb0e0) to get access to Ladybug Tools 1.8.0 and its simulations
* Find a [EPW Map](https://www.ladybug.tools/epwmap/) or use the epw file from munich provided in the rhino folder. You need to load that file in grasshopper to run the simulations.

**Note**: If you work with Mac please install **OpenStudio** first. This needs to work together with your LadyBug version. LadyBug 1.8.0. works together with [OpenStudio 3.7.0](https://github.com/NREL/OpenStudio/releases/tag/v3.7.0)

* Run the install grasshopper file for LadyBug and restart Rhino / Grasshopper
       
### 2. Cloning and installing the repository

#### Repository Cloning

* Create a workspace directory: C:\Users\YOUR_USERNAME\workspace
  
Open Github Desktop and clone the following repositories into you workspace folder:

* [this repository](https://github.com/augmentedfabricationlab/robotic_bricklaying) and
* [assembly_information_model](https://github.com/augmentedfabricationlab/assembly_information_model)
  
**Note**: Ensure that your branch on Github Desktop is set to **compas2**!

### 3. Install the repositories in editable mode

Open Rhino, Grasshopper and a Python3 block. Go to Tools/Options and below add the following paths:

* C:\Users\your_user_name\workspace\assembly_information_model\src
* C:\Users\your_user_name\workspace\robotic_bricklaying\src


**Voilà! You can now go to VS Code, Rhino or Grasshopper to run the example file in your rhino folder!**
