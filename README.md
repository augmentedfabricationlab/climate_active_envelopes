# Climate Active Envelopes
## Full installation 

**Quick links:** [COMPAS](https://compas.dev/compas/latest/index.html), [Ladybug Tools](https://www.ladybug.tools/)

## Requirements

* Operating System: **Windows 10 Pro** <sup>(1)</sup>.
* [Rhinoceros 3D 8.0](https://www.rhino3d.com/)
* [Anaconda Python Distribution](https://www.anaconda.com/download/): 3.x
* [Visual Studio Code](https://code.visualstudio.com/)
* [GitHub Desktop](https://desktop.github.com/)
* [Ladybug Tools](https://www.food4rhino.com/en/app/ladybug-tools)

## Dependencies
* [compas 2.2.0](https://compas.dev/index.html)
* [compas_fab 1.0.2 or higher](https://gramaziokohler.github.io/compas_fab/latest/)
* [Ladybug Tools](https://www.food4rhino.com/en/app/ladybug-tools)
* [Radiance](https://github.com/LBNL-ETA/Radiance/releases/tag/27dbb0e0)

## Getting Started

### 1. Setting up the Anaconda environment with all dependencies

Execute the commands below in Anaconda Prompt:

#### Install dependencies

    (base) conda create -n cae -c conda-forge compas_fab
    (base) conda activate cae
    
#### Verify Installation

    (cae) python -m compas
            Yay! COMPAS is installed correctly!

#### Install dependencies on Rhino 8 Python 3 with Rhinocode

    (cae) cd .rhinocode\py39-rh8
    (cae) python.exe -m pip install compas==2.2.0 compas_robots==0.4.0 roslibpy pyserial
    (cae) python.exe -m pip install --no-deps compas_fab

#### Install Ladybug Tools to run the climatic simulations
* Download and Install [Ladybug Tools](https://www.food4rhino.com/en/app/ladybug-tools) and [Radiance](https://github.com/LBNL-ETA/Radiance/releases/tag/27dbb0e0) to get access to the latest Ladybug Tools and its simulations
* Find the EPW maps here: [EPW Maps](https://www.ladybug.tools/epwmap/)
       
### 2. Cloning and installing the repository

#### Repository Cloning

* Create a workspace directory: C:\Users\YOUR_USERNAME\workspace
  
Open Github Desktop and clone

* [this repository](https://github.com/augmentedfabricationlab/climate_active_envelopes) into you workspace folder 
* [assembly_information_model](https://github.com/augmentedfabricationlab/assembly_information_model) into you workspace folder.
* [mobile_robot_control](https://github.com/augmentedfabricationlab/mobile_robot_control) into you workspace folder.
* [ur_fabrication_control](https://github.com/augmentedfabricationlab/ur_fabrication_control) into you workspace folder.
* [fabrication_manager](https://github.com/augmentedfabricationlab/fabrication_manager) into you workspace folder.

SWITCH TO compas2 BRANCH IN MOBILE_ROBOT_CONTROL AND UR_FABRICATION_CONTROL!!!!

### 3. Install the repositories in editable mode

Open Rhino, Grasshopper and a Python3 block. Go to Tools/Options and below add the following paths:

* C:\Users\your_user_name\workspace\assembly_information_model\src
* C:\Users\your_user_name\workspace\climate_active_envelopes\src
* C:\Users\your_user_name\workspace\mobile_robot_control\src
* C:\Users\your_user_name\workspace\ur_fabrication_control\src
* C:\Users\your_user_name\workspace\fabrication_manager\src

### 4. Run docker

Open up the Docker desktop.

Open up VScode, and install the docker extension.

On VScode, find the file: \mobile_robot_control\docker\ros-systems\rbvogui-xl-ur20-noetic\docker-compose.yml, right click and click "compose up". Wait for the images to be copied and started.


## Credits

This package was created by Julia Fleckenstein <julia.fleckenstein@tum.de> at `@augmentedfabricationlab` [augmentedfabricationlab](https://github.com/augmentedfabricationlab)
