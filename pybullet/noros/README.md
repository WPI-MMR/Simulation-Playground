# PyBullet (No ROS) Simulation
This simulation uses the solo12 robot in PyBullet as a test playground.


## Environment Setup
The recommended way is to use virtual environments just in order to keep 
everything separate. Make sure that you are in this folder from the command
line when running these instructions.

1. Create the virtual environment: `python3 -m venv venv-pybullet-noros`
2. Activate it: `source venv-pybullet-noros/bin/activate`
3. Upgrade your environment: `pip install -U wheel pip`
4. Change the current directory to pybullet/noros  
5. Install deps: `pip install -r requirements.txt`

And that's it! If you ever need to get out of the environment, simply enter
`deactivate`.


## Interactive Demo
To run the interactive demo, make sure to activate your environment, then 
enter `python interactive.py`.
