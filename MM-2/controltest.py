from mobilegello.gello_controller import GELLOcontroller
import time
import numpy as np

# make_hdf5("lift_data")


mygello = GELLOcontroller("doodle", torque_start=True)
new_home = np.array([2068, 2010, 2094, 1885, 2124, 3150]) #rest position
config_1 = np.array([2338, 2447, 3393, 1740, 1994, 3710]) #pick up position - gripper open
config_2 = np.array([2338, 2447, 3393, 1740, 1994, 3100]) #pick up position - gripper closed



def rest(): 
    mygello.goto_controlled_home(new_home)


def pickup():
    mygello.goto_controlled_home(new_home)
    mygello.goto_controlled_home(config_1)
    mygello.goto_controlled_home(config_2)
    mygello.goto_controlled_home(new_home)

def deliver():
    mygello.goto_controlled_home(new_home)
    mygello.goto_controlled_home(config_2)
    mygello.goto_controlled_home(config_1)
    mygello.goto_controlled_home(new_home)

rest()
pickup()
deliver()
print(mygello.read_encoder_values())