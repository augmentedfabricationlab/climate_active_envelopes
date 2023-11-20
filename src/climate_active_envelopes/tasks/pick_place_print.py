from fabrication_manager.task import Task
from compas.geometry import Frame, Translation, Vector
from compas_fab.robots import Configuration, Robot
# from fabtory_fabrication_control.commands.motion import move_to_joints, move_to_robtarget
# from fabtory_fabrication_control.commands.communication import open, close

import fabtory_fabrication_control as ffc
import compas_rrc as rrc


__all__ = [
    "StartConfigurationTask"
    "SetToolTask"
    "PickBrickTask"
]

class StartConfigurationTask(Task):
    def __init__(self, robot, configuration, key=None):
        super(StartConfigurationTask, self).__init__(key)
        self.robot = robot
        self.configuration = configuration
        
    def run(self, thread_stop):
        ffc.commands.move_to_joints(self.robot, self.configuration)
        print("robot moved to start configuration")
        self.is_completed = True
        return True

class SetToolTask(Task):
    def __init__(self, robot, tool_name, key=None):
        super(SetToolTask, self).__init__(key)
        self.robot = robot
        self.tool_name = tool_name

    def run(self, stop_thread):
        if self.robot.attached_tool:
            self.tool_name = self.robot.attached_tool.name
            self.robot.abb_client.send(rrc.SetTool(self.tool_name))
            print(self.tool_name + "set")
            self.is_completed = True
            return True
    
class PickBrickTask(Task):
    def __init__(self, robot, cart, pick_frame, key=None):
        super(PickBrickTask, self).__init__(key)
        self.robot = robot
        self.cart = cart
        self.pick_frame = pick_frame

    def run(self, stop_thread):
        # Move to safety_frame
        T = Translation.from_vector(Vector(0,0,0.2)) 
        safety_frame = self.pick_frame.transformed(T)
        ffc.commands.move_to_robtarget(self.robot, safety_frame, self.cart)
        print ('robot moved to safety frame')

        # Open gripper
        ffc.commands.open(self.robot)
        print ('gripper is open')

        # Move to pick_up_frame
        ffc.commands.move_to_robtarget(self.robot, self.pick_frame, self.cart)
        print ('robot moved to pick_up_frame')

        # Close gripper
        ffc.commands.close(self.robot)
        print ('gripper closed')

        # Move back to safety_frame
        ffc.commands.move_to_robtarget(self.robot, safety_frame, self.cart)
        print ('robot moved back to safety frame')

        self.is_completed = True
        return True
    

if __name__ == "__main__":
    pass







        # for frame in self.pick_frame[i]:

        #     ffc.commands.move_to_robtarget(self.robot, frame, self.cart)
        #     print ('moved to pick up frame')





# class PlaceBrickTask(Task):
#     def __init__(self, robot, cart, place_frames, key=None):
#         super(PlaceBrickTask, self).__init__(key)
#         pass

# class PrintMortarTask(Task):
#     def __init__(self, robot, cart, extrude_frames, key=None):
#         pass


"""
class ExtrudeSegmentTask(Task):
    def __init__(self, robot, cart, extrude_frames, key=None):
        self.robot = robot
        self.cart = cart
        self.extrude_frames = extrude_frames
        
    def run(self, stop_thread):
        T = Translation.from_vector(Vector(0,0,0.2)) 

        # Moves to approach frame
        approach_frame = self.extrude_frames[0].transformed(T)
        move_to_robtarget(self.robot, approach_frame, self.cart)
        print ('robot moved to approach frame')

        # Turns valve on
        open(self.robot)
        print ('valve is open')

        # Turns pump on
        
        print ('pump is on')

        # Moves to extrusion frames
        for frame in self.extrude_frames:
            move_to_robtarget(self.robot, frame, self.cart)
            print ('moved to extrusion frame')
        
        # Turns valve off
        close(self.robot)
        print ('valve is closed')

        # Turns pump off
        
        print ('pump is off')

        # Moves to retract frame
        retract_frame = self.extrude_frames[-1].transformed(T)
        move_to_robtarget(self.robot, retract_frame, self.cart)

        self.is_completed = True
        return True

class StampLayerTask(Task):
    def __init__(self, robot, cart, stamp_frames, key=None):
        super(StampLayerTask, self).__init__(key)
        self.robot = robot
        self.cart = cart
        self.stamp_frames = stamp_frames
        
    def run(self, stop_thread):

        # Moves to stamping frames
        for frame in self.stamp_frames:
            move_to_robtarget(self.robot, frame, self.cart)
            print ('moved to stamp frame')

        self.is_completed = True 
        return True
"""
