from fabrication_manager.task import Task
from compas.geometry import Frame, Translation, Vector
from compas_fab.robots import Configuration, Robot
import fabtory_fabrication_control as ffc
import compas_rrc as rrc

__all__ = [
    "StartConfigurationTask"
    "SetToolTask"
    "PickBrickTask"
    "SafePositionTask"
    "PlaceBrickTask"
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
    def __init__(self, robot, cart, configuration, pick_frame, key=None):
        super(PickBrickTask, self).__init__(key)
        self.robot = robot
        self.cart = cart
        self.configuration = configuration
        self.pick_frame = pick_frame

    def run(self, stop_thread):
        # Move to safety_position
        self.configuration = Configuration((4.200, -1.120, 0.543, 0.491, 0.513, -1.093, -1.517), (2, 0, 0, 0, 0, 0, 0), ('axis_joint', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'))
        ffc.commands.move_to_joints(self.robot, self.configuration)
        print ('robot moved to safety position')

        # Open gripper
        ffc.commands.open(self.robot)
        print ('gripper is open')

        # Move to pick_up_frame
        ffc.commands.move_to_robtarget(self.robot, self.pick_frame, self.cart)
        print ('robot moved to pick_up_frame')

        # Close gripper
        ffc.commands.close(self.robot)
        print ('gripper closed')

        # Move back to safety_position
        self.configuration = Configuration((4.200, -1.120, 0.543, 0.491, 0.513, -1.093, -1.517), (2, 0, 0, 0, 0, 0, 0), ('axis_joint', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'))
        ffc.commands.move_to_joints(self.robot, self.configuration)
        print ('robot moved to safety position')

        self.is_completed = True
        return True

class SafePositionTask(Task):
    def __init__(self, robot, cart, configuration, key=None):
        super(SafePositionTask, self).__init__(key)
        self.robot = robot
        self.cart = cart
        self.configuration = configuration
    def run(self, stop_thread):
        self.configuration = Configuration((4.200, -1.406, -0.041, -0.146, -0.727, 0.249, -0.859), (2, 0, 0, 0, 0, 0, 0), ('axis_joint', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'))
        ffc.commands.move_to_joints(self.robot, self.configuration)
        print ('robot moved to safety position')
        self.is_completed = True
        return True
    
class PlaceBrickTask(Task):
    def __init__(self, robot, cart, configuration, place_frames, key=None):
        super(PlaceBrickTask, self).__init__(key)
        self.robot = robot
        self.cart = cart
        self.configuration = configuration
        self.place_frames = place_frames

    def run(self, stop_thread):
        for place_frame in self.place_frames:

            #Move to safety_position
            T = Translation.from_vector(Vector(0,0,0.2)) 
            safety_frame = self.place_frame.transformed(T)
            ffc.commands.move_to_robtarget(self.robot, safety_frame, self.cart)
            print ('robot moved to safety_frame')

            #Move to place_frame
            ffc.commands.move_to_robtarget(self.robot, place_frame, self.cart)
            print ('robot moved to place_frame')

            # Open gripper
            ffc.commands.open(self.robot)
            print ('gripper is open')

            #Move back to safety_position
            T = Translation.from_vector(Vector(0,0,0.2)) 
            safety_frame = self.place_frame.transformed(T)
            ffc.commands.move_to_robtarget(self.robot, safety_frame, self.cart)
            print ('robot moved to safety_frame')

        self.is_completed = True
        return True
    
if __name__ == "__main__":
    pass