from fabrication_manager.task import Task
from compas.geometry import Frame, Translation, Vector
from compas_fab.robots import Configuration, Robot
#import fabtory_fabrication_control as ffc
import compas_rrc as rrc
import abb_fabrication_control as ffc

__all__ = [
    "StartConfigurationTask"
    "SetToolTask"
    "PickBrickTask"
    "SafePositionTask"
    "PlaceBrickTask"
]

# class ABBTask(Task):
#     def run(self, stop_thread, last_command=None):
#         while not stop_thread():
#             if last_command.done or last_command is None:
#                 self.is_completed = True
#                 return True

class StartConfigurationTask(Task):
    def __init__(self, robot, configuration, key=None):
        super(StartConfigurationTask, self).__init__(key)
        self.robot = robot
        self.configuration = configuration
        self.sent = False
    def run(self, stop_thread):
        ffc.commands.move_to_joints(self.robot, self.configuration)
        self.log("robot moved to start configuration")
        self.is_completed = True
        return True
        # running the abbtask run statement, with argument LastCommand to wait for the robot to finish the last command
        #return super(StartConfigurationTask, self).run(stop_thread, lc)       

class SetToolTask(Task):
    def __init__(self, robot, tool_name, key=None):
        super(SetToolTask, self).__init__(key)
        self.robot = robot
        self.tool_name = tool_name
    def run(self, stop_thread):
        if self.robot.attached_tool:
            self.tool_name = self.robot.attached_tool.name
            self.robot.abb_client.send(rrc.SetTool(self.tool_name))
            self.log(self.tool_name + "set")
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
        #Move to side of brick
        self.configuration = Configuration((4.107, -1.307, 1.022, -0.252, 0.371, -0.805, -1.409), (2, 0, 0, 0, 0, 0, 0), ('axis_joint', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'))
        ffc.commands.move_to_joints(self.robot, self.configuration)
        print ('robot moved to safety position')

        #Open gripper
        ffc.commands.open(self.robot)
        self.log('gripper is open')

        # Move to safety distance above pick_up_frame
        self.configuration = Configuration((4.200, -1.249, 1.111, -0.333, 0.444, -0.829, -1.457), (2, 0, 0, 0, 0, 0, 0), ('axis_joint', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'))
        ffc.commands.move_to_joints(self.robot, self.configuration)
        print ('robot moved to safety position')

        # Move to pick_up_frame
        ffc.commands.move_to_robtarget(self.robot, self.pick_frame, self.cart)
        self.log('robot moved to pick_up_frame')
        
        # Close gripper
        ffc.commands.close(self.robot)
        self.log('gripper closed')

        # Move to safety distance above pick_up_frame
        self.configuration = Configuration((4.200, -1.249, 1.111, -0.333, 0.444, -0.829, -1.457), (2, 0, 0, 0, 0, 0, 0), ('axis_joint', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'))
        ffc.commands.move_to_joints(self.robot, self.configuration)
        self.log('robot moved to safety position')

        # Move to side of brick
        self.configuration = Configuration((4.107, -1.307, 1.022, -0.252, 0.371, -0.805, -1.409), (2, 0, 0, 0, 0, 0, 0), ('axis_joint', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'))
        ffc.commands.move_to_joints(self.robot, self.configuration)
        self.log('robot moved to safety position')

        self.is_completed = True
        return True
        #return super(PickBrickTask, self).run(stop_thread, lc)

class SafePositionTask(Task):
    def __init__(self, robot, cart, configuration, key=None):
        super(SafePositionTask, self).__init__(key)
        self.robot = robot
        self.cart = cart
        self.configuration = configuration
    def run(self, stop_thread):
        self.configuration = Configuration((4.200, -1.406, -0.041, -0.146, -0.727, 0.249, -0.859), (2, 0, 0, 0, 0, 0, 0), ('axis_joint', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'))
        ffc.commands.move_to_joints(self.robot, self.configuration)
        self.log('robot moved to safety position')
        self.is_completed = True
        return True
    
class PlaceBrickTask(Task):
    def __init__(self, robot, cart, configuration, place_frame, key=None):
        super(PlaceBrickTask, self).__init__(key)
        self.robot = robot
        self.cart = cart
        self.place_frame = place_frame
        self.configuration = configuration

    def run(self, stop_thread):
        #Move to safety_position
        T = Translation.from_vector(Vector(0,0,0.2)) 
        safety_frame = self.place_frame.transformed(T)
        ffc.commands.move_to_robtarget(self.robot, safety_frame, self.cart)
        self.log('robot moved to safety_frame')

        #Move to place_frame
        ffc.commands.move_to_robtarget(self.robot, self.place_frame, self.cart)
        self.log('robot moved to place_frame')

        # Open gripper
        ffc.commands.open(self.robot)
        self.log('gripper is open')

        #Move back to safety_position
        ffc.commands.move_to_robtarget(self.robot, safety_frame, self.cart)
        self.log('robot moved to safety_frame')

        self.configuration = Configuration((3.790, -1.251, 0.028, 0.546, 0.547, -0.648, -2.023), (2, 0, 0, 0, 0, 0, 0), ('axis_joint', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'))
        ffc.commands.move_to_joints(self.robot, self.configuration)
        self.log('robot locked position')


        self.is_completed = True
        return True
    
    
if __name__ == "__main__":
    pass