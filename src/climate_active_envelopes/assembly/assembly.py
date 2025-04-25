from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.datastructures import Datastructure
from compas.datastructures import Graph
from compas.datastructures import AssemblyError
from compas.geometry import Frame, Translation, Rotation, Transformation, Vector, Point, normalize_vector
from compas_rhino.conversions import plane_to_compas_frame, point_to_compas, mesh_to_rhino, point_to_rhino, vector_to_rhino


from scipy.spatial import cKDTree
from compas.geometry import Polygon
from shapely.geometry import Polygon as ShapelyPolygon

import math as m
import numpy as np


from compas.geometry import Frame
from compas.geometry import local_to_world_coordinates_numpy


from collections import deque

from assembly_information_model import Assembly
from .part import CAEPart as Part
from scipy.spatial import cKDTree
#from .reference_model import CAECellNetwork as CellNetwork


import math
import Rhino.Geometry as rg



class CAEAssembly(Assembly):
    """A data structure for managing the connections between different parts of an assembly.

    Parameters
    ----------
    name : str, optional
        The name of the assembly.
    **kwargs : dict, optional
        Additional keyword arguments, which are stored in the attributes dict.

    Attributes
    ----------
    graph : :class:`compas.datastructures.Graph`
        The graph that is used under the hood to store the parts and their connections.

    See Also
    --------
    :class:`compas.datastructures.Graph`
    :class:`compas.datastructures.Mesh`
    :class:`compas.datastructures.VolMesh`

    """

    def __init__(self, name=None, brick_full=None, brick_insulated=None, brick_half=None, brick_air_dried=None, **kwargs):
        super(CAEAssembly, self).__init__()
        self.brick_params = None
        if brick_full and brick_insulated and brick_half and brick_air_dried:
            self.set_brick_params(brick_full, brick_insulated, brick_half, brick_air_dried)


    def export_to_json(self, path, is_built=False):
        # TODO!
        self.graph.update_default_node_attributes({"is_built":False})
        for key in self.parts():
            self.graph.node_attribute(key, "is_built", is_built)
        self.to_json(path)


    def set_brick_params(self, brick_full, brick_insulated, brick_half, brick_air_dried):

        self.brick_params = {
            "brick_full": brick_full,
            "brick_insulated": brick_insulated,
            "brick_half": brick_half,
            "brick_air_dried": brick_air_dried,
           
        }

        return self.brick_params

    def get_brick_dimensions(self):

        """
        Get the dimensions of the bricks
        
        Parameters
        ----------
        brick_full : :class:`CAEPart`
            The full brick to use for the wall
        brick_insulated : :class:`CAEPart`
            The insulated brick to use for the wall
        """

        if self.brick_params is None:
            raise ValueError("brick_params is not set. Please set brick_params using set_brick_params method before calling get_brick_dimensions.")

        brick_full = self.brick_params["brick_full"]
        brick_half = self.brick_params["brick_half"]

        brick_length = brick_full.shape.ysize
        brick_height = brick_full.shape.zsize
        brick_width = brick_full.shape.xsize
        brick_length_h = brick_half.shape.ysize

        return brick_length, brick_height, brick_width, brick_length_h
    
    def compute_brick_layout(self, cell_network, mesh, course_height, brick_spacing, input_type):

        # get the dimensions of the bricks
        brick_length, _, brick_width, _, = self.get_brick_dimensions()

        # get the assembly data from the cell network
        #assembly_data = cell_network.generate_assembly_data_from_cellnetwork(cell_network, course_height)
        assembly_data = cell_network.generate_assembly_data(mesh, course_height, input_type)


        direction_vector = assembly_data['direction_vector']
        edge_length = assembly_data['edge_length']
        start_edge_type = assembly_data['start_edge_type']
        end_edge_type = assembly_data['end_edge_type']
        num_courses = assembly_data['num_courses']
        curve_start_point = assembly_data['curve_start_point']
        curve_end_point = assembly_data['curve_end_point']


        course_brick_data = []
        for course in range(math.ceil(num_courses + 1)):    

            # Check if the course is odd         
            course_is_odd = course % 2 != 0

            # Calculate the number of bricks per course
            bricks_per_course = math.floor(edge_length / ((brick_width + brick_length) / 2 + brick_spacing))

            # Adjust the z-coordinate of the curves_start_point for each course
            adjusted_start_point = rg.Point3d(curve_start_point[0], curve_start_point[1], curve_start_point[2] + course * course_height)
            adjusted_end_point = rg.Point3d(curve_end_point[0], curve_end_point[1], curve_end_point[2] + course * course_height)

            # Calculate the midpoint of the contour curve
            curve_midpoint = point_to_rhino((adjusted_start_point + adjusted_end_point) / 2)
            
            # Number of bricks per course are always odd
            if bricks_per_course % 2 == 0:
                bricks_per_course -= 1

            #If course is odd and number of bricks per course is even, subtract 1
            if course_is_odd and bricks_per_course % 2 != 0:
                bricks_per_course -= 1

            course_brick_data.append((bricks_per_course, course_is_odd, direction_vector, start_edge_type,
                                    end_edge_type, curve_midpoint, adjusted_start_point, adjusted_end_point))

        return course_brick_data

    def generate_wall(self, 
                      cell_network,
                      bond_type, 
                      ornament,
                      wall_system,  
                      brick_spacing, 
                      course_height, 
                      input_type,
                      mesh
                      ):

        course_brick_data = self.compute_brick_layout(cell_network, mesh, course_height, brick_spacing, input_type)
        
        for j, data in enumerate(course_brick_data):
            bricks_per_course, course_is_odd, direction_vector, start_edge_type, end_edge_type, curve_midpoint, adjusted_start_point, adjusted_end_point = data

            if bond_type == "flemish_bond":
                # Calculate the total length of the course
                total_length = self.calculate_flemish_course_length(
                    bricks_per_course=bricks_per_course,
                    brick_spacing=brick_spacing,
                    course_is_odd=course_is_odd)                              

                # Adjust the initial brick position based on corner detection
                # if start_edge_type == "corner":
                #     initial_brick_position = adjusted_start_point
                # if end_edge_type == "corner":
                #     initial_brick_position = adjusted_end_point - (direction_vector * (total_length))
                # else:
                initial_brick_position = curve_midpoint - (direction_vector * (total_length / 2))


                self.generate_flemish_bond(
                    initial_brick_position=initial_brick_position,
                    bricks_per_course=bricks_per_course,
                    course_is_odd=course_is_odd,
                    direction_vector=direction_vector,
                    wall_system=wall_system,
                    brick_spacing=brick_spacing,
                    start_edge_type=start_edge_type,
                    end_edge_type=end_edge_type,
                    )
                
            if bond_type =="vertical_bond":

                total_length = self.calculate_flemish_course_length(
                    bricks_per_course=bricks_per_course,
                    brick_spacing=brick_spacing,
                    course_is_odd=course_is_odd)                              

                initial_brick_position = curve_midpoint - (direction_vector * (total_length / 2))
                self.generate_vertical_bond(
                        initial_brick_position=initial_brick_position,
                        line_length=total_length,          
                        course_is_odd=course_is_odd,
                        direction_vector=direction_vector,
                        wall_system=wall_system,
                        brick_spacing=brick_spacing,
                        start_edge_type=start_edge_type,
                        end_edge_type=end_edge_type,
                        j=j,
                        ornament = ornament
                        )

    def create_brick_and_add_to_assembly(self,
                        brick_type, 
                        transform_type, 
                        frame=None,
                        ): 
        """Create a brick with a specified type and add it to the assembly

        Parameters
        ----------
        brick_type : str
            The type of brick to create ("full" or "insulated").
        brick_full : :class:`CAEPart`
            The full brick to use for the wall.
        brick_insulated : :class:`CAEPart`
            The insulated brick to use for the wall.
        transform_type : str
            Type of transformation to apply ("translate" or "rotate").
        frame : :class:`compas.geometry.Frame`, optional
            The frame of the brick.
        
        Returns
        -------
        Part
            The brick part that is added to the assembly. 
        """

        brick_full = self.brick_params["brick_full"]
        brick_insulated = self.brick_params["brick_insulated"]
        brick_half = self.brick_params["brick_half"]
        brick_air_dried = self.brick_params["brick_air_dried"]

        if frame is None:
            frame = frame

        if brick_type == "full":
            brick = brick_full

        if brick_type == "insulated":
            brick = brick_insulated

        if brick_type == "half":
            brick = brick_half

        if brick_type == "air_dried":
            brick = brick_air_dried
            
        my_brick = brick.transformed(Transformation.from_frame(frame))

        # Adjust the gripping frame by translating it along the z-axis to account for the brick height
        gripping_frame = frame.transformed(Translation.from_vector(frame.zaxis*((brick_full.shape.zsize - 0.020)/2)))

        # Rotate the gripping frame by 180 degrees around the x-axis
        R = Rotation.from_axis_and_angle(gripping_frame.xaxis, math.radians(180), gripping_frame.point)
        gripping_frame.transform(R)
        # Set the gripping frame of the brick 
        my_brick.gripping_frame = gripping_frame
        my_brick.frame = frame
        my_brick.original_frame = my_brick.frame.copy()

        self.add_part(my_brick, attr_dict={"brick_type": brick_type, "transform_type": transform_type})

    def generate_french_bond(self,
                    brick_full,
                    brick_insulated,
                    initial_brick_position,
                    line_length,          
                    plane,
                    course_is_odd,
                    j,
                    wall_system):

        brick_spacing = 0.015
        mortar_joint_height = 0.015
        brick_width_i, brick_length_i, brick_length, brick_height, brick_width = self.get_brick_dimensions(brick_full, brick_insulated)
      
        brick_params = {"brick_full": brick_full, 
                  "brick_insulated": brick_insulated 
                }

        
        center_brick_frame = plane_to_compas_frame(plane)
        num_bricks1 = math.floor(line_length / (((brick_width+2*brick_length) + 3*brick_spacing)))

        for i in range(num_bricks1):
            T = plane.XAxis * -(i*(2*(brick_spacing+brick_length)))
            translation = Translation.from_vector(T)
            
            # Apply translation to the initial brick center
            brick_center = initial_brick_position + T
            brick_frame = Frame(point_to_compas(brick_center), center_brick_frame.xaxis, center_brick_frame.yaxis)
            
            # Transform the frame with translation
            current_frame = brick_frame.transformed(translation)
            # Add the brick to the assembly
            if course_is_odd:
                if wall_system == "single_layer" or wall_system == "double_layer":
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **brick_params)
            else:
                T = plane.XAxis * -((((brick_length+brick_spacing+brick_width)/2)))
                translation = Translation.from_vector(T)
                current_frame = current_frame.transformed(translation)
                if wall_system == "single_layer" or wall_system == "double_layer":
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **brick_params)

            
            T = plane.XAxis * -((((brick_length+brick_spacing)*1.5)))
            T2 = plane.XAxis * -(i*(((2*(brick_length+brick_spacing)))))
            T1 = plane.YAxis * ((brick_width-brick_length)/2)
            
            translation = Translation.from_vector(T)
            Translation2 = Translation.from_vector(T1)
            Translation3 = Translation.from_vector(T2)
            
            # Create a rotation transformation (90 degrees around Z-axis)
            R = Rotation.from_axis_and_angle(current_frame.zaxis, math.radians(90), brick_frame.point)
            
            # Apply rotation
            rotated_frame = brick_frame.transformed(R)
            
            # Apply translation to the rotated frame
            current_frame = rotated_frame.transformed(translation*Translation2*Translation3)
            
            # Add the rotated brick to the assembly
            if course_is_odd:
                if wall_system == "single_layer" or wall_system == "double_layer":
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **brick_params)
                    T = plane.XAxis * -((((brick_length+brick_spacing))))
                    translation = Translation.from_vector(T)
                    current_frame = current_frame.transformed(translation)
                    if wall_system == "single_layer" or wall_system == "double_layer":
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **brick_params)

                if wall_system == "double_layer":
                    T = plane.YAxis * (brick_spacing+brick_width)
                    translation = Translation.from_vector(T)
                    current_frame = current_frame.transformed(translation)
                    self.create_brick_and_add_to_assembly(brick_type="insulated", transform_type = "fixed", frame=current_frame, **brick_params)

            else:
                T = plane.XAxis * -((((brick_length+brick_spacing+brick_width)/2)))
                translation = Translation.from_vector(T)
                current_frame = current_frame.transformed(translation)
                if wall_system == "single_layer" or wall_system == "double_layer":
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **brick_params)
                    T = plane.XAxis * -((((brick_length+brick_spacing))))
                    translation = Translation.from_vector(T)
                    current_frame = current_frame.transformed(translation)
                    if wall_system == "single_layer" or wall_system == "double_layer":
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **brick_params)
                        
                if wall_system == "double_layer":
                    T = plane.YAxis * (brick_spacing+brick_width)
                    translation = Translation.from_vector(T)
                    current_frame = current_frame.transformed(translation)
                    self.create_brick_and_add_to_assembly(brick_type="insulated", transform_type = "fixed", frame=current_frame, **brick_params)

    def generate_vertical_bond(self,
                                initial_brick_position,
                                line_length,
                                course_is_odd,
                                direction_vector,
                                wall_system,
                                brick_spacing,
                                start_edge_type,
                                end_edge_type,
                                j,
                                ornament):



        brick_length, _, brick_width, _ = self.get_brick_dimensions()
        brick_full = self.brick_params["brick_full"]
        center_brick_frame = brick_full.frame
        num_bricks1 = math.ceil(line_length / (brick_width + brick_spacing))

        ornament = ornament  # "cross" or "straight", "diamond"
        

        if start_edge_type == "corner":
            self.generate_corner_vertical_bond(
                initial_brick_position=initial_brick_position,
                course_is_odd=course_is_odd,
                direction_vector=direction_vector,
                brick_spacing=brick_spacing,
                start_edge_type=start_edge_type,
                end_edge_type=end_edge_type                        
            )

        elif end_edge_type == "corner":
            self.generate_corner_vertical_bond(
                initial_brick_position=initial_brick_position,
                course_is_odd=course_is_odd,
                direction_vector=direction_vector,
                brick_spacing=brick_spacing,
                start_edge_type=start_edge_type,
                end_edge_type=end_edge_type,
        )
        if not course_is_odd:


            # Bricks laid short side out (rotated 90 degrees)
            num_bricks = math.ceil(line_length / (brick_length + brick_spacing))
            num_bricks1 = math.ceil(line_length / (brick_width + brick_spacing))

            # Shift the starting point to align to the middle of the long facing brick
            adjusted_initial_position = initial_brick_position + direction_vector * ((brick_width - brick_length) / 2)

            for brick in range(num_bricks):
                T = direction_vector * (brick * (brick_length + (brick_spacing/2)+ ((brick_width - (2*brick_length))/2)))
                brick_position = adjusted_initial_position + T

                # Create base brick frame
                if direction_vector[1] in [-1, 1]:
                    brick_frame = Frame(brick_position, direction_vector, center_brick_frame.xaxis)
                else:
                    brick_frame = Frame(brick_position, direction_vector, center_brick_frame.yaxis)

                # Rotate 90 degrees
                R = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), brick_frame.point)
                rotated_frame = brick_frame.transformed(R)

                # Translate to align correctly
                T1 = Translation.from_vector(rotated_frame.yaxis * ((brick_width - brick_length) / 2))
                brick_frame_final = rotated_frame.transformed(T1)

                # Ornament logic for even courses
                if ornament == "cross":
                    if brick % 2 == 0 and j % 4 == 0:
                        transform_type = "translate"
                    elif brick % 2 != 0 and j % 4 != 0:
                        transform_type = "translate"
                    else:
                        transform_type = "fixed"
                elif ornament == "straight":
                    transform_type = "translate" if brick % 2 == 0 else "fixed"
                elif ornament == "diamond":
                    transform_type = "translate" if brick % 2 == 0 else "fixed"
                else:
                    transform_type = "fixed"

                # Add the brick
                if brick in range (0,3) and start_edge_type == "corner" :
                    pass
                else:
                    self.create_brick_and_add_to_assembly("full", transform_type, brick_frame_final)
            
            num_bricks1 = math.ceil(line_length / (brick_width + brick_spacing))

            for brick in range(num_bricks1):
                T = direction_vector * (brick * (brick_width + brick_spacing))
                brick_position = initial_brick_position + T

                # Create base brick frame
                if direction_vector[1] in [-1, 1]:
                    brick_frame = Frame(brick_position, direction_vector, center_brick_frame.xaxis)
                else:
                    brick_frame = Frame(brick_position, direction_vector, center_brick_frame.yaxis)


                # Double-layer wall? Add insulated brick shifted along y-axis
                if wall_system == "double_layer":                
                    T2 = Translation.from_vector(brick_frame.yaxis * ( ((brick_width/2)+brick_width/4) + brick_spacing - ((brick_width-(2*(brick_length)))/4)))
                    insulated_frame = brick_frame.transformed(T2)
                    if brick in range (0,2) and start_edge_type == "corner":
                        pass
                    else:
                        self.create_brick_and_add_to_assembly("insulated", "fixed", insulated_frame)

                    T3 = Translation.from_vector(insulated_frame.yaxis * (brick_length+(brick_width-(2*(brick_length)))))
                    insulated_frame = insulated_frame .transformed(T3)
                    if brick in range (0,2) and start_edge_type == "corner":
                        pass
                    else:
                        self.create_brick_and_add_to_assembly("insulated", "fixed", insulated_frame)
        # -------------------------
        # ODD COURSE: TWO LOOPS
        # -------------------------
        if course_is_odd:
            # LOOP 1: Bricks laid long side out (normal orientation)
            num_bricks1 = math.ceil(line_length / (brick_width + brick_spacing))

            for brick in range(num_bricks1):
                T = direction_vector * (brick * (brick_width + brick_spacing))
                brick_position = initial_brick_position + T

                # Create brick frame
                if direction_vector[1] in [-1, 1]:
                    brick_frame = Frame(brick_position, direction_vector, center_brick_frame.xaxis)
                else:
                    brick_frame = Frame(brick_position, direction_vector, center_brick_frame.yaxis)

                # Ornament logic
                if ornament == "cross":
                    transform_type = "fixed"
                elif ornament == "straight":
                    transform_type = "fixed"
                elif ornament == "diamond":
                    transform_type = "translate" if brick % 2 == 0 else "fixed"
                else:
                    transform_type = "fixed"

                T1 = Translation.from_vector(-1* brick_frame.yaxis * (( (brick_width - brick_length) / 2)))
                brick_frame= brick_frame.transformed(T1)

                # Add the brick
                if brick in range (0,2) and start_edge_type == "corner":
                    pass
                else:
                    self.create_brick_and_add_to_assembly("full", transform_type, brick_frame)

                # Single-layer (full)
                if wall_system == "single_layer":
                    T1 = Translation.from_vector((brick_frame.yaxis * (brick_length+ (brick_width-(2*brick_length)))))
                    insulated_frame = brick_frame.transformed(T1)
                    if brick in range (0,2) and start_edge_type == "corner":
                        pass
                    else:
                        self.create_brick_and_add_to_assembly("full", "fixed", insulated_frame)

                # Double-layer (insulated)
                if wall_system == "double_layer":
                    T1 = Translation.from_vector((brick_frame.yaxis * (brick_length+ (brick_width-(2*brick_length)))))
                    insulated_frame = brick_frame.transformed(T1)
                    if brick in range (0,2) and start_edge_type == "corner":
                        pass

                    else:
                        self.create_brick_and_add_to_assembly("insulated", "fixed", insulated_frame)

            # LOOP 2: Bricks laid short side out, back (rotated 90 degrees)
            num_bricks2 = math.ceil(line_length / (brick_length + brick_spacing))
            adjusted_initial_position = initial_brick_position + direction_vector * ((brick_width - brick_length) / 2)

            for brick in range(num_bricks2):
                T = direction_vector * (brick * (brick_length + (brick_spacing/2)+ ((brick_width - (2*brick_length))/2)))
                brick_position = initial_brick_position + T

                # Create base brick frame
                if direction_vector[1] in [-1, 1]:
                    brick_frame = Frame(brick_position, direction_vector, center_brick_frame.xaxis)
                else:
                    brick_frame = Frame(brick_position, direction_vector, center_brick_frame.yaxis)

                # Rotate 90 degrees around Z
                R = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), brick_frame.point)
                rotated_frame = brick_frame.transformed(R)
                
                # Translate to align correctly
                T1 = Translation.from_vector(rotated_frame.xaxis * (((2*brick_length + brick_spacing)) + ((brick_width - (2*(brick_length))))))
                brick_frame_final = rotated_frame.transformed(T1)

                if brick in range(0,3) and start_edge_type == "corner":
                    pass
                else:
                    # Add insulated brick if double layer
                    if wall_system == "double_layer":
                        self.create_brick_and_add_to_assembly("insulated", "fixed", brick_frame_final)


    def generate_corner_vertical_bond(self, 
                                    initial_brick_position,
                                    course_is_odd,
                                    direction_vector,
                                    brick_spacing,
                                    start_edge_type,
                                    end_edge_type):
        """
        Generates a corner course for a vertical bond.
        
        For odd courses (bricks laid long side out):
        - The spacing step is: step_odd = brick_width + brick_spacing.
        - When the corner condition applies, two bricks are generated as corner bricks.
        
        For even courses (bricks laid with short side out, rotated 90°):
        - The spacing step is: step_even = brick_length + brick_spacing.
        - In a corner region the sequence is: [full, half, full, full] bricks.
        
        Note: In these formulas, brick_width is actually the brick’s long side,
            and brick_length is the brick’s short side.
        """
        # Retrieve brick dimensions.
        brick_length, _, brick_width, _ = self.get_brick_dimensions()  # brick_length: short side; brick_width: long side.
        brick_full = self.brick_params["brick_full"]
        center_brick_frame = brick_full.frame

        if course_is_odd:
            bricks_per_course = 2
            
            for brick in range(bricks_per_course):
                T = direction_vector * (brick * (brick_width + brick_spacing))
                brick_position = initial_brick_position + T

                # Create brick frame
                if direction_vector[1] in [-1, 1]:
                    brick_frame = Frame(brick_position, direction_vector, center_brick_frame.xaxis)
                else:
                    brick_frame = Frame(brick_position, direction_vector, center_brick_frame.yaxis)
                
                T1 = Translation.from_vector(-1* brick_frame.yaxis * (( (brick_width - brick_length) / 2)))
                brick_frame= brick_frame.transformed(T1)

                self.create_brick_and_add_to_assembly("full", "fixed", brick_frame)

                if brick == 1:
                    T2 = Translation.from_vector(brick_frame.yaxis * (( (brick_length + (brick_width - (2*brick_length))))))
                    brick_frame= brick_frame.transformed(T2)

                    self.create_brick_and_add_to_assembly("full", "fixed", brick_frame)




            bricks_per_course = 3
        
            for brick in range(bricks_per_course):
                T = direction_vector * (brick * (brick_width + brick_spacing))
                brick_position = initial_brick_position + T

                # Create brick frame
                if direction_vector[1] in [-1, 1]:
                    brick_frame = Frame(brick_position, direction_vector, center_brick_frame.xaxis)
                else:
                    brick_frame = Frame(brick_position, direction_vector, center_brick_frame.yaxis)

                
                
                T1 = Translation.from_vector(-1* brick_frame.yaxis * (( (brick_width - brick_length) / 2)))
                brick_frame= brick_frame.transformed(T1)
                if brick == 0:
                    T2 = Translation.from_vector((brick_frame.yaxis * ((((brick_length/3)*2)+(3*(brick_width - (2*brick_length)))) + 3*((brick_width - (2*brick_length)))/2)))
                    brick_frame = brick_frame.transformed(T2)

                    self.create_brick_and_add_to_assembly("half", "fixed", brick_frame) # adding the half brick for the corner
                elif brick == 1:

                    R = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), brick_frame.point)
                    rotated_frame = brick_frame.transformed(R)
                    T2 = Translation.from_vector(rotated_frame.xaxis * (((((brick_length))))))
                    brick_frame = rotated_frame.transformed(T2)
                    T3 = Translation.from_vector(brick_frame.yaxis*((brick_length/2)+ ((brick_width - (2*brick_length))/2)))
                    brick_frame = brick_frame.transformed(T3)

                    self.create_brick_and_add_to_assembly("full", "fixed", brick_frame) # adding the half brick for the corner

                    T3 = Translation.from_vector(brick_frame.yaxis * (-1)* (((3*brick_length)/4)+brick_spacing/2))
                    brick_frame = brick_frame.transformed(T3)

                    self.create_brick_and_add_to_assembly("half", "fixed", brick_frame) # adding the half brick for the corner

            # Double-layer (insulated)
        else:
            # Bricks laid short side out (rotated 90 degrees)
            bricks_per_course = 4
            # Shift the starting point to align to the middle of the long facing brick
            adjusted_initial_position = initial_brick_position 

            for brick in range(bricks_per_course):
                if brick == 0:
                        T = direction_vector * (brick * (brick_length + (brick_spacing/2)+ ((brick_width - (2*brick_length))/2)))
                        brick_position = adjusted_initial_position + T

                        # Create base brick frame
                        if direction_vector[1] in [-1, 1]:
                            brick_frame = Frame(brick_position, direction_vector, center_brick_frame.xaxis)
                        else:
                            brick_frame = Frame(brick_position, direction_vector, center_brick_frame.yaxis)

                        # Rotate 90 degrees
                        R = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), brick_frame.point)
                        rotated_frame = brick_frame.transformed(R)

                        # Translate to align correctly
                        T1 = Translation.from_vector(rotated_frame.yaxis * ((brick_width - brick_length) / 2))
                        brick_frame_final = rotated_frame.transformed(T1)
                        self.create_brick_and_add_to_assembly("full", "fixed", brick_frame_final)


                        T2 = Translation.from_vector(brick_frame_final.xaxis * ((brick_width + brick_spacing)))
                        brick_frame_final = brick_frame_final.transformed(T2)
                        self.create_brick_and_add_to_assembly("full", "fixed", brick_frame_final)

                        T3 = Translation.from_vector(brick_frame_final.yaxis * (-1)*((brick_length + brick_spacing)))
                        brick_frame_final = brick_frame_final.transformed(T3)
                        self.create_brick_and_add_to_assembly("full", "fixed", brick_frame_final)
                        
                        

                        T3 = Translation.from_vector(brick_frame_final.xaxis * ((brick_width - (brick_length/2) - ((brick_width - (2*(brick_length)))/2) + brick_spacing)))
                        T4 = Translation.from_vector(-1*(brick_frame_final.yaxis * ((2*(brick_length+ brick_spacing)))))
                        brick_frame_final = brick_frame.transformed(T3*T4)
                        self.create_brick_and_add_to_assembly("full", "fixed", brick_frame_final)

                        T5 = Translation.from_vector((brick_frame_final.yaxis * ((brick_length+ brick_spacing) - (brick_length/4))))
                        brick_frame_final = brick_frame_final.transformed(T5)
                        self.create_brick_and_add_to_assembly("half", "fixed", brick_frame_final)




                elif brick == 1:
                    T = direction_vector * (brick * (brick_length/2 + (brick_spacing/2)+ ((brick_width - (brick_length))/2)))
                    brick_position = adjusted_initial_position + T

                    # Create base brick frame
                    if direction_vector[1] in [-1, 1]:
                        brick_frame = Frame(brick_position, direction_vector, center_brick_frame.xaxis)
                    else:
                        brick_frame = Frame(brick_position, direction_vector, center_brick_frame.yaxis)

                    # Rotate 90 degrees
                    R = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), brick_frame.point)
                    rotated_frame = brick_frame.transformed(R)

                    # Translate to align correctly
                    T1 = Translation.from_vector(rotated_frame.yaxis * ((brick_width - brick_length/2) / 2))
                    brick_frame_final = rotated_frame.transformed(T1)
                    self.create_brick_and_add_to_assembly("half", "fixed", brick_frame_final)

                elif brick in range (2,4):
                    T = direction_vector * (brick * (brick_length/2 + (brick_spacing/2)+ ((brick_width - (brick_length))/2)))
                    brick_position = adjusted_initial_position + T

                    # Create base brick frame
                    if direction_vector[1] in [-1, 1]:
                        brick_frame = Frame(brick_position, direction_vector, center_brick_frame.xaxis)
                    else:
                        brick_frame = Frame(brick_position, direction_vector, center_brick_frame.yaxis)

                    # Rotate 90 degrees
                    R = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), brick_frame.point)
                    rotated_frame = brick_frame.transformed(R)

                    # Translate to align correctly
                    T1 = Translation.from_vector(rotated_frame.yaxis * ((brick_width - brick_length)))
                    brick_frame_final = rotated_frame.transformed(T1)
                    self.create_brick_and_add_to_assembly("full", "fixed", brick_frame_final)

    def generate_flemish_bond(self,
                                initial_brick_position,
                                bricks_per_course,
                                course_is_odd,
                                direction_vector,
                                wall_system,
                                brick_spacing, 
                                start_edge_type,
                                end_edge_type,                                                          
                                ):
        
        
        brick_length, _, brick_width, _ = self.get_brick_dimensions()
        brick_full = self.brick_params["brick_full"]
        center_brick_frame = brick_full.frame

        shift_vector = direction_vector * ((brick_length + brick_width)/2 + brick_spacing)
        
        if not course_is_odd:
            if start_edge_type == "corner":
                self.generate_corner_flemish_bond(
                    initial_brick_position=initial_brick_position,
                    bricks_per_course=bricks_per_course,
                    course_is_odd=course_is_odd,
                    direction_vector=direction_vector,
                    brick_spacing=brick_spacing,
                    start_edge_type=start_edge_type,
                    end_edge_type=end_edge_type                        
                )
    
            elif end_edge_type == "corner":
                self.generate_corner_flemish_bond(
                    initial_brick_position=initial_brick_position,
                    bricks_per_course=bricks_per_course,
                    course_is_odd=course_is_odd,
                    direction_vector=direction_vector,
                    brick_spacing=brick_spacing,
                    start_edge_type=start_edge_type,
                    end_edge_type=end_edge_type,
                )
                    
            else:
                for brick in range(bricks_per_course):
                    T = direction_vector * (brick * ((brick_length + brick_width)/2 + brick_spacing))
                    if course_is_odd:
                        T += shift_vector

                    brick_position = initial_brick_position + T                    
                    if direction_vector[1] in [-1, 1]:
                        brick_frame = Frame(brick_position, direction_vector, center_brick_frame.xaxis)
                    else:
                        brick_frame = Frame(brick_position, direction_vector, center_brick_frame.yaxis)

                    if brick % 2 != 0: #brick is odd - header
                        R1 = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                        T1 = Translation.from_vector(brick_frame.xaxis * (brick_length + brick_spacing)/2)
                        current_frame = brick_frame.transformed(R1*T1)
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame) 

                        # second row - insulated bricks - header bricks
                        T2 = Translation.from_vector(current_frame.xaxis * (brick_width + brick_spacing))
                        copy_current_frame = current_frame.transformed(T2)
                        if wall_system == "double_layer":
                            self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = copy_current_frame) 

                    else: #strecther bricks - first row
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "rotate", frame=brick_frame) 
                        
                        # second row - full bricks - stretcher bricks
                        if wall_system == "single_layer":                      
                            T3 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_spacing))
                            current_frame = brick_frame.transformed(T3)
                            self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame)
                        
                        if wall_system == "double_layer":
                            T4 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_width + 2 * brick_spacing))
                            current_frame = brick_frame.transformed(T4)
                            self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame) 

                            # middle bricks in the row
                            R2 = Rotation.from_axis_and_angle(current_frame.zaxis, math.radians(90), point=current_frame.point)
                            T5 = Translation.from_vector(current_frame.yaxis * ((brick_width - brick_length)/2 + brick_spacing/4))
                            T6 = Translation.from_vector(current_frame.xaxis * - ((brick_width + brick_length)/2 + brick_spacing))
                            current_frame = current_frame.transformed(R2 * T5 * T6)
                            self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame )

                            T7 = Translation.from_vector(current_frame.yaxis * - (brick_length + brick_spacing))
                            current_frame = current_frame.transformed(T7)
                            self.create_brick_and_add_to_assembly( brick_type = "insulated", transform_type = "fixed", frame = current_frame )

        else: #if course_is_odd:
            for brick in range(bricks_per_course):
                T = direction_vector * (brick * ((brick_length + brick_width)/2 + brick_spacing))
                if course_is_odd:
                    T += shift_vector

                brick_position = initial_brick_position + T
                if direction_vector[1] in [-1, 1]:
                    brick_frame = Frame(brick_position, direction_vector, center_brick_frame.xaxis)
                else:
                    brick_frame = Frame(brick_position, direction_vector, center_brick_frame.yaxis)
                
                if brick == 0:  # first brick in course
                    R3 = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                    current_frame = brick_frame.transformed(R3)
                    T8 = Translation.from_vector(current_frame.xaxis * ((brick_length + brick_spacing)/2))
                    T9 = Translation.from_vector(current_frame.yaxis * ((brick_length + brick_width)/2 + brick_spacing))
                    current_frame = current_frame.transformed(T9 * T8)
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame)

                    if wall_system == "double_layer":
                        # first insulated brick
                        T10 = Translation.from_vector(current_frame.xaxis *((brick_width + brick_spacing)))
                        current_frame = current_frame.transformed(T10)
                        self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame) 

                if brick >= 0 and brick < bricks_per_course - 1: 
                    if brick % 2 != 0: #brick is even               
                        # first row - header bricks
                        R4 = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                        T11 = Translation.from_vector(brick_frame.xaxis * (brick_length + brick_spacing)/2)
                        current_frame = brick_frame.transformed(R4 * T11)
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame)

                        # second row - insulated bricks - header bricks
                        T13 = Translation.from_vector(current_frame.xaxis * (brick_width + brick_spacing))
                        copy_current_frame = current_frame.transformed(T13)
                        if wall_system == "double_layer":                        
                            self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = copy_current_frame)

                    else:
                        # first row - self-shading bricks - stretcher bricks
                        T14 = Translation.from_vector(brick_frame.xaxis * (brick_length + brick_spacing))
                        current_frame = brick_frame.transformed(T14)
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "rotate", frame=brick_frame) 

                        if wall_system == "single_layer": # stretcher bricks - copy
                            T15 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_spacing))
                            current_frame = brick_frame.transformed(T15)
                            self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame)

                        if wall_system == "double_layer":
                            # last brick in the row
                            T14 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_width + 2 * brick_spacing))
                            current_frame = brick_frame.transformed(T14)
                            self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame) 

                            # middle bricks in the row
                            R5 = Rotation.from_axis_and_angle(current_frame.zaxis, math.radians(90), point=current_frame.point)
                            T15 = Translation.from_vector(current_frame.yaxis * ((brick_width - brick_length)/2 + brick_spacing/4))
                            T16 = Translation.from_vector(current_frame.xaxis * - ((brick_width + brick_length)/2 + brick_spacing))
                            current_frame = current_frame.transformed(R5 * T15 * T16)
                            self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame)

                            T17 = Translation.from_vector(current_frame.yaxis * -(brick_length + brick_spacing))
                            current_frame = current_frame.transformed(T17)
                            self.create_brick_and_add_to_assembly( brick_type = "insulated", transform_type = "fixed", frame = current_frame)

                elif brick == bricks_per_course - 1: # last brick even courses
                    R6 = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                    current_frame = brick_frame.transformed(R6)
                    T18 = Translation.from_vector(current_frame.xaxis * ((brick_length + brick_spacing)/2))
                    current_frame = current_frame.transformed(T18)
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame)

                    if wall_system == "double_layer":
                        # last insulated brick
                        T19 = Translation.from_vector(current_frame.xaxis *((brick_width + brick_spacing)))
                        current_frame = current_frame.transformed(T19)
                        self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame)

    def generate_corner_flemish_bond(self, 
                                    initial_brick_position,
                                    bricks_per_course,
                                    course_is_odd,
                                    direction_vector,
                                    brick_spacing,
                                    start_edge_type,
                                    end_edge_type,
                                    ):
        
        
        brick_length, _, brick_width, brick_length_h = self.get_brick_dimensions()
        brick_full = self.brick_params["brick_full"]
        center_brick_frame = brick_full.frame

        shift_vector = direction_vector * ((brick_length + brick_width)/2 + brick_spacing)
        for brick in range(bricks_per_course):
            T = direction_vector * (brick * ((brick_length + brick_width)/2 + brick_spacing))
            if course_is_odd:
                T += shift_vector

            brick_position = initial_brick_position + T

            if direction_vector[1] in [-1, 1]:
                brick_frame = Frame(brick_position, direction_vector, center_brick_frame.xaxis)
            else:
                brick_frame = Frame(brick_position, direction_vector, center_brick_frame.yaxis)

            if not course_is_odd:
                if brick % 2 != 0:  # last header brick
                    R1 = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)
                    T1 = Translation.from_vector(brick_frame.xaxis * (brick_length + brick_spacing) / 2)
                    current_frame = brick_frame.transformed(R1 * T1)
                    
                    if start_edge_type == 'corner' and brick == 1:
                        T2 = Translation.from_vector(brick_frame.xaxis * (brick_length_h / 2))
                        current_frame = current_frame.transformed(T2)
                        self.create_brick_and_add_to_assembly(brick_type="half", transform_type="fixed", frame=current_frame)

                    elif end_edge_type == 'corner' and brick == bricks_per_course - 2:
                        T3 = Translation.from_vector(brick_frame.xaxis * (-brick_length_h / 2))
                        current_frame = current_frame.transformed(T3)
                        self.create_brick_and_add_to_assembly(brick_type="half", transform_type="fixed", frame=current_frame)

                    else: # end_edge_type == 'corner':
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type="fixed", frame=current_frame)

                else:
                    if start_edge_type == 'corner' and brick == 0:
                        T4 = Translation.from_vector(brick_frame.xaxis * (brick_length_h))
                        current_frame = brick_frame.transformed(T4)
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type="fixed", frame=current_frame)

                        #T8 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_spacing))
                        #current_frame = current_frame.transformed(T8)
                        #self.create_brick_and_add_to_assembly(brick_type="insulated", transform_type="fixed", frame=current_frame)
                        
                    elif end_edge_type == 'corner' and brick == bricks_per_course - 1:
                        T5 = Translation.from_vector(brick_frame.xaxis * (-brick_length_h))
                        current_frame = brick_frame.transformed(T5)
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type="fixed", frame=current_frame)

                        #T6 = Translation.from_vector(current_frame.yaxis * (brick_length + brick_spacing))
                        #current_frame = current_frame.transformed(T6)
                        #self.create_brick_and_add_to_assembly(brick_type="insulated", transform_type="fixed", frame=current_frame)

                    else: #end_edge_type == 'corner':
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type="fixed", frame=brick_frame)

                        T7 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_spacing))
                        current_frame = brick_frame.transformed(T7)
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type="fixed", frame=current_frame)

    def calculate_flemish_course_length(self,
                                        bricks_per_course,
                                        brick_spacing,
                                        course_is_odd):
        """
        Calculate the total length of a Flemish bond course 

        Parameters
        ----------
        brick_full : :class:`CAEPart`
            The full brick to use for the wall.
        bricks_per_course : int
            The number of bricks in the course.
        brick_spacing : float
            The spacing between bricks.
        course_is_odd : bool
            True if the course is an odd-numbered course, False otherwise.

        Returns
        -------
        float
            The total length of the brick bond course.
        """

        brick_length, _, brick_width, _ = self.get_brick_dimensions()

        # Calculate the total length based on the pattern
        if course_is_odd:
            # Odd courses start and end with a header, alternate in between
            total_length = (bricks_per_course // 2) * (brick_width + brick_length + 2 * brick_spacing)
        else:
            # Even courses start and end with a stretcher, alternate in between
            total_length = (bricks_per_course // 2) * (brick_length + brick_width + 2 * brick_spacing)

        return total_length

    def reset_transformations(self):
        """
        Reset all transformations applied to the parts back to their original state.
        """
        for key in self.graph.nodes():
            part = self.part(key)
            if hasattr(part, 'original_frame'):
                # Reset the part's frame to its original frame
                T = Transformation.from_frame_to_frame(part.frame, part.original_frame)
                part.transform(T)


    def apply_gradient(self, values, points, keys, transform_type, rotation_direction, nrbh_size, reset):
        """
        Apply a gradient transformation to the parts.

        Parameters
        ----------
        values : list
            List of values to determine the transformation.
        points : list or array_like
            3D points corresponding to the parts (used for nearest neighbor search).
        keys : list
            List of keys identifying the parts.
        transform_type : str
            Type of transformation to apply ("translate" or "rotate"). 
        rotation_direction : str
            Direction for rotation ("left" or other).
        nrbh_size : int
            The number of nearest neighbors to consider.
        global_direction : array_like, optional
            A 3D vector representing the global reference for the gradient. 
            Default is np.array([0, 1, 0]).
        """

        if reset:
            self.reset_transformations()
            return

        # Build a KDTree for fast nearest neighbor search.
        tree = cKDTree(points)

        # Normalize the global direction (using compas.geometry.normalize_vector or numpy)
        global_direction = Vector(0, 1, 0)
        global_direction = normalize_vector(global_direction)

        for key in keys:
            part = self.part(key)
            part_position = part.frame.point  # Assumed to be a NumPy array

            # Nearest neighbor search to grab associated values
            distances, indices = tree.query(part_position, k=nrbh_size)
            neighbor_values = np.array([values[i] for i in indices])
            
            # Use inverse-distance weighting for interpolation
            weights = 1 / (distances + 1e-10)
            weights /= weights.sum()
            value = np.dot(weights, neighbor_values)

            translation_factor = value * -0.08  # Factor for translation
            rotation_factor = value * -0.1      # Factor for rotation

            if transform_type == "translate":
                # Determine the orientation of the brick based on its local frame and global direction
                brick_length, _, brick_width, _ = self.get_brick_dimensions()

                local_xaxis, local_yaxis = part.frame.xaxis, part.frame.yaxis

                # Compute the dot products to determine the facing direction
                dot_x = abs(local_xaxis.dot(global_direction))  
                dot_y = abs(local_yaxis.dot(global_direction))

                if brick_length > brick_width:
                    # xsize is the shorter dimension.
                    facing = "header" if dot_x >= dot_y else "strecher"
                else:
                    # ysize is the shorter dimension.
                    facing = "header" if dot_y >= dot_x else "strecher"
            
                if facing == "header":
                    translation_vector = local_yaxis * translation_factor
                else:
                    translation_vector = local_xaxis * translation_factor
                T = Translation.from_vector(translation_vector)

            elif transform_type == "rotate":
                center_brick_frame = part.frame
                if rotation_direction == "left":
                    rotation_factor = -rotation_factor
                    R = Rotation.from_axis_and_angle(center_brick_frame.zaxis, rotation_factor, point=center_brick_frame.point)
                    translation_vector = np.array(center_brick_frame.yaxis) * (-(0.09 * rotation_factor))
                else:
                    R = Rotation.from_axis_and_angle(center_brick_frame.zaxis, rotation_factor, point=center_brick_frame.point)
                    translation_vector = np.array(center_brick_frame.yaxis) * (0.09 * rotation_factor)
                    x_translation_vector = np.array(center_brick_frame.xaxis) * (-0.015 * rotation_factor)
                    translation_vector += x_translation_vector

                if value < 0:
                    translation_vector = -translation_vector

                T = R * Translation.from_vector(translation_vector)

            else:
                continue

            part.transform(T)

    def add_part_from_model(self, part, key=None, attr_dict=None, **kwargs):
        """Add a part to the assembly.

        Parameters
        ----------
        part : :class:`compas.datastructures.Part`
            The part to add.
        key : int | str, optional
            The identifier of the part in the assembly.
            Note that the key is unique only in the context of the current assembly.
            Nested assemblies may have the same `key` value for one of their parts.
            Default is None in which case the key will be an automatically assigned integer value.
        **kwargs: dict[str, Any], optional
            Additional named parameters collected in a dict.

        Returns
        -------
        int | str
            The identifier of the part in the current assembly graph.

        """
        if part.guid in self._parts:
            raise AssemblyError("Part already added to the assembly")
        
        key = self.graph.add_node(key=key, part=part, x=part.frame.point.x, y=part.frame.point.y, z=part.frame.point.z, **kwargs)
        part.key = key
        self._parts[part.guid] = part.key

        if attr_dict:
            for attr, value in attr_dict.items():
                part.attributes[attr] = value
                self.graph.node_attribute(key, attr, value)

        part.original_frame = part.frame.copy()

        return key
    
    def read_model_to_assembly(self, brick_list, brick_type, transform_type):
        """Read a model to an assembly.

        Parameters
        ----------
        brick_list : list
            List of bricks to add to the assembly.
        brick_type : str
            The type of brick to add to the assembly.
        transform_type : str, optional
            Type of transformation to apply ("fixed", "translate", or "rotate"). 
        """
        for brick in brick_list:

            if brick.Faces.Count > 0:
                face = brick.Faces[5]
                
                u, v = face.Domain(0).Mid, face.Domain(1).Mid
                plane = face.FrameAt(u, v)
                frame = plane_to_compas_frame(plane[1])

                brick_part = self.brick_params[brick_type]
                z_size = brick_part.shape.zsize
                translation_vector = -frame.zaxis * (z_size / 2)
                
                frame.point += translation_vector
                T = Transformation.from_frame_to_frame(brick_part.frame, frame)
                part = brick_part.transformed(T)

                part.frame = frame

            part_key = self.add_part_from_model(part, attr_dict={"brick_type": brick_type, 
                                                                 "transform_type": transform_type,
                                                                 })
            z_value = frame.point.z
            self.graph.node_attribute(part_key, 'z', z_value)

    def assembly_courses(self, tol=0.001):
        """Identify the courses in a wall of bricks.

        Parameters
        ----------
        wall : Assembly
            The wall assembly data structure.

        Examples
        --------
        .. code-block:: python

            pass

        """
        courses = []

        # all part keys
        parts = set(self.graph.nodes())

        # base course keys
        c_min = min(self.graph.nodes_attribute('z'))

        base = set()
        for e in parts:
            z = self.graph.node_attribute(key=e, name='z')
            if (z - c_min) ** 2 < tol:
                base.add(e)

        if base:
            courses.append(list(base))
            parts -= base
            while parts:  # and counter<1000:
                c_min = min([self.graph.node_attribute(key=key, name='z') for key in parts])
                base = set()
                for p in parts:
                    z = self.graph.node_attribute(key=p, name='z')
                    if (z - c_min) ** 2 < tol:
                        base.add(p)
                courses.append(list(base))
                parts -= base

        # assign course id's to the corresponding blocks
        for i, course in enumerate(courses):
            self.graph.nodes_attribute(name='course', value=i, keys=course)

        return courses   

    def project_part_faces(self, courses):
        """Project the faces of the parts in the assembly to 2D polygons.

        Parameters
        ----------
        courses : list
            List of courses in the assembly.

        Returns
        -------
        dict
            Dictionary mapping part keys to their corresponding 2D polygons.
        """    

        courses = self.assembly_courses()
        sorted_parts = [part for course in courses for part in course]
        polygons = {}
        
        for part in sorted_parts:
            part = self.part(part)
            mesh = part.mesh
            fkey_centroid = {fkey: mesh.face_center(fkey) for fkey in mesh.faces()}
            
            # Find the face with the lowest z-coordinate
            bottom_face = min(fkey_centroid.items(), key=lambda x: x[1][2])[0]

            # Get the vertices of the bottom face
            face_vertices = mesh.face_vertices(bottom_face)
            vertices = [mesh.vertex_coordinates(vkey) for vkey in face_vertices]

            # Project the vertices to 2D (x, y)
            projected_vertices = [(x, y) for x, y, z in vertices]

            # Create a 2D polygon from the projected vertices
            polygon = Polygon(projected_vertices)
            polygons[part.key] = polygon

        return polygons


        # for part in sorted_parts:
        #     frame_point = self.graph.node_attributes(part, 'xyz')
        #     part = self.part(part)
        #     for face in part.mesh.faces():
        #         face_vertices = part.mesh.face_vertices(face)
        #         face_coords = [part.mesh.vertex_coordinates(vkey) for vkey in face_vertices]
        #         if all(coord[2] == frame_point[2] for coord in face_coords):
        #             target_face = face
        #     face_vertices = part.mesh.face_vertices(target_face)
        #     vertices = [part.mesh.vertex_coordinates(vkey) for vkey in face_vertices]
        #     projected_vertices = [(x, y) for x, y, z in vertices]
        #     polygon = Polygon(projected_vertices)
        #     polygons[part.key] = polygon
        # return polygons

    def compute_polygon_intersections(self, polygons, courses):
        intersections = {}
        for i in range(len(courses) - 1):
            current_course = courses[i]
            next_course = courses[i + 1]


            current_course_parts = current_course
            next_course_parts = next_course

            # # Filter out parts with skip_intersections=True in the current course
            # current_course_parts = [
            #     part for part in current_course if not self.graph.node_attribute(part, 'skip_intersections')
            # ]

            # # Filter out parts with skip_intersections=True in the next course
            # next_course_parts = [
            #     part for part in next_course if not self.graph.node_attribute(part, 'skip_intersections')
            # ]

            for part in current_course_parts:
                part_polygon = polygons.get(part)
                if not part_polygon:
                    continue

                for neighbor in next_course_parts:
                    neighbor_polygon = polygons.get(neighbor)
                    if not neighbor_polygon:
                        continue

                    # Convert compas polygons to shapely polygons
                    shapely_part_polygon = ShapelyPolygon([(point.x, point.y) for point in part_polygon.points])
                    shapely_neighbor_polygon = ShapelyPolygon([(point.x, point.y) for point in neighbor_polygon.points])

                    # Compute intersection
                    intersection = shapely_part_polygon.intersection(shapely_neighbor_polygon)
                    if not intersection.is_empty:
                        # Convert shapely intersection back to compas polygon
                        intersection_coords = list(intersection.exterior.coords)
                        intersection_points = [Point(x, y, part_polygon.points[0].z) for x, y in intersection_coords]
                        intersections[(part, neighbor)] = Polygon(intersection_points)

        return intersections

    def transform_intersections_to_original_course(self, intersections, courses):
        transformed_intersections = {}
        for (part, neighbor), intersection in intersections.items():
            part_course = next(i for i, course in enumerate(courses) if part in course)
            neighbor_course = next(i for i, course in enumerate(courses) if neighbor in course)
            if part_course < neighbor_course:
                z_coord = self.graph.node_attribute(part, 'z')
                transformed_intersection_points = [Point(p.x, p.y, z_coord) for p in intersection.points]
                transformed_intersections[(part, neighbor)] = Polygon(transformed_intersection_points)
            else:
                z_coord = self.graph.node_attribute(neighbor, 'z')
                transformed_intersection_points = [Point(p.x, p.y, z_coord) for p in intersection.points]
                transformed_intersections[(neighbor, part)] = Polygon(transformed_intersection_points)

            attr = {
            'interface_type': 'face_face',
            'interface_points': [(point.x, point.y, point.z) for point in transformed_intersection_points],
            }
            self.graph.add_edge(part, neighbor, attr_dict=attr)

        return transformed_intersections

    def transform_intersections_to_course_above(self, intersections, courses):
        transformed_intersections = {}
        for (part, neighbor), intersection in intersections.items():
            neighbor_course = next(i for i, course in enumerate(courses) if neighbor in course)
            z_coord = self.graph.node_attribute(neighbor, 'z')
            transformed_intersection_points = [Point(p.x, p.y, z_coord) for p in intersection.points]
            transformed_intersections[(part, neighbor)] = Polygon(transformed_intersection_points)

            attr = {
                'interface_type': 'face_face',
                'interface_points': [(point.x, point.y, point.z) for point in transformed_intersection_points],
            }
            self.graph.add_edge(part, neighbor, attr_dict=attr)

        return transformed_intersections

    def set_interface_points(self):
        for key in self.graph.nodes():
            # Skip parts tagged with "skip_intersections"
            if self.graph.node_attribute(key, 'skip_intersections'):
                continue

            part = self.part(key)
            frame_point = self.graph.node_attributes(key, 'xyz')
            target_face = None

            # Find the face that matches the condition
            for face in part.mesh.faces():
                face_vertices = part.mesh.face_vertices(face)
                face_coords = [part.mesh.vertex_coordinates(vkey) for vkey in face_vertices]
                if all(coord[2] == frame_point[2] for coord in face_coords):
                    target_face = face
                    break

            # If no valid face is found, skip this part
            if target_face is None:
                continue

            # Get the vertices of the target face
            face_vertices = part.mesh.face_vertices(target_face)
            vertices = [part.mesh.vertex_coordinates(vkey) for vkey in face_vertices]
            projected_vertices = [(x, y) for x, y, z in vertices]

            # Store the interface points as a node attribute
            self.graph.node_attribute(key, 'interface_points', projected_vertices)

    def assembly_building_sequence(self, key):
        """Determine the sequence of bricks that need to be assembled to be able to
        place a target brick.

        Parameters
        ----------
        assembly : Assembly
            An assembly data structure.
        key : hashable
            The block identifier.

        Returns
        -------
        list
            A sequence of block identifiers.
        Notes
        -----
        This will only work for properly supported *wall* assemblies of which the
        interfaces and courses have been identified.

        Examples
        --------
        .. code-block:: python

            # this code only works in Rhino

            assembly = Assembly.from_json(...)

            placed = list(assembly.nodes_where({'is_placed': True}))

            artist = AssemblyArtist(assembly, layer="Assembly")

            artist.clear_layer()
            artist.draw_nodes()
            artist.draw_blocks(show_faces=False, show_edges=True)

            if placed:
                artist.draw_blocks(keys=placed, show_faces=True, show_edges=False)

            artist.redraw()

            key = AssemblyHelper.select_node(assembly)

            sequence = assembly_block_building_sequence(assembly, key)

            print(sequence)

            keys = list(set(sequence) - set(placed))

            artist.draw_blocks(keys=keys, show_faces=True, show_edges=False)
            artist.redraw()

        """

        course = self.graph.node_attribute(key, 'course')

        if course is None:
            raise Exception("The courses of the assembly have not been identified.")

        sequence = []
        seen = set()
        tovisit = deque([(key, course + 1)])

        while tovisit:
            k, course_above = tovisit.popleft()

            if k not in seen:
                seen.add(k)
                course = self.graph.node_attribute(k, 'course')

                if course_above == course + 1:
                    sequence.append(k)
                    for nbr in self.graph.neighbors(k):
                        if nbr not in seen:
                            tovisit.append((nbr, course))

        for i in range(len(sequence) - 1):
            self.add_connection(sequence[i], sequence[i + 1])

        for i in range(len(sequence)):
            current_node = sequence[i]
            current_z = self.graph.node_attribute(current_node, 'z')
            for j in range(i + 1, len(sequence)):
                next_node = sequence[j]
                next_z = self.graph.node_attribute(next_node, 'z')
                if next_z > current_z:
                    self.add_connection(current_node, next_node)
                    break

        return sequence[::-1]
    
