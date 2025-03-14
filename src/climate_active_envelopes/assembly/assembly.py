from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.datastructures import Datastructure
from compas.datastructures import Graph
from compas.datastructures import AssemblyError
from compas.geometry import Frame, Translation, Rotation, Transformation, Vector, Point, normalize_vector
from compas_rhino.conversions import plane_to_compas_frame, point_to_compas, mesh_to_rhino, point_to_rhino, vector_to_rhino

from numpy import array
from numpy import float64
from scipy.linalg import solve
from scipy.spatial import cKDTree
from compas.geometry import Polygon

import math as m


from compas.geometry import Frame
from compas.geometry import local_to_world_coordinates_numpy









from collections import deque

from assembly_information_model import Assembly
from .part import CAEPart as Part
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

    def __init__(self, name=None, brick_full=None, brick_insulated=None, brick_half=None, **kwargs):
        super(CAEAssembly, self).__init__()
        self.brick_params = None
        if brick_full and brick_insulated and brick_half:
            self.set_brick_params(brick_full, brick_insulated, brick_half)


    def export_to_json(self, path, is_built=False):

        # TODO!
        self.graph.update_default_node_attributes({"is_built":False})
        for key in self.parts():
            self.graph.node_attribute(key, "is_built", is_built)

        self.to_json(path)


    def set_brick_params(self, brick_full, brick_insulated, brick_half):

        self.brick_params = {
            "brick_full": brick_full,
            "brick_insulated": brick_insulated,
            "brick_half": brick_half,
           
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
    
    def compute_brick_layout(self, cell_network, course_height, brick_spacing):

        # get the dimensions of the bricks
        brick_length, _, brick_width, _, = self.get_brick_dimensions()

        # get the assembly data from the cell network
        assembly_data = cell_network.generate_assembly_data_from_cellnetwork(cell_network, course_height)

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
                      wall_system,  
                      brick_spacing, 
                      course_height, 
                      ):

        course_brick_data = self.compute_brick_layout(cell_network, course_height, brick_spacing)
        
        for data in course_brick_data:
            bricks_per_course, course_is_odd, direction_vector, start_edge_type, end_edge_type, curve_midpoint, adjusted_start_point, adjusted_end_point  = data

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
        #brick_air_dried = self.brick_params["brick_air_dried"]

        if frame is None:
            frame = frame

        if brick_type == "full":
            brick = brick_full

        if brick_type == "insulated":
            brick = brick_insulated

        if brick_type == "half":
            brick = brick_half

        # if brick_type == "air_dried":
        #     brick = brick_air_dried
            
        my_brick = brick.transformed(Transformation.from_frame(frame))

        # Adjust the gripping frame by translating it along the z-axis to account for the brick height
        gripping_frame = frame.transformed(Translation.from_vector(frame.zaxis*((brick_full.shape.zsize - 0.020)/2)))

        # Rotate the gripping frame by 180 degrees around the x-axis
        R = Rotation.from_axis_and_angle(gripping_frame.xaxis, math.radians(180), gripping_frame.point)
        gripping_frame.transform(R)
        # Set the gripping frame of the brick 
        my_brick.gripping_frame = gripping_frame
        my_brick.frame = frame

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
                            brick_full,
                            brick_insulated,
                            initial_brick_position,
                            line_length,          
                            plane,
                            course_is_odd,
                            j,
                            direction_vector,
                            wall_system):
        """
        Generates a Cross Bond pattern of bricks.

        Parameters
        ----------
        brick_full : :class:`CAEPart`
            The full brick to use for the wall.
        brick_insulated : :class:`CAEPart`
            The insulated brick to use for the wall.
        initial_brick_position : :class:`compas.geometry.Point`
            Starting center point for brick placement.
        line_length : float
            The length of the wall (the line along which bricks are laid).
        plane : :class:`compas.geometry.Plane`
            The reference plane for brick placement.
        course_is_odd : bool
            Boolean indicating if the course is odd.
        j : int
            Course index (used here for pattern ornamentation).
        wall_system : str
            The type of wall system to generate ("single_layer" or "double_layer").
        """
        
        brick_spacing = 0.015
        mortar_joint_height = 0.015
        brick_width_i, brick_length_i, brick_length, brick_height, brick_width = self.get_brick_dimensions(brick_full, brick_insulated)
      
        brick_params = {"brick_full": brick_full, 
                  "brick_insulated": brick_insulated 
                }

        
        center_brick_frame = plane_to_compas_frame(plane)
        ornament = "cross"  #name it

        if course_is_odd:
            num_bricks1 = math.floor(line_length / (brick_width+brick_spacing))
            num_bricks2 = math.floor(line_length / (brick_length+brick_spacing))
            # Odd courses: Bricks laid with the long side facing out
            for i in range(num_bricks1):

                # Calculate translation vector for the current brick
                T = plane.XAxis * -(i * (((brick_width+ brick_spacing)/2)))
                translation = Translation.from_vector(T)
                
                # Apply translation to the initial brick center
                brick_center = initial_brick_position + T
                brick_frame = Frame(point_to_compas(brick_center), center_brick_frame.xaxis, center_brick_frame.yaxis)
                
                # Transform the frame with translation
                current_frame = brick_frame.transformed(translation)
                # Add the brick to the assembly
                if ornament == "cross" or ornament =="straight":
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **brick_params)

                T1 = plane.YAxis * ((brick_width+ brick_length + (2*brick_spacing)))
                T2 = plane.XAxis * -((brick_length + brick_spacing)/2)
                translation1 = Translation.from_vector(T1)
                translation2 = Translation.from_vector(T2)
                current_frame = current_frame.transformed(translation1*translation2)
                if wall_system == "double_layer":
                    self.create_brick_and_add_to_assembly(brick_type="insulated", transform_type = "fixed", frame=current_frame, **brick_params) #maybe ornament can be here




            for i in range(num_bricks2):
                T = plane.XAxis * -(i * (((brick_length+brick_spacing)/2)))
                T1 = plane.YAxis * (((brick_width-brick_length)/2)+ (brick_length+brick_spacing))
                
                translation = Translation.from_vector(T)
                Translation2 = Translation.from_vector(T1)
                
                # Create the initial brick frame
                brick_center = initial_brick_position + T
                brick_frame = Frame(point_to_compas(brick_center), center_brick_frame.xaxis, center_brick_frame.yaxis)
                
                # Create a rotation transformation (90 degrees around Z-axis)
                R = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), brick_frame.point)
                
                # Apply rotation
                rotated_frame = brick_frame.transformed(R)
                
                # Apply translation to the rotated frame
                current_frame = rotated_frame.transformed(translation*Translation2)
                
    
                if wall_system == "double_layer":
                   self.create_brick_and_add_to_assembly(brick_type="insulated", transform_type = "fixed", frame=current_frame, **brick_params)



                        
        elif not course_is_odd:
            num_bricks = math.floor(line_length / (brick_length+brick_spacing))
            # Even courses: Bricks laid with the short side facing out (rotated by 90 degrees)
            for i in range(num_bricks):
                # Calculate translation vector based on brick length
                T = plane.XAxis * -(i * (((brick_length+brick_spacing)/2)))
                T1 = plane.YAxis * ((brick_width-brick_length)/2)
                
                translation = Translation.from_vector(T)
                Translation2 = Translation.from_vector(T1)
                
                # Create the initial brick frame
                brick_center = initial_brick_position + T
                brick_frame = Frame(point_to_compas(brick_center), center_brick_frame.xaxis, center_brick_frame.yaxis)
                
                # Create a rotation transformation (90 degrees around Z-axis)
                R = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), brick_frame.point)
                
                # Apply rotation
                rotated_frame = brick_frame.transformed(R)
                
                # Apply translation to the rotated frame
                current_frame = rotated_frame.transformed(translation*Translation2)
                
                # Add the rotated brick to the assembly
  
    
                if ornament == "cross": 
                    if i % 2 == 0 and j% 4 == 0:
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **brick_params)
                    elif i % 2 != 0 and j% 4 != 0:
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **brick_params)
                    else:
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **brick_params)

                if ornament == "straight": 
                    if i % 2 == 0:
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **brick_params)
                    else:
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **brick_params)


                if ornament == "diamond": 
                    if i % 2 == 0:
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **brick_params)
                    else:
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **brick_params)


                T2 = plane.YAxis * (brick_width+ brick_spacing)
                T3 = plane.XAxis * ((brick_length+brick_spacing)/2)
                Translation3 = Translation.from_vector(T2)
                Translation4= Translation.from_vector(T3)
                current_frame = current_frame.transformed(Translation3 * Translation4)
                if wall_system == "double_layer":
                    self.create_brick_and_add_to_assembly(brick_type="insulated", transform_type = "fixed", frame=current_frame, **brick_params)
  
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

    def apply_gradient(self, values, keys, transform_type):
        """
        Apply a gradient transformation to the parts.

        Parameters
        ----------
        values : list
            List of values to determine the transformation.
        keys : list
            List of keys identifying the parts.
        transform_type : str, optional
            Type of transformation to apply ("translate" or "rotate"). 
        """
        
        sorted_keys_values = sorted(zip(keys, values), key=lambda kv: kv[1])
        sorted_keys, sorted_values = zip(*sorted_keys_values)

        for i, key in enumerate(keys):
            part = self.part(key)
            if i < len(sorted_values):
                translation_factor = sorted_values[i] * -0.08  # factor for translation
                rotation_factor = sorted_values[i] * -0.4     # factor for rotation
            else:
                translation_factor = 0  # Default value if sorted_values list is shorter than sorted_keys list
                rotation_factor = 0  # Default value for rotation factor

            if transform_type == "translate":
                translation_vector = part.frame.xaxis * translation_factor
                T = Translation.from_vector(translation_vector)
               
            elif transform_type == "rotate":
                center_brick_frame = part.frame
                R = Rotation.from_axis_and_angle(center_brick_frame.zaxis, rotation_factor, point=center_brick_frame.point)
                translation_vector = center_brick_frame.yaxis * (0.1 * rotation_factor)
                T = R * Translation.from_vector(translation_vector)
            
            part.transform(T)

    # def add_part(self, part, key=None, attr_dict=None, **kwargs):
    #     """Add a part to the assembly.

    #     Parameters
    #     ----------
    #     part : :class:`compas.datastructures.Part`
    #         The part to add.
    #     key : int | str, optional
    #         The identifier of the part in the assembly.
    #         Note that the key is unique only in the context of the current assembly.
    #         Nested assemblies may have the same `key` value for one of their parts.
    #         Default is None in which case the key will be an automatically assigned integer value.
    #     **kwargs: dict[str, Any], optional
    #         Additional named parameters collected in a dict.

    #     Returns
    #     -------
    #     int | str
    #         The identifier of the part in the current assembly graph.

    #     """
    #     if part.guid in self._parts:
    #         raise AssemblyError("Part already added to the assembly")
        
    #     key = self.graph.add_node(key=key, part=part, x=part.frame.point.x, y=part.frame.point.y, z=part.frame.point.z, **kwargs)
    #     part.key = key
    #     self._parts[part.guid] = part.key

    #     if attr_dict:
    #         for attr, value in attr_dict.items():
    #             part.attributes[attr] = value
    #             #self.graph.node_attribute(key, attr, value)

    #     return key
    
    # def add_connection(self, a_key, b_key, **kwargs):
    #     """Add a connection between two parts.

    #     Parameters
    #     ----------
    #     a_key : int | str
    #         The identifier of the "from" part.
    #     b_key : int | str
    #         The identifier of the "to" part.
    #     **kwargs : dict[str, Any], optional
    #         Attribute dict compiled from named arguments.

    #     Returns
    #     -------
    #     tuple[int | str, int | str]
    #         The tuple of node identifiers that identifies the connection.

    #     Raises
    #     ------
    #     :class:`AssemblyError`
    #         If `a_key` and/or `b_key` are not in the assembly.

    #     """
    #     error_msg = "Both parts have to be added to the assembly before a connection can be created."
    #     if not self.graph.has_node(a_key) or not self.graph.has_node(b_key):
    #         raise AssemblyError(error_msg)
    #     #print(f"Adding connection between {a_key} and {b_key}")
    #     return self.graph.add_edge(a_key, b_key, **kwargs)

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
    
    def _find_nearest_neighbours(self, cloud, nmax):
        tree = cKDTree(cloud)
        nnbrs = [tree.query(root, nmax) for root in cloud]
        nnbrs = [(d.flatten().tolist(), n.flatten().tolist()) for d, n in nnbrs]
        return nnbrs

    def assembly_interfaces_numpy(self,
                                nmax=10,
                                tmax=1e-6,
                                amin=1e-1,
                                lmin=1e-3,
                                face_face=True,
                                face_edge=False,
                                face_vertex=False):
        """Identify the interfaces between the blocks of an assembly.

        Parameters
        ----------
        assembly : compas_assembly.datastructures.Assembly
            An assembly of discrete blocks.
        nmax : int, optional
            Maximum number of neighbours per block.
            Default is ``10``.
        tmax : float, optional
            Maximum deviation from the perfectly flat interface plane.
            Default is ``1e-6``.
        amin : float, optional
            Minimum area of a "face-face" interface.
            Default is ``1e-1``.
        lmin : float, optional
            Minimum length of a "face-edge" interface.
            Default is ``1e-3``.
        face_face : bool, optional
            Test for "face-face" interfaces.
            Default is ``True``.
        face_edge : bool, optional
            Test for "face-edge" interfaces.
            Default is ``False``.
        face_vertex : bool, optional
            Test for "face-vertex" interfaces.
            Default is ``False``.

        References
        ----------
        The identification of interfaces is discussed in detail here [Frick2016]_.

        Examples
        --------
        .. code-block:: python

            pass

        """
        # replace by something proper
        self.graph.edge = {}
        self.graph.halfedge = {}
        for key in self.graph.nodes():
            self.graph.edge[key] = {}
            self.graph.halfedge[key] = {}

        key_index = self.graph.node_index()
        index_key = self.graph.index_node()

        blocks = [self.part(key) for key in self.graph.nodes()]
        nmax = min(nmax, len(blocks))
        block_cloud = self.graph.nodes_attributes('xyz')

        print("block_cloud:", block_cloud)

        block_nnbrs = self._find_nearest_neighbours(block_cloud, nmax)

        print("block_nnbrs:", block_nnbrs)

        # k:      key of the base block
        # i:      index of the base block
        # block:  base block
        # nbrs:   list of indices of the neighbouring blocks
        # frames: list of frames for each of the faces of the base block

        # f0:   key of the current base face
        # A:    uvw base frame of f0
        # o:    origin of the base frame of f0
        # xyz0: xyz coordinates of the nodes of f0
        # rst0: local coordinates of the nodes of f0, with respect to the frame of f0
        # p0:   2D polygon of f0 in local coordinates

        # j:   index of the current neighbour
        # n:   key of the current neighbour
        # nbr: neighbour block
        # k_i: key index map for the nodes of the nbr block
        # xyz: xyz coorindates of all nodes of nbr
        # rst: local coordinates of all nodes of nbr, with respect to the frame of f0

        # f1:   key of the current neighbour face
        # rst1: local coordinates of the nodes of f1, with respect to the frame of f0
        # p1:   2D polygon of f1 in local coordinates

        interfaces = []
        for k in self.graph.nodes():
            print(f"Processing block with key: {k}")
            i = key_index[k]
            print(f"Index of block: {i}")
            block = self.part(k)
            print(f"Block: {block}")
            nbrs = block_nnbrs[i][1]
            print(f"Neighbours: {nbrs}")

            frames = block.face_frames
            print(f"Frames: {frames}")
            if face_face:
                print("Checking face-face interfaces")
                # parallelise?
                # exclude faces with parallel normals
                # e.g. exclude overlapping top faces of two neighbouring blocks in same row

                for f0, (origin, uvw) in frames.items():
                    print(f"Processing face: {f0}")
                    A = array(uvw, dtype=float64)
                    o = array(origin, dtype=float64).reshape((-1, 1))
                    xyz0 = array(block.mesh.face_coordinates(f0), dtype=float64).reshape((-1, 3)).T
                    rst0 = solve(A.T, xyz0 - o).T.tolist()
                    p0 = Polygon(rst0)

                    for j in nbrs:
                        n = index_key[j]
                        print(f"Processing neighbour: {n}")
                        if n == k:
                            print("Skipping self")
                            continue

                        if k in self.graph.edge and n in self.graph.edge[k]:
                            print("Skipping existing edge")
                            continue

                        if n in self.graph.edge and k in self.graph.edge[n]:
                            print("Skipping existing edge")
                            continue

                        nbr = self.part(n)
                        print(f"Neighbour block: {nbr}")
                        # print(nbr)
                        k_i = {key: index for index, key in enumerate(nbr.mesh.vertices())}
                        xyz = array(nbr.mesh.vertices_attributes('xyz'), dtype=float64).reshape((-1, 3)).T
                        rst = solve(A.T, xyz - o).T.tolist()
                        rst = {key: rst[k_i[key]] for key in nbr.mesh.vertices()}

                        for f1 in nbr.mesh.faces():
                            print(f"Processing neighbour face: {f1}")

                            rst1 = [rst[key] for key in nbr.mesh.face_vertices(f1)]

                            if any(m.fabs(t) > tmax for r, s, t in rst1):
                                print("Skipping face due to tmax")
                                continue

                            p1 = Polygon(rst1)

                            if p1.area < amin:
                                print("Skipping face due to amin")
                                continue

                            if p0.intersects(p1):
                                intersection = p0.intersection(p1)

                                area = intersection.area

                                if area >= amin:
                                    coords = [[x, y, 0.0] for x, y, z in intersection.exterior.coords]
                                    coords = local_to_world_coordinates_numpy(Frame(o, A[0], A[1]), coords)

                                    attr = {
                                        'interface_type': 'face_face',
                                        'interface_size': area,
                                        'interface_points': coords.tolist()[:-1],
                                        'interface_origin': origin,
                                        'interface_uvw': uvw,
                                    }

                                    self.graph.add_edge(k, n, attr_dict=attr)
                                    interfaces.append(attr)
                                    print(f"Added interface: {attr}")
        print("Final interfaces:", interfaces)
        return interfaces