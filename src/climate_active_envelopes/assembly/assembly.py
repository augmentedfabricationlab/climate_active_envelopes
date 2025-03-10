from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.datastructures import Datastructure
from compas.datastructures import Graph
from compas.datastructures import AssemblyError
from compas.geometry import Frame, Translation, Rotation, Transformation, Vector, Point, normalize_vector
from compas_rhino.conversions import plane_to_compas_frame, point_to_compas, mesh_to_rhino, point_to_rhino, vector_to_rhino

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
                        self.create_brick_and_add_to_assembly(brick_type="half", transform_type="translate", frame=current_frame)

                    elif end_edge_type == 'corner' and brick == bricks_per_course - 2:
                        T3 = Translation.from_vector(brick_frame.xaxis * (-brick_length_h / 2))
                        current_frame = current_frame.transformed(T3)
                        self.create_brick_and_add_to_assembly(brick_type="half", transform_type="translate", frame=current_frame)

                    else: # end_edge_type == 'corner':
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type="translate", frame=current_frame)

                else:
                    if start_edge_type == 'corner' and brick == 0:
                        T4 = Translation.from_vector(brick_frame.xaxis * (brick_length_h))
                        current_frame = brick_frame.transformed(T4)
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type="rotate", frame=current_frame)

                        #T8 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_spacing))
                        #current_frame = current_frame.transformed(T8)
                        #self.create_brick_and_add_to_assembly(brick_type="insulated", transform_type="fixed", frame=current_frame)
                        
                    elif end_edge_type == 'corner' and brick == bricks_per_course - 1:
                        T5 = Translation.from_vector(brick_frame.xaxis * (-brick_length_h))
                        current_frame = brick_frame.transformed(T5)
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type="rotate", frame=current_frame)

                        #T6 = Translation.from_vector(current_frame.yaxis * (brick_length + brick_spacing))
                        #current_frame = current_frame.transformed(T6)
                        #self.create_brick_and_add_to_assembly(brick_type="insulated", transform_type="fixed", frame=current_frame)

                    else: #end_edge_type == 'corner':
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type="rotate", frame=brick_frame)

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

    def add_part(self, part, key=None, attr_dict=None, **kwargs):
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

        return key
    
    def add_connection(self, a_key, b_key, **kwargs):
        """Add a connection between two parts.

        Parameters
        ----------
        a_key : int | str
            The identifier of the "from" part.
        b_key : int | str
            The identifier of the "to" part.
        **kwargs : dict[str, Any], optional
            Attribute dict compiled from named arguments.

        Returns
        -------
        tuple[int | str, int | str]
            The tuple of node identifiers that identifies the connection.

        Raises
        ------
        :class:`AssemblyError`
            If `a_key` and/or `b_key` are not in the assembly.

        """
        error_msg = "Both parts have to be added to the assembly before a connection can be created."
        if not self.graph.has_node(a_key) or not self.graph.has_node(b_key):
            raise AssemblyError(error_msg)
        #print(f"Adding connection between {a_key} and {b_key}")
        return self.graph.add_edge(a_key, b_key, **kwargs)

    def assembly_courses_xy(self, tol=0.01):
        """Identify the courses in a wall of bricks.

        Parameters
        ----------
        tol : float, optional
            Tolerance for identifying courses.

        Examples
        --------
        .. code-block:: python

            pass

        """
        courses = []

        # all element keys
        elements = set(self.graph.nodes())

        # base course keys
        c_min = min(self.graph.nodes_attribute('z'))
        #base = set(assembly.network.nodes_where({'z': c_min}))

        base = set()
        for e in elements:
            z = self.graph.node_attribute(key=e, name='z')
            if (z - c_min) ** 2 < tol:
                base.add(e)
        # print(base)

        if base:
            courses.append(list(base))

            elements -= base

            while elements:  # and counter<1000:

                c_min = min([self.graph.node_attribute(key=key, name='z') for key in elements])
                # print(c_min)
                #base = set(assembly.network.nodes_where({'z': c_min}))
                # print(base)
                base = set()
                for e in elements:
                    z = self.graph.node_attribute(key=e, name='z')
                    if (z - c_min) ** 2 < tol:
                        base.add(e)

                courses.append(list(base))
                elements -= base

        # assign course id's to the corresponding blocks
        for i, course in enumerate(courses):
            self.graph.nodes_attribute(name='course', value=i, keys=course)

        # Sort nodes within each course by proximity
        for i, course in enumerate(courses):
            sorted_course = []
            remaining_nodes = set(course)
            current_node = remaining_nodes.pop()
            sorted_course.append(current_node)

            while remaining_nodes:
                next_node = min(remaining_nodes, key=lambda node: self.distance_xy(current_node, node))
                sorted_course.append(next_node)
                remaining_nodes.remove(next_node)
                current_node = next_node

            courses[i] = sorted_course

        return courses


    def distance_xy(self, node1, node2):
        """Calculate the distance between two nodes in the x and y directions."""
        x1, y1, _ = self.graph.node_attributes(node1, ['x', 'y', 'z'])
        x2, y2, _ = self.graph.node_attributes(node2, ['x', 'y', 'z'])
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def assembly_courses_xyz(self, tol=0.01, xy_tol=0.04):

        """Identify the courses in a wall of bricks.

        Parameters
        ----------
        tol : float, optional
            Tolerance for identifying courses.

        Examples
        --------
        .. code-block:: python

            pass

        """
        courses = []

        # all element keys
        elements = set(self.graph.nodes())
        #print(f"All element keys: {elements}")

        # base course keys
        c_min = min(self.graph.nodes_attribute('z'))
        #print(f"Minimum z value: {c_min}")

        base = set()
        for e in elements:
            z = self.graph.node_attribute(key=e, name='z')
            #print(f"Element key: {e}, z value: {z}")
            if abs(z - c_min) < tol:
                base.add(e)

        if base:
            courses.append(list(base))
            elements -= base

            while elements:
                c_min = min([self.graph.node_attribute(key=key, name='z') for key in elements])
                base = set()
                for e in elements:
                    z = self.graph.node_attribute(key=e, name='z')
                    if abs(z - c_min) < tol:
                        base.add(e)

                if base:
                    courses.append(list(base))
                    elements -= base

        # Sort courses by their minimum z value
        courses.sort(key=lambda course: min(self.graph.node_attribute(key, 'z') for key in course))

        # Print the sorted courses for debugging
        for i, course in enumerate(courses):
            course_z_values = [self.graph.node_attribute(key, 'z') for key in course]
            #print(f"Course {i}: {course} with z values {course_z_values}")

        # assign course id's to the corresponding blocks
        for i, course in enumerate(courses):
            self.graph.nodes_attribute(name='course', value=i, keys=course)
            # Print the course assignment for debugging
            for key in course:
                assigned_course = self.graph.node_attribute(key, 'course')
                #print(f"Node {key} assigned to course {assigned_course}")

        # Sort nodes within each course by proximity using graph.neighbors
        for i, course in enumerate(courses):
            sorted_course = []
            remaining_nodes = set(course)
            current_node = remaining_nodes.pop()
            sorted_course.append(current_node)

            while remaining_nodes:
                neighbors = set(self.graph.neighbors(current_node))
                next_node = neighbors.intersection(remaining_nodes)
                if next_node:
                    next_node = next_node.pop()
                    sorted_course.append(next_node)
                    remaining_nodes.remove(next_node)
                    current_node = next_node
                else:
                    # If no direct neighbor is found, pick the closest remaining node
                    closest_node = min(remaining_nodes, key=lambda node: self.distance_xy(current_node, node))
                    sorted_course.append(closest_node)
                    remaining_nodes.remove(closest_node)
                    current_node = closest_node

            courses[i] = sorted_course

            # Add connections between each node and its next closest neighbor in either x or y axis
            for j in range(len(sorted_course) - 1):
                node = sorted_course[j]
                next_node = min(
                    sorted_course[j + 1:],
                    key=lambda n: self.distance_xy(node, n)
                )
                self.add_connection(node, next_node)

        # Add connections to the closest neighbor in the course below
        for i in range(1, len(courses)):
            current_course = courses[i]
            below_course = courses[i - 1]
            connected_nodes = set()
            for node in current_course:
                z_node = self.graph.node_attribute(key=node, name='z')
                x_node, y_node = self.graph.node_attributes(node, ['x', 'y'])
                available_neighbors = [
                    n for n in below_course if n not in connected_nodes and
                    abs(self.graph.node_attribute(key=n, name='x') - x_node) < xy_tol and
                    abs(self.graph.node_attribute(key=n, name='y') - y_node) < xy_tol
                ]
                if available_neighbors:
                    closest_neighbor = min(
                        available_neighbors,
                        key=lambda n: abs(z_node - self.graph.node_attribute(key=n, name='z'))
                    )
                    self.add_connection(node, closest_neighbor)
                    connected_nodes.add(closest_neighbor)

        #print("Reached the end of the method")
        return courses
    
    def assembly_with_interfaces_courses(self, tol=0.01):
        """Identify the courses in a wall of bricks.

        Parameters
        ----------
        tol : float, optional
            Tolerance for identifying courses.

        Examples
        --------
        .. code-block:: python

            pass

        """
        courses = []

        # all element keys
        elements = set(self.graph.nodes())

        # base course keys
        c_min = min(self.graph.nodes_attribute(name='z'))
        base = set(self.graph.nodes_where({'z': c_min}))

        if base:
            courses.append(list(base))
            elements -= base

            while elements:
                nbrs = set(nbr for key in courses[-1] for nbr in self.graph.neighbors(key))
                course = list(nbrs & elements)
                if course:
                    courses.append(course)
                    elements -= set(course)
                else:
                    next_base = set()
                    for e in elements:
                        z = self.graph.node_attribute(key=e, name='z')
                        if abs(z - c_min) < tol:
                            next_base.add(e)
                    if next_base:
                        courses.append(list(next_base))
                        elements -= next_base
                    else:
                        # If no new base course is found, break the loop to avoid infinite loop
                        break

        # assign course id's to the corresponding blocks
        for i, course in enumerate(courses):
            self.graph.nodes_attribute(name='course', value=i, keys=course)
        return courses
    
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

        # all element keys
        elements = set(self.graph.nodes())

        # base course keys
        c_min = min(self.graph.nodes_attribute('z'))
        #base = set(assembly.network.nodes_where({'z': c_min}))

        base = set()
        for e in elements:
            z = self.graph.node_attribute(key=e, name='z')
            if (z - c_min) ** 2 < tol:
                base.add(e)
        # print(base)

        if base:
            courses.append(list(base))

            elements -= base

            while elements:  # and counter<1000:

                c_min = min([self.graph.node_attribute(key=key, name='z') for key in elements])
                # print(c_min)
                #base = set(assembly.network.nodes_where({'z': c_min}))
                # print(base)
                base = set()
                for e in elements:
                    z = self.graph.node_attribute(key=e, name='z')
                    if (z - c_min) ** 2 < tol:
                        base.add(e)

                courses.append(list(base))
                elements -= base

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