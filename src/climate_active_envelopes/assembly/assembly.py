from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.datastructures import Datastructure
from compas.datastructures import Graph
from compas.datastructures import AssemblyError
from compas.geometry import Frame, Translation, Rotation, Transformation, Point, Vector, Plane
from compas_rhino.conversions import plane_to_compas_frame, point_to_compas

from assembly_information_model import Assembly
from .part import CAEPart as Part
#from .element import CAEElement as Element

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

    def __init__(self, name=None, **kwargs):
        super(CAEAssembly, self).__init__()

    def export_to_json(self, path, is_built=False):

        # TODO!
        self.graph.update_default_node_attributes({"is_built":False})
        for key in self.parts():
            self.graph.node_attribute(key, "is_built", is_built)

        self.to_json(path)


    def get_brick_dimensions(self, brick_full, brick_insulated):

        """
        Get the dimensions of the bricks
        
        Parameters
        ----------
        brick_full : :class:`CAEPart`
            The full brick to use for the wall
        brick_insulated : :class:`CAEPart`
            The insulated brick to use for the wall
        """

        brick_insulated_vertices = [brick_insulated.mesh.vertex_coordinates(vkey) for vkey in brick_insulated.mesh.vertices()]
        min_x = min(vertex[0] for vertex in brick_insulated_vertices)
        max_x = max(vertex[0] for vertex in brick_insulated_vertices)
        min_y = min(vertex[1] for vertex in brick_insulated_vertices)
        max_y = max(vertex[1] for vertex in brick_insulated_vertices)

        brick_width_i = max_x - min_x
        brick_length_i = max_y - min_y
        brick_length = brick_full.shape.ysize
        brick_height = brick_full.shape.zsize
        brick_width = brick_full.shape.xsize

        return brick_width_i, brick_length_i, brick_length, brick_height, brick_width


    
    def calculate_brick_parameters(self, contour_curves, brick_full, brick_insulated, brick_spacing):

        brick_parameters = []

        _, _, brick_length, _, brick_width = self.get_brick_dimensions(brick_full, brick_insulated)

        brick_length = brick_full.shape.ysize
        brick_width = brick_full.shape.xsize

        for course, contour_curve in enumerate(contour_curves):

            # Skip last course
            if course == len(contour_curves) - 1:
                continue

            curve_length = abs(contour_curve.GetLength())
            bricks_per_course = math.floor(curve_length / ((brick_width + brick_length) / 2 + brick_spacing))

            # Calculate the direction vector of the curve
            direction_vector = contour_curve.PointAtEnd - contour_curve.PointAtStart
            direction_vector.Unitize()

            # Calculate the first brick position to start from
            #T = direction_vector # * (abs(brick_length) + abs(brick_spacing))
            initial_brick_position = contour_curve.PointAtStart # + T       

            # check if course is odd or even   
            course_is_even = course % 2 == 0
            course_is_odd = course  %2 != 0 

            # number of bricks per course are always odd
            if bricks_per_course % 2 == 0:
                bricks_per_course -= 1

            # if course is odd and number of bricks per course is even, subtract 1
            if course_is_odd == True and bricks_per_course % 2 != 0:
                bricks_per_course -= 1

            brick_parameters.append((bricks_per_course, course_is_even, course_is_odd, direction_vector, initial_brick_position))
            
            
        return contour_curves, brick_parameters


    def generate_wall(self, bond_type, wallsystem, brick_full, brick_insulated, contour_curves, brick_spacing):

        contour_curves, brick_parameters = self.calculate_brick_parameters(contour_curves, brick_full, brick_insulated, brick_spacing)

        params = {
            "brick_full": brick_full,
            "brick_insulated": brick_insulated
        }
        
        for contour_curve, (bricks_per_course, course_is_even, course_is_odd, direction_vector, initial_brick_position) in zip(contour_curves, brick_parameters):

            # curve_midpoint = (contour_curve.PointAtStart + contour_curve.PointAtEnd) / 2

            # # Adjust the initial brick center to be centered around the midpoint
            # total_length =  bricks_per_course * brick_full.shape.ysize + (bricks_per_course - 1) * brick_spacing
            # initial_brick_position = curve_midpoint - (direction_vector * (total_length / 2))

            if bond_type == "flemish_bond":
                self.generate_flemish_bond(
                    initial_brick_position=initial_brick_position,
                    bricks_per_course=bricks_per_course,
                    course_is_odd=course_is_odd,
                    direction_vector=direction_vector,
                    wall_system=wallsystem,
                    **params
            #     )
            # elif bond_type == "vertical_bond":
            #     self.generate_vertical_bond(
            #         initial_brick_position=initial_brick_position,
            #         line_length=abs(lines[j].Length),
            #         course_is_odd=course_is_odd,
            #         j=j,
            #         direction_vector=direction_vector,
            #         wall_system=wallsystem,
            #         **params
            #     )
            # elif bond_type == "french_bond":
            #     self.generate_french_bond(
            #         initial_brick_position=initial_brick_position,
            #         line_length=abs(lines[j].Length),
            #         course_is_odd=course_is_odd,
            #         j=j,
            #         wall_system=wallsystem,
            #         **params
                )
            elif bond_type == "generate_flemish_for_cellnetwork":
                self.generate_flemish_for_cellnetwork(
                    initial_brick_position=initial_brick_position,
                    bricks_per_course=bricks_per_course,
                    course_is_odd=course_is_odd,
                    direction_vector=direction_vector,
                    wall_system=wallsystem,
                    brick_spacing=brick_spacing,
                    **params
                )

    def create_brick_and_add_to_assembly(self,
                        brick_type, 
                        brick_full, 
                        brick_insulated,
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

        if frame is None:
            frame = frame

        if brick_type == "full":
            brick = brick_full

        if brick_type == "insulated":
            brick = brick_insulated
            
        my_brick = brick.transformed(Transformation.from_frame(frame))

        # Adjust the gripping frame by translating it along the z-axis to account for the brick height
        gripping_frame = frame.transformed(Translation.from_vector(frame.zaxis*((brick_full.shape.zsize - 0.020)/2)))

        # Rotate the gripping frame by 180 degrees around the x-axis
        R = Rotation.from_axis_and_angle(gripping_frame.xaxis, math.radians(180), gripping_frame.point)
        gripping_frame.transform(R)
        # Set the gripping frame of the brick 
        my_brick.gripping_frame = gripping_frame

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
      
        params = {"brick_full": brick_full, 
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
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **params)
            else:
                T = plane.XAxis * -((((brick_length+brick_spacing+brick_width)/2)))
                translation = Translation.from_vector(T)
                current_frame = current_frame.transformed(translation)
                if wall_system == "single_layer" or wall_system == "double_layer":
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **params)

            
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
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **params)
                    T = plane.XAxis * -((((brick_length+brick_spacing))))
                    translation = Translation.from_vector(T)
                    current_frame = current_frame.transformed(translation)
                    if wall_system == "single_layer" or wall_system == "double_layer":
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **params)

                if wall_system == "double_layer":
                    T = plane.YAxis * (brick_spacing+brick_width)
                    translation = Translation.from_vector(T)
                    current_frame = current_frame.transformed(translation)
                    self.create_brick_and_add_to_assembly(brick_type="insulated", transform_type = "fixed", frame=current_frame, **params)

            else:
                T = plane.XAxis * -((((brick_length+brick_spacing+brick_width)/2)))
                translation = Translation.from_vector(T)
                current_frame = current_frame.transformed(translation)
                if wall_system == "single_layer" or wall_system == "double_layer":
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **params)
                    T = plane.XAxis * -((((brick_length+brick_spacing))))
                    translation = Translation.from_vector(T)
                    current_frame = current_frame.transformed(translation)
                    if wall_system == "single_layer" or wall_system == "double_layer":
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **params)
                        
                if wall_system == "double_layer":
                    T = plane.YAxis * (brick_spacing+brick_width)
                    translation = Translation.from_vector(T)
                    current_frame = current_frame.transformed(translation)
                    self.create_brick_and_add_to_assembly(brick_type="insulated", transform_type = "fixed", frame=current_frame, **params)

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
      
        params = {"brick_full": brick_full, 
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
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **params)

                T1 = plane.YAxis * ((brick_width+ brick_length + (2*brick_spacing)))
                T2 = plane.XAxis * -((brick_length + brick_spacing)/2)
                translation1 = Translation.from_vector(T1)
                translation2 = Translation.from_vector(T2)
                current_frame = current_frame.transformed(translation1*translation2)
                if wall_system == "double_layer":
                    self.create_brick_and_add_to_assembly(brick_type="insulated", transform_type = "fixed", frame=current_frame, **params) #maybe ornament can be here




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
                   self.create_brick_and_add_to_assembly(brick_type="insulated", transform_type = "fixed", frame=current_frame, **params)



                        
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
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **params)
                    elif i % 2 != 0 and j% 4 != 0:
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **params)
                    else:
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **params)

                if ornament == "straight": 
                    if i % 2 == 0:
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **params)
                    else:
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **params)


                if ornament == "diamond": 
                    if i % 2 == 0:
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **params)
                    else:
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **params)


                T2 = plane.YAxis * (brick_width+ brick_spacing)
                T3 = plane.XAxis * ((brick_length+brick_spacing)/2)
                Translation3 = Translation.from_vector(T2)
                Translation4= Translation.from_vector(T3)
                current_frame = current_frame.transformed(Translation3 * Translation4)
                if wall_system == "double_layer":
                    self.create_brick_and_add_to_assembly(brick_type="insulated", transform_type = "fixed", frame=current_frame, **params)
  
    def generate_flemish_bond(self,
                    brick_full,
                    brick_insulated,
                    initial_brick_position,
                    bricks_per_course,
                    plane,
                    course_is_even,
                    course_is_odd,
                    direction_vector,
                    wall_system,
                    ):
        """
        Generate a Flemish bond pattern for the wall.
        
        Parameters
        ----------
        brick_full : :class:`CAEPart`
            The full brick to use for the wall
        brick_insulated : :class:`CAEPart`
            The insulated brick to use for the wall
        initial_brick_position : :class:`compas.geometry.Point`
            The initial center point of the first brick
        bricks_per_course : int
            The number of bricks per course
        plane : :class:`compas.geometry.Plane`
            The plane on which the wall is generated
        course_is_odd : bool
            Check if the course is odd or even
        direction_vector : :class:`compas.geometry.Vector`
            The direction vector of the line
        transform_type : str, optional
            Type of transformation to apply ("translate" or "rotate").
        wall_system : str
            The type of wall system to generate ("single_layer" or "double_layer").
        """
        
        brick_spacing = 0.015
        mortar_joint_height = 0.015
        center_brick_frame = plane_to_compas_frame(plane)
        brick_width_i, brick_length_i, brick_length, brick_height, brick_width = self.get_brick_dimensions(brick_full, brick_insulated)
    
        params = {"brick_full": brick_full, 
                  "brick_insulated": brick_insulated 
           }

        shift_vector = direction_vector * ((brick_length + brick_width)/2 + brick_spacing + 0.005)
        
        for brick in range(bricks_per_course):
            T = direction_vector * (brick * ((brick_length_i + brick_width_i)/2 + mortar_joint_height))

            # Shifting every second row
            if course_is_odd:
                T += shift_vector

            brick_center = initial_brick_position + T
            brick_frame = Frame(point_to_compas(brick_center), direction_vector, center_brick_frame.yaxis)

            if course_is_even:
                if brick % 2 != 0: #brick is odd
                    # first row - self-shading bricks 
                    R1 = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                    T1 = Translation.from_vector(brick_frame.xaxis * (brick_length + brick_spacing)/2)
                    current_frame = brick_frame.transformed(R1*T1)
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **params) 

                    # second row - insulated bricks
                    T2 = Translation.from_vector(current_frame.xaxis * (brick_width_i + brick_spacing))
                    copy_current_frame = current_frame.transformed(T2)
                    if wall_system == "double_layer":
                        self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = copy_current_frame, **params) 
                    
                else: #brick is even
                    # first row - self-shading bricks 
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "rotate", frame=brick_frame, **params) 
                    
                    # second row - full bricks
                    if wall_system == "single_layer":
                        T3 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_spacing))
                        current_frame = brick_frame.transformed(T3)
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **params)

                    # second row - insulated bricks
                    if wall_system == "double_layer":
                        # last brick in the row
                        T4 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_width + 2 * brick_spacing))
                        current_frame = brick_frame.transformed(T4)
                        self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame , **params) 

                        # middle bricks in the row
                        R2 = Rotation.from_axis_and_angle(current_frame.zaxis, math.radians(90), point=current_frame.point)
                        T5 = Translation.from_vector(current_frame.yaxis * (brick_length_i + brick_spacing)/2)
                        current_frame = current_frame.transformed(R2 * T5)
                        T6 = Translation.from_vector(current_frame.xaxis * - ((brick_width_i + brick_length_i)/2 + brick_spacing))
                        current_frame = current_frame.transformed(T6)
                        self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame , **params)

                        T7 = Translation.from_vector(current_frame.yaxis * -(brick_width/2 + brick_spacing))
                        current_frame = current_frame.transformed(T7)
                        self.create_brick_and_add_to_assembly( brick_type = "insulated", transform_type = "fixed", frame = current_frame , **params)

            if course_is_odd:
                if brick == 0: # first self_shading brick
                    R3 = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                    current_frame = brick_frame.transformed(R3)
                    T8 = Translation.from_vector(current_frame.xaxis * ((brick_length + brick_spacing)/2))
                    current_frame = current_frame.transformed(T8)
                    T9 = Translation.from_vector(current_frame.yaxis * (brick_length_i + brick_length_i/2 + brick_width_i + 3 * brick_spacing)/2)
                    current_frame = current_frame.transformed(T9)
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **params) 

                    if wall_system == "double_layer":
                        # first insulated brick
                        T10 = Translation.from_vector(current_frame.xaxis *((brick_width_i+brick_spacing)))
                        current_frame = current_frame.transformed(T10)
                        self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame, **params) 


                if brick >= 0 and brick < bricks_per_course - 1: 
                    if brick % 2 != 0: #brick is even               
                        # first row - self-shading bricks
                        R4 = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                        T11 = Translation.from_vector(brick_frame.xaxis * (brick_length + brick_spacing)/2)
                        current_frame = brick_frame.transformed(R4 * T11)
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **params)

                        # second row - insulated bricks
                        T12 = Translation.from_vector(current_frame.xaxis * (brick_width_i + brick_spacing))
                        copy_current_frame = current_frame.transformed(T12)
                        if wall_system == "double_layer":                        
                            self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = copy_current_frame, **params)

                    else:
                        # first row - self-shading bricks
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "rotate", frame=brick_frame, **params) 

                        if wall_system == "single_layer":
                            T13 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_spacing))
                            current_frame = brick_frame.transformed(T13)
                            self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **params)

                        if wall_system == "double_layer":
                            # last brick in the row
                            T14 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_width + 2 * brick_spacing))
                            current_frame = brick_frame.transformed(T14)
                            self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame , **params) 

                            # middle bricks in the row
                            R5 = Rotation.from_axis_and_angle(current_frame.zaxis, math.radians(90), point=current_frame.point)
                            T15 = Translation.from_vector(current_frame.yaxis * (brick_length_i + brick_spacing)/2)
                            current_frame = current_frame.transformed(R5 * T15)
                            T16 = Translation.from_vector(current_frame.xaxis * - ((brick_width_i + brick_length_i)/2 + brick_spacing))
                            current_frame = current_frame.transformed(T16)
                            self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame , **params)

                            T17 = Translation.from_vector(current_frame.yaxis * -(brick_width/2 + brick_spacing))
                            current_frame = current_frame.transformed(T17)
                            self.create_brick_and_add_to_assembly( brick_type = "insulated", transform_type = "fixed", frame = current_frame , **params)

                elif brick == bricks_per_course - 1: # last self-shading brick
                    R6 = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                    current_frame = brick_frame.transformed(R6)
                    T18 = Translation.from_vector(current_frame.xaxis * ((brick_length + brick_spacing)/2))
                    current_frame = current_frame.transformed(T18)
                    T19 = Translation.from_vector(current_frame.yaxis * - (brick_length_i/2)/2)
                    current_frame = current_frame.transformed(T19)
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **params)

                    if wall_system == "double_layer":
                        # last insulated brick
                        T20 = Translation.from_vector(current_frame.xaxis *((brick_width_i+brick_spacing)))
                        current_frame = current_frame.transformed(T20)
                        self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame, **params)

    def generate_flemish_for_cellnetwork(self,
                                        brick_full,
                                        brick_insulated,
                                        initial_brick_position,
                                        bricks_per_course,
                                        course_is_odd,
                                        direction_vector,
                                        wall_system,
                                        brick_spacing,
                                        ):
        
        mortar_joint_height = 0.015
        center_brick_frame = brick_full.frame
        brick_width_i, brick_length_i, brick_length, brick_height, brick_width = self.get_brick_dimensions(brick_full, brick_insulated)



        params = {"brick_full": brick_full, 
                  "brick_insulated": brick_insulated 
           }

        shift_vector = direction_vector * ((brick_length + brick_width)/2 + brick_spacing + 0.005)
        
        for brick in range(bricks_per_course):

            # Shifting every second row
            T = direction_vector * (brick * ((brick_length_i + brick_width_i)/2 + mortar_joint_height))
            if course_is_odd:
                T += shift_vector

            brick_position = initial_brick_position + T
            brick_frame = Frame(point_to_compas(brick_position), direction_vector, center_brick_frame.yaxis)

            # courses are even
            if not course_is_odd:
                if brick % 2 != 0: #brick is odd
                    # first row - self-shading bricks 
                    R1 = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                    T1 = Translation.from_vector(brick_frame.xaxis * (brick_length + brick_spacing)/2)
                    current_frame = brick_frame.transformed(R1*T1)
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **params) 

                    # second row - insulated bricks
                    T2 = Translation.from_vector(current_frame.xaxis * (brick_width_i + brick_spacing))
                    copy_current_frame = current_frame.transformed(T2)
                    if wall_system == "double_layer":
                        self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = copy_current_frame, **params) 
                    
                else: #brick is even
                    # first row - self-shading bricks 
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "rotate", frame=brick_frame, **params) 
                    
                    # second row - full bricks
                    if wall_system == "single_layer":
                        T3 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_spacing))
                        current_frame = brick_frame.transformed(T3)
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **params)

                    # second row - insulated bricks
                    if wall_system == "double_layer":
                        T4 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_width + 2 * brick_spacing))
                        current_frame = brick_frame.transformed(T4)
                        self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame , **params) 

                        # middle bricks in the row
                        R2 = Rotation.from_axis_and_angle(current_frame.zaxis, math.radians(90), point=current_frame.point)
                        T5 = Translation.from_vector(current_frame.yaxis * (brick_length_i + brick_spacing)/2)
                        current_frame = current_frame.transformed(R2 * T5)
                        T6 = Translation.from_vector(current_frame.xaxis * - ((brick_width_i + brick_length_i)/2 + brick_spacing))
                        current_frame = current_frame.transformed(T6)
                        self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame , **params)

                        T7 = Translation.from_vector(current_frame.yaxis * -(brick_width/2 + brick_spacing))
                        current_frame = current_frame.transformed(T7)
                        self.create_brick_and_add_to_assembly( brick_type = "insulated", transform_type = "fixed", frame = current_frame , **params)

            if course_is_odd:
                if brick == 0:  # first brick in course
                    R3 = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                    current_frame = brick_frame.transformed(R3)
                    T8 = Translation.from_vector(current_frame.xaxis * ((brick_length + brick_spacing)/2))
                    current_frame = current_frame.transformed(T8)
                    T9 = Translation.from_vector(current_frame.yaxis * (brick_width - brick_length/2 + brick_spacing))
                    current_frame = current_frame.transformed(T9)
                    # T9 = Translation.from_vector(current_frame.yaxis * (brick_length_i + brick_length_i/2 + brick_width_i + 3 * brick_spacing)/2)
                    # current_frame = current_frame.transformed(T9)
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **params) 

                    if wall_system == "double_layer":
                        # first insulated brick
                        T10 = Translation.from_vector(current_frame.xaxis *((brick_width_i+brick_spacing)))
                        current_frame = current_frame.transformed(T10)
                        self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame, **params) 


                if brick >= 0 and brick < bricks_per_course - 1: 
                    if brick % 2 != 0: #brick is even               
                        # first row - fixed bricks
                        R4 = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                        T11 = Translation.from_vector(brick_frame.xaxis * (brick_length + brick_spacing)/2)
                        current_frame = brick_frame.transformed(R4 * T11)
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **params)

                        # second row - insulated bricks
                        T12 = Translation.from_vector(current_frame.xaxis * (brick_width_i + brick_spacing))
                        copy_current_frame = current_frame.transformed(T12)
                        if wall_system == "double_layer":                        
                            self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = copy_current_frame, **params)

                    else:
                        # first row - self-shading bricks
                        self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "rotate", frame=brick_frame, **params) 

                        if wall_system == "single_layer": # row behind the self-shading bricks
                            T13 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_spacing))
                            current_frame = brick_frame.transformed(T13)
                            self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "fixed", frame=current_frame, **params)

                        if wall_system == "double_layer":
                            # last brick in the row
                            T14 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_width + 2 * brick_spacing))
                            current_frame = brick_frame.transformed(T14)
                            self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame , **params) 

                            # middle bricks in the row
                            R5 = Rotation.from_axis_and_angle(current_frame.zaxis, math.radians(90), point=current_frame.point)
                            T15 = Translation.from_vector(current_frame.yaxis * (brick_length_i + brick_spacing)/2)
                            current_frame = current_frame.transformed(R5 * T15)
                            T16 = Translation.from_vector(current_frame.xaxis * - ((brick_width_i + brick_length_i)/2 + brick_spacing))
                            current_frame = current_frame.transformed(T16)
                            self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame , **params)

                            T17 = Translation.from_vector(current_frame.yaxis * -(brick_width/2 + brick_spacing))
                            current_frame = current_frame.transformed(T17)
                            self.create_brick_and_add_to_assembly( brick_type = "insulated", transform_type = "fixed", frame = current_frame , **params)

                elif brick == bricks_per_course - 1: # last brick
                    R6 = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                    current_frame = brick_frame.transformed(R6)
                    T18 = Translation.from_vector(current_frame.xaxis * ((brick_length + brick_spacing)/2))
                    current_frame = current_frame.transformed(T18)
                    #T19 = Translation.from_vector(current_frame.yaxis * - (brick_length_i/2)/2)
                    #current_frame = current_frame.transformed(T19)
                    self.create_brick_and_add_to_assembly(brick_type="full", transform_type = "translate", frame=current_frame, **params)

                    if wall_system == "double_layer":
                        # last insulated brick
                        T20 = Translation.from_vector(current_frame.xaxis *((brick_width_i+brick_spacing)))
                        current_frame = current_frame.transformed(T20)
                        self.create_brick_and_add_to_assembly(brick_type = "insulated", transform_type = "fixed", frame = current_frame, **params)


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

        #transform_types = {}
        #for node in self.graph.nodes():
        #    transform_types[node] = self.graph.node_attribute(node, 'transform_type')

        for i, key in enumerate(keys):
            part = self.part(key)
            if i < len(sorted_values):
                translation_factor = sorted_values[i] * -0.08  # factor for translation
                rotation_factor = sorted_values[i] * -0.4     # factor for rotation
            else:
                translation_factor = 0  # Default value if sorted_values list is shorter than sorted_keys list
                rotation_factor = 0  # Default value for rotation factor

            if transform_type == "translate":
                #part_transform_type = transform_types.get(str(key), None)
                #print(f"key: {key}, transform_type: {part_transform_type}")
                # Calculate the translation vector using the direction vector
                translation_vector = part.frame.xaxis * translation_factor
                T = Translation.from_vector(translation_vector)
               
            elif transform_type == "rotate":
                # Calculate the rotation transformation around the center_brick_frame
                center_brick_frame = part.frame
                R = Rotation.from_axis_and_angle(center_brick_frame.zaxis, rotation_factor, point=center_brick_frame.point)
                translation_vector = center_brick_frame.yaxis * (0.1 * rotation_factor)
                T = R * Translation.from_vector(translation_vector)
            
            # else:
            #     T = Transformation()

            part.transform(T)