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

    def add_to_assembly(self,
                        brick_type, 
                        brick_full, 
                        brick_insulated,
                        fixed = True, 
                        frame=None
                        ): 
        """Create a brick with a specified type and add it to the assembly"""

        if frame is None:
            frame = frame

        if brick_type == "full":
            brick = brick_full

        if brick_type == "insulated":
            brick = brick_insulated
            
        my_brick = brick.transformed(Transformation.from_frame(frame))
        brick_height = brick_full.shape.zsize #brick_height is the same for "full" and "insulated" bricks
        # Adjust the gripping frame by translating it along the z-axis to account for the brick height
        gripping_frame = frame.transformed(Translation.from_vector(frame.zaxis*((brick_height-0.020)/2)))
        R = Rotation.from_axis_and_angle(gripping_frame.xaxis, math.radians(180), gripping_frame.point)
        gripping_frame.transform(R)
        my_brick.gripping_frame = gripping_frame

        self.add_part(my_brick, attr_dict={"brick_type": brick_type, "fixed": fixed})


    def get_brick_dimensions(self, brick_full, brick_insulated):
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

    def generate_flemish_bond(self,
                    brick_full,
                    brick_insulated,
                    initial_brick_center,
                    bricks_per_course,
                    plane,
                    course_is_odd,
                    direction_vector
                    ):
        
        brick_spacing = 0.015
        mortar_joint_height = 0.015
        center_brick_frame = plane_to_compas_frame(plane)
        brick_width_i, brick_length_i, brick_length, _, _ = self.get_brick_dimensions(brick_full, brick_insulated)
    
        params = {"brick_full": brick_full, 
                  "brick_insulated": brick_insulated 
                }

        for brick in range(bricks_per_course):
            T = direction_vector * (brick * ((brick_length_i + brick_width_i)/2 + mortar_joint_height))

            #Shifting every second row
            if course_is_odd == True:
                T += direction_vector * ((brick_length_i+brick_width_i+ 3* brick_spacing)/2-0.003)

            brick_center = initial_brick_center + T
            brick_frame = Frame(point_to_compas(brick_center), direction_vector, center_brick_frame.yaxis)

            if not course_is_odd:
                if brick % 2 != 0:
                    #self-shading bricks
                    R = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                    T1 = Translation.from_vector(brick_frame.xaxis * (brick_length + brick_spacing)/2)
                    current_frame = brick_frame.transformed(R*T1)
                    self.add_to_assembly(brick_type = "full", fixed = False, frame = current_frame, **params) 
                else:
                    #bricks - fixed
                    self.add_to_assembly(brick_type = "full", fixed = True, frame = brick_frame, **params) 
                    
                    T2 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_spacing))
                    current_frame = brick_frame.transformed(T2)
                    self.add_to_assembly(brick_type = "full", fixed = True, frame = current_frame, **params)
            
            if course_is_odd:
                if brick == 0: #Outter wall bricks
                    R = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                    current_frame = brick_frame.transformed(R)
                    T3 = Translation.from_vector(current_frame.xaxis *((brick_length + brick_spacing)/2))
                    current_frame = current_frame.transformed(T3)
                    T4 = Translation.from_vector(current_frame.yaxis * (brick_length_i + brick_length_i/2 + brick_width_i + 3 * brick_spacing)/2)
                    current_frame = current_frame.transformed(T4)
                    self.add_to_assembly(brick_type = "full", fixed = False, frame = current_frame, **params) 

                if brick >= 0 and brick < bricks_per_course-1:
                    if brick % 2 != 0:                
                        #self-shading bricks - first layer
                        R = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                        T5 = Translation.from_vector(brick_frame.xaxis * (brick_length + brick_spacing)/2)
                        current_frame = brick_frame.transformed(R*T5)
                        self.add_to_assembly(brick_type = "full", fixed = False, frame = current_frame, **params) 

                    else:
                        #bricks - first layer - solid
                        self.add_to_assembly(brick_type = "full", fixed = True, frame = brick_frame, **params) 

                        T6 = Translation.from_vector(brick_frame.yaxis * (brick_length + brick_spacing))
                        current_frame = brick_frame.transformed(T6)
                        self.add_to_assembly(brick_type = "full", fixed = True, frame = current_frame, **params)

                elif brick == bricks_per_course-1:
                    R = Rotation.from_axis_and_angle(brick_frame.zaxis, math.radians(90), point=brick_frame.point)  
                    current_frame = brick_frame.transformed(R)
                    T7 = Translation.from_vector(current_frame.xaxis *((brick_length + brick_spacing)/2))
                    current_frame = current_frame.transformed(T7)
                    T8 = Translation.from_vector(current_frame.yaxis * - (brick_length_i/2)/2)
                    current_frame = current_frame.transformed(T8)
                    self.add_to_assembly(brick_type = "full", fixed = False, frame = current_frame, **params) 


    def generate_wall(self,
                            bond,
                            brick_full,
                            brick_insulated,
                            plane,
                            lines
                                                                            ):
        brick_spacing = 0.015
        _, _, brick_length, _, brick_width = self.get_brick_dimensions(brick_full, brick_insulated)
    
        total_length = 0
        start_x = lines[1].FromX

        params = {"brick_full": brick_full,
                  "brick_insulated": brick_insulated,
                  "plane": plane
                   }

        for j, line in enumerate(lines):
            line_length = line.Length
            bricks_per_course = math.floor(line_length / ((brick_width + brick_length) /2 + brick_spacing))

            if bricks_per_course % 2 == 0:
                bricks_per_course = bricks_per_course - 1
            course_is_odd = j %2 != 0 #check if course is odd or even

            if course_is_odd == True and bricks_per_course % 2 != 0:
                bricks_per_course = bricks_per_course-1

            # Making sure all the lines are the same
            start_point = line.From
            start_point.X = start_x 
            line.From = start_point
            initial_brick_center = line.From

            # Calculate the direction vector of the line
            direction_vector = line.To - line.From
            direction_vector.Unitize()
            T = direction_vector * (brick_length + brick_spacing)
            initial_brick_center += T
        
            if j == 0:
                #Calculating the length of the wall
                half_bricks = math.ceil(bricks_per_course / 2)
                total_length += half_bricks * (brick_length + brick_spacing) + (bricks_per_course - half_bricks) * (brick_width + brick_spacing)
                total_length += 2 * (brick_length/2) 

            #Pick the bond   
            if bond == 0:
                self.generate_flemish_bond(
                        initial_brick_center = initial_brick_center,
                        bricks_per_course = bricks_per_course,
                        course_is_odd = course_is_odd,
                        direction_vector=direction_vector,
                        **params)                   
        return total_length

    def apply_gradient(self, values, keys):

        sorted_keys_values = sorted(zip(keys, values), key=lambda kv: kv[1])
        sorted_keys, sorted_values = zip(*sorted_keys_values)

        i = 0
        for key in keys:
            print(key)
            if key == 4: 
                pass
            else:
                part = self.part(key)
                if i < len(sorted_values):
                    y_translation = sorted_values[i] * -0.08
                else:
                    y_translation = 0  # Default value if sorted_values list is shorter than sorted_keys list

                
                # Calculate the translation vector using the direction vector
                translation_vector = part.frame.xaxis  * y_translation
                translation = Translation.from_vector(translation_vector)

                # Update the geometry position
                part.transform(translation)
            i += 1



