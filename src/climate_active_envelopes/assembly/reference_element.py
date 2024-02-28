from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json
import math as m
#from compas_fab.robots import JointTrajectoryPoint

from compas.datastructures import Mesh, CellNetwork
from compas.datastructures import mesh_transform

from compas.geometry import Box
from compas.geometry import Frame, Translation, Rotation, Transformation
from compas.geometry import centroid_points
from compas.geometry import cross_vectors
from compas.geometry import normalize_vector
from compas.geometry import centroid_polyhedron
from compas.geometry import volume_polyhedron
from .utilities import _deserialize_from_data
from .utilities import _serialize_to_data

from .brick_assembly import Assembly
from .brick import Brick
from .cell import Cell


__all__ = ['ReferenceElement']


class ReferenceElement(object):
    """Data structure representing a building element of an reference model.

    Attributes
    ----------
    _frame : :class:`compas.geometry.Frame`
        The frame of the element.

    _tool_frame : :class:`compas.geometry.Frame`
        The frame of the element where the robot's tool should attach to.

    _source : :class:`compas.geometry.Shape`
        The source geometry of the element, e.g., `compas.geometry.Box`.

    _mesh : :class:`compas.geometry.Mesh`
        The mesh geometry of the element.

    trajectory : :class:`compas_fab.robots.JointTrajectory`
        The robot trajectory in joint space.

    path : :list: :class:`compas.geometry.Frame`
        The robot tool path in cartesian space.

    Examples
    --------
    >>> from compas.datastructures import Mesh
    >>> from compas.geometry import Box
    >>> element = Element.from_box(Box(Frame.worldXY(), ))

    """

    def __init__(self, frame):
        super(ReferenceElement, self).__init__()

        self.frame = frame #origin frame
        self.count = [0] #counter for self shading effect


    @classmethod
    def from_parameters(cls, frame, mesh, length=3.0, height=1.0, bond_type="stretcher_header_bond"):
        """Construct a reference element from a set of parameters.

        Parameters
        ----------
        mesh : :class:`Mesh`
            Mesh datastructure.
        frame : :class:`Frame`
            Origin frame of the element.

        Returns
        -------
        :class:`Element`
            New instance of element.
        """
        element = cls(frame)
        element.mesh = mesh
        element.length = length
        element.height = height
        element.bond_type = bond_type

        element.brick_assembly = None

        # element = cls(Frame(mesh.centroid(),[1, 0, 0], [0, 1, 0]))
        # T = Transformation.from_frame_to_frame(element.frame, t_frame)
        # mesh_transformed = mesh.transformed(T)
        # element._source = element._mesh = mesh_transformed

        return element


    @classmethod
    def from_dimensions(cls, length=3.0, height=2.5, depth=3.0):
        """Construct a primitive element with the given dimensions.

        Parameters
        ----------
        length : float
            length of the face

        width : float
            width of the face.
        Returns
        -------
        :class:`Element`
            New instance of element.
        """
        frame = Frame.worldXY()
        
        element = cls(frame)
        element.length = length
        element.height = height
        element.depth = depth

        vertices = [
            frame.point,
            frame.point + frame.xaxis * length,
            frame.point + frame.xaxis * length + frame.yaxis * height,
            frame.point + frame.yaxis * height,
            frame.point + frame.zaxis * depth,
            frame.point + frame.zaxis * depth + frame.xaxis * length,
            frame.point + frame.zaxis * depth + frame.xaxis * length + frame.yaxis * height,
            frame.point + frame.zaxis * depth + frame.yaxis * height
        ]

        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],
            [vertices[4], vertices[5], vertices[6], vertices[7]],
            [vertices[0], vertices[1], vertices[5], vertices[4]],
            [vertices[1], vertices[2], vertices[6], vertices[5]],
            [vertices[2], vertices[3], vertices[7], vertices[6]],
            [vertices[3], vertices[0], vertices[4], vertices[7]]
        ]

        element.vertices = vertices
        element.faces = faces
        return element

    @property
    def frame(self):
        """Frame of the element."""
        return self._frame

    @frame.setter
    def frame(self, frame):
        self._frame = frame.copy()

    @property
    def centroid(self):
        return self.mesh.centroid()

    @property
    def face_frames(self):
        """Compute the local frame of each face of the element's mesh.

        Returns
        -------
        dict
            A dictionary mapping face identifiers to face frames.
        """
        return {fkey: self.face_frame(fkey) for fkey in self.mesh.faces()}

    def face_frame(self, fkey):
        """Compute the frame of a specific face.

        Parameters
        ----------
        fkey : hashable
            The identifier of the frame.

        Returns
        -------
        frame
            The frame of the specified face.
        """
        xyz = self.mesh.face_coordinates(fkey)
        o = self.mesh.face_center(fkey)
        w = self.mesh.face_normal(fkey)
        u = [xyz[1][i] - xyz[0][i] for i in range(3)]  # align with longest edge instead?
        v = cross_vectors(w, u)
        uvw = normalize_vector(u), normalize_vector(v), normalize_vector(w)
        return o, uvw


    @classmethod
    def from_data(cls, data):
        """Construct an element from its data representation.

        Parameters
        ----------
        data : :obj:`dict`
            The data dictionary.

        Returns
        -------
        Element
            The constructed element.
        """
        element = cls(Frame.worldXY())
        element.data = data
        return element

    @property
    def data(self):
        """Returns the data dictionary that represents the element.

        Returns
        -------
        dict
            The element data.

        Examples
        --------
        >>> element = Element(Frame.worldXY())
        >>> print(element.data)
        """

        #TODO MUST BE EXPANDED WITH ATTRIBUTES

        d = dict(frame=self.frame.to_data())

        # Only include gripping plane if attribute is really set
        # (unlike the property getter that defaults to `self.frame`)
        if self._frame:
            d['_frame'] = self._frame.to_data()

        if self.mesh:
            #d['_mesh'] = _serialize_to_data(self._mesh)
            d['mesh'] = self.mesh.to_data()
        
        return d

    @data.setter
    def data(self, data):

        #TODO MUST BE EXPANDED WITH ATTRIBUTES

        self.frame = Frame.from_data(data['frame'])
        if 'mesh' in data:
            #self._mesh = _deserialize_from_data(data['_mesh'])
            self.mesh = Mesh.from_data(data['mesh'])

    def to_data(self):
        """Returns the data dictionary that represents the element.

        Returns
        -------
        dict
            The element data.

        Examples
        --------
        >>> from compas.geometry import Frame
        >>> e1 = Element(Frame.worldXY())
        >>> e2 = Element.from_data(element.to_data())
        >>> e2.frame == Frame.worldXY()
        True
        """
        return self.data

    def transform(self, transformation):
        """Transforms the element.

        Parameters
        ----------
        transformation : :class:`Transformation`

        Returns
        -------
        None

        Examples
        --------
        >>> from compas.geometry import Box
        >>> from compas.geometry import Translation
        >>> element = Element.from_box(Box(Frame.worldXY(), 1, 1, 1))
        >>> element.transform(Translation.from_vector([1, 0, 0]))
        """
        self.frame.transform(transformation)
        if self.mesh:
            mesh_transform(self.mesh, transformation)

    def transformed(self, transformation):
        """Returns a transformed copy of this element.

        Parameters
        ----------
        transformation : :class:`Transformation`

        Returns
        -------
        Element

        Examples
        --------
        >>> from compas.geometry import Box
        >>> from compas.geometry import Translation
        >>> element = Element.from_box(Box(Frame.worldXY(), 1, 1, 1))
        >>> element2 = element.transformed(Translation.from_vector([1, 0, 0]))
        """
        elem = self.copy()
        elem.transform(transformation)
        return elem

    def copy(self):
        """Returns a copy of this element.

        Returns
        -------
        Element
        """
        #TODO MUST BE EXPANDED WITH ATTRIBUTES
        elem = ReferenceElement(self.frame.copy())
        if self.mesh:
            elem.mesh = self.mesh.copy()
        return elem
    
    def create_self_shading(self, frame, color_values): 
        """Function to create self shading effect"""

        if self.count[0] < len(color_values): 
            translation_color_values = color_values[self.count[0]] # Get translation value from color_values list
            self.count[0] += 1  # Increment counter
            T = Translation.from_vector(frame.xaxis * -(translation_color_values)) #Translate frame in x-direction with values from color_values list 
            frame = frame.transformed(T) #Transform frame
            print("frame:", frame) 
        return frame  

    def create_brick_and_add_to_assembly(self, 
                                         brick_type, 
                                         brick_dimensions, 
                                         assembly, 
                                         color_values, 
                                         insulated_brick_mesh, 
                                         fixed = True, 
                                         frame=None, 
                                         create_self_shading = True): 
        """Create a brick with a specified type and to add it to the assembly"""

        if frame is None:
            frame = self.frame

        # Width based on brick type
        brick_width = brick_dimensions["width"]
        if brick_type == "half":
            brick_width /= 2

        # Create full brick
        my_brick = Brick.from_dimensions(frame, brick_dimensions["length"], brick_width, brick_dimensions["height"])

        # Apply self shading if True
        if create_self_shading and not fixed: #if self shading is True and brick is not fixed
            frame = self.create_self_shading(frame, color_values)
            my_brick = Brick.from_dimensions(frame, brick_dimensions["length"], brick_width, brick_dimensions["height"])

        # Create insulated brick
        if brick_type == "insulated":
            my_brick = Brick.from_mesh_and_frame(insulated_brick_mesh, frame)  

        assembly.add_element(my_brick, attr_dict={"brick_type": brick_type, "fixed": fixed})


    def generate_outer_corner_flemish_bond(self, 
                                            brick_dimensions={"length": 0.24, "width": 0.115, "height": 0.075, "joint_height": 0.01}, 
                                            insulated_brick_mesh=None, 
                                            color_values=None, 
                                            create_self_shading=True                                                                     
                                            ):
        """Function to generate the outer corner brick."""

        assembly = Assembly() #create assembly
        frame = self.frame
        brick_length = brick_dimensions["length"] #get brick dimensions
        brick_height = brick_dimensions["height"]
        brick_width = brick_dimensions["width"]
        mortar_joint_height = brick_dimensions["joint_height"] #get mortar joint height
        courses = int(self.height / (brick_height + mortar_joint_height/2)) #number of courses
        bricks_per_course = int(self.length / ((brick_length + brick_width)/2 + mortar_joint_height)) #number of bricks per course 

        params = {"brick_dimensions": brick_dimensions, 
                  "assembly": assembly, 
                  "color_values": color_values, 
                  "insulated_brick_mesh": insulated_brick_mesh, 
                  "create_self_shading": create_self_shading}
        
        for course in range(courses): #loop over courses
            z_val = course * (brick_height + mortar_joint_height) 
            course_is_odd = course%2 == 0 #check if course is odd or even

            for brick in range(bricks_per_course): #loop over bricks in course
                x_val = brick * ((brick_length + brick_width)/2 + mortar_joint_height) 
                T = Translation.from_vector(frame.xaxis * x_val + frame.zaxis * z_val)
                current_frame = frame.transformed(T)
                brick_in_course_is_odd = brick%2 == 0
                brick_in_course_is_even = brick%2 == 1                

                T1 = Translation.from_vector(current_frame.yaxis *+ (brick_width/2))
                T2 = Translation.from_vector(current_frame.xaxis *+ (brick_width - mortar_joint_height*2 + mortar_joint_height/2))
                current_frame = current_frame.transformed(T1*T2)  
                
                if course_is_odd: 
                    if brick_in_course_is_odd: 
                        if brick==0:
                            T = Translation.from_vector(current_frame.yaxis *+ (brick_length/4 + mortar_joint_height/4)) #brick_full, not_fixed
                            R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point) 
                            current_frame = current_frame.transformed(T*R) 
                            self.create_brick_and_add_to_assembly(brick_type="full", fixed=False, frame = current_frame, **params)

                            add_frame = current_frame.copy()
                            T1 = Translation.from_vector(add_frame.xaxis *+ (brick_length + brick_width/2+ mortar_joint_height*2))
                            add_frame.transform(T1)
                            self.create_brick_and_add_to_assembly(brick_type="full", fixed=True, frame = add_frame,**params)

                            copy_frame = add_frame.copy()
                            T2 = Translation.from_vector(copy_frame.yaxis *- (brick_width + mortar_joint_height))
                            copy_frame.transform(T2)
                            self.create_brick_and_add_to_assembly(brick_type="insulated", fixed=True, frame = copy_frame, **params) 

                            R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point) 
                            T1 = Translation.from_vector(current_frame.xaxis *- (brick_width/2 + mortar_joint_height/4))
                            T2 = Translation.from_vector(current_frame.yaxis *- (brick_width/2 + brick_length/2 - mortar_joint_height*2))
                            current_frame.transform(R*T1*T2)
                            self.create_brick_and_add_to_assembly(brick_type="half", fixed=False, frame = current_frame, **params)
                        
                    else: 
                        if brick == 1:
                            self.create_brick_and_add_to_assembly(brick_type="full", fixed=True, frame = current_frame, **params) # brick_full, fixed     
                        
                            R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(180), point=current_frame.point) #brick_insulated, fixed
                            T1 = Translation.from_vector(current_frame.yaxis *- (brick_width + mortar_joint_height))
                            current_frame = current_frame.transformed(R*T1)
                            self.create_brick_and_add_to_assembly(brick_type="insulated",fixed=True,frame = current_frame, **params)                                                    

                            T1 = Translation.from_vector(current_frame.yaxis *- (brick_length-brick_width/2 + mortar_joint_height/2)) #brick_full, not_fixed
                            T2 = Translation.from_vector(current_frame.xaxis *- (brick_width/2 + mortar_joint_height/2))
                            R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(-90), point=current_frame.point) 
                            current_frame = current_frame.transformed(T1*T2*R)
                            self.create_brick_and_add_to_assembly(brick_type="insulated",fixed=True,frame = current_frame, **params)

                            R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point) #brick_insulated, fixed
                            T1 = Translation.from_vector(current_frame.xaxis *- (brick_width/2 + mortar_joint_height/2))
                            T2 = Translation.from_vector(current_frame.yaxis *- (brick_width + brick_width/2 - mortar_joint_height - mortar_joint_height/2))
                            current_frame.transform(R*T1*T2)
                            self.create_brick_and_add_to_assembly(brick_type="half",fixed=False,frame = current_frame, **params)

                else: #course_is_even
                    if brick_in_course_is_even:
                        T = Translation.from_vector(current_frame.yaxis *+ (brick_length/4 + mortar_joint_height/4)) #brick_full, not_fixed
                        R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point)
                        current_frame = current_frame.transformed(T*R) #brick_full, not_fixed
                        if brick == 1:
                            T1 = Translation.from_vector(current_frame.yaxis *- (brick_width/4)) #brick_full, not_fixed 
                            current_frame.transform(T1)
                            brick_type="half"
                            self.create_brick_and_add_to_assembly(brick_type=brick_type, fixed=False,frame = current_frame, **params) 

                            T1 = Translation.from_vector(current_frame.xaxis *+ (brick_length + mortar_joint_height)) #brick_insulated, fixed
                            T2 = Translation.from_vector(current_frame.yaxis *- (mortar_joint_height/6))
                            T3 = Translation.from_vector(current_frame.yaxis *- (brick_length + mortar_joint_height)) 
                            current_frame.transform(T1*T2*T3)
                            self.create_brick_and_add_to_assembly(brick_type="half", fixed=True,frame = current_frame, **params) 
                                    
                    else: #brick_in_course_is_odd, brick_full, fixed
                        if brick == 0:
                            T = Translation.from_vector(current_frame.xaxis *+ (brick_width/2)) #brick_full, not_fixed 
                            current_frame.transform(T)
                            self.create_brick_and_add_to_assembly(brick_type="full",fixed=True, frame = current_frame, **params)                                             

                        R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point) #brick_insulated, fixed
                        T1 = Translation.from_vector(current_frame.yaxis *+ (brick_width/2 + mortar_joint_height/4))
                        T2 = Translation.from_vector(current_frame.xaxis *+ ((brick_length/2+brick_width/2) + mortar_joint_height))
                        current_frame = current_frame.transformed(R*T1*T2)                       
                        if brick == 0:
                            self.create_brick_and_add_to_assembly(brick_type="full", fixed=True, frame = current_frame,**params)
                        if brick == 2:
                            R1 = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point)
                            T3 = Translation.from_vector(current_frame.yaxis *+ (brick_width/2 + mortar_joint_height/4))
                            T4 = Translation.from_vector(current_frame.xaxis *- (brick_width/2 + mortar_joint_height/4))
                            current_frame = current_frame.transformed(R1*T3*T4)
                            self.create_brick_and_add_to_assembly(brick_type="insulated", fixed=True, frame = current_frame,**params)

                        if brick == 0:                                                
                            T = Translation.from_vector(current_frame.yaxis *- (brick_width + mortar_joint_height)) #brick_insulated, fixed
                            current_frame = current_frame.transformed(T)
                            self.create_brick_and_add_to_assembly(brick_type="insulated", fixed=True, frame = current_frame, **params)
                        if brick == 2:
                            T1 = Translation.from_vector(current_frame.xaxis *+ (brick_width/2 + mortar_joint_height/2))
                            current_frame = current_frame.transformed(T1)

                        R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point) #brick_insulated, fixed
                        T1 = Translation.from_vector(current_frame.xaxis *+ (brick_width/2 + mortar_joint_height))
                        T2 = Translation.from_vector(current_frame.yaxis *- ((brick_length/2 + brick_width/2) + mortar_joint_height/2 + mortar_joint_height))
                        current_frame = current_frame.transformed(R*T1*T2)
                        
                        if brick == 0:
                            self.create_brick_and_add_to_assembly(brick_type="full", fixed=True, frame = current_frame, **params)
                        elif brick == 2:
                            R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(-90), point=current_frame.point)
                            T1 = Translation.from_vector(current_frame.yaxis *- (brick_length/2 + brick_width/2 + mortar_joint_height+ mortar_joint_height/2))
                            T2 = Translation.from_vector(current_frame.xaxis *- (brick_length - brick_width/2 + mortar_joint_height))
                            current_frame = current_frame.transformed(R*T1*T2)
                            self.create_brick_and_add_to_assembly(brick_type="insulated", fixed=True, frame = current_frame, **params)
                            copy_frame = current_frame.copy()
                            T3 = Translation.from_vector(copy_frame.yaxis *- (brick_width + mortar_joint_height))
                            copy_frame.transform(T3)
                            self.create_brick_and_add_to_assembly(brick_type="insulated", fixed=True, frame = copy_frame, **params)
     
        self.brick_assembly = assembly                 

    def generate_brick_assembly_flemish_bond(self, 
                                             brick_dimensions={"length": 0.24, "width": 0.115, "height": 0.075, "joint_height": 0.01}, 
                                             insulated_brick_mesh=None, 
                                             color_values=None, 
                                             create_self_shading=True                                                                                      
                                             ):
        """Function to generate the brick_assembly model for the flemish bond."""

        assembly = Assembly() #create assembly
        frame = self.frame #get frame of element
        brick_length = brick_dimensions["length"] #get brick dimensions
        brick_height = brick_dimensions["height"]
        brick_width = brick_dimensions["width"]
        mortar_joint_height = brick_dimensions["joint_height"] #get mortar joint height
        courses = int(self.height / (brick_height + mortar_joint_height/2)) #number of courses
        bricks_per_course = int(self.length / ((brick_length + brick_width)/2 + mortar_joint_height)) #number of bricks per course     
       
        params = {"brick_dimensions": brick_dimensions, 
                  "assembly": assembly, 
                  "color_values": color_values, 
                  "insulated_brick_mesh": insulated_brick_mesh, 
                  "create_self_shading": create_self_shading}

        for course in range(courses): #loop over courses
            z_val = course * (brick_height + mortar_joint_height) 
            course_is_odd = course%2 == 0 #check if course is odd or even

            for brick in range(bricks_per_course): #loop over bricks in course
                x_val = brick * ((brick_length + brick_width)/2 + mortar_joint_height) 
                T = Translation.from_vector(frame.xaxis * x_val + frame.zaxis * z_val)
                current_frame = frame.transformed(T)
                brick_in_course_is_odd = brick%2 == 0
                brick_in_course_is_even = brick%2 == 1                

                T1 = Translation.from_vector(current_frame.yaxis *+ (brick_width/2))
                T2 = Translation.from_vector(current_frame.xaxis *+ (brick_width - mortar_joint_height*2 + mortar_joint_height/2))
                current_frame = current_frame.transformed(T1*T2)  
                
                if course_is_odd: 
                    if brick_in_course_is_odd: 
                        T = Translation.from_vector(current_frame.yaxis *+ (brick_length/4 + mortar_joint_height/4)) #brick_full, not_fixed
                        R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point) 
                        current_frame = current_frame.transformed(T*R) 

                        # if add_outer_corner and brick==0:
                        #     add_frame = current_frame.copy()
                        #     T1 = Translation.from_vector(add_frame.xaxis *+ (brick_length + brick_width/2+ mortar_joint_height*2))
                        #     add_frame.transform(T1)
                        #     brick_type="full"
                        #     self.create_brick_and_add_to_assembly(brick_type="full", fixed=True, frame = add_frame,**params)

                        #     copy_frame = add_frame.copy()
                        #     T2 = Translation.from_vector(copy_frame.yaxis *- (brick_width + mortar_joint_height))
                        #     copy_frame.transform(T2)
                        #     brick_type="insulated"
                        #     self.create_brick_and_add_to_assembly(brick_type=brick_type, fixed=True, frame = copy_frame, **params)

                        self.create_brick_and_add_to_assembly(brick_type="full", fixed=False, frame = current_frame, **params)
                        
                        T1 = Translation.from_vector(current_frame.xaxis *+ (brick_length + mortar_joint_height)) #brick_insulated, fixed
                        T2 = Translation.from_vector(current_frame.yaxis *- (mortar_joint_height/6))
                        current_frame = current_frame.transformed(T1*T2)
                        brick_type = "insulated"
                        fixed = True
                        # if add_outer_corner and brick == 0:
                        #     R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point) 
                        #     T1 = Translation.from_vector(current_frame.xaxis *- (brick_width/2 + mortar_joint_height/4))
                        #     T2 = Translation.from_vector(current_frame.yaxis *+ (brick_length/3 + mortar_joint_height))
                        #     current_frame.transform(R*T1*T2)
                        #     brick_type = "half"
                        #     fixed = False
                        self.create_brick_and_add_to_assembly(brick_type=brick_type, fixed=fixed, frame = current_frame, **params)
                        
                    else: 
                        self.create_brick_and_add_to_assembly(brick_type="full", fixed=True, frame = current_frame, **params) # brick_full, fixed
                        
                        R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point) #brick_insulated, fixed
                        T1 = Translation.from_vector(current_frame.yaxis *+ (brick_width/2 + mortar_joint_height/4))
                        T2 = Translation.from_vector(current_frame.xaxis *+ ((brick_length/2+brick_width/2) + mortar_joint_height))
                        current_frame = current_frame.transformed(R*T1*T2)
                        # if add_outer_corner and brick == 1:
                        #     T3 = Translation.from_vector(current_frame.yaxis *- (brick_length - brick_width/2 + mortar_joint_height/4))
                        #     current_frame = current_frame.transformed(T2*R*T3)
                        self.create_brick_and_add_to_assembly(brick_type="insulated",fixed=True,frame = current_frame, **params)
                        
                        T = Translation.from_vector(current_frame.yaxis *- (brick_width+mortar_joint_height)) #brick_insulated, fixed
                        current_frame = current_frame.transformed(T)
                        # if add_outer_corner and brick == 1:
                        #     T1 = Translation.from_vector(current_frame.yaxis *- (brick_length/2 - brick_width/2 )) #brick_full, not_fixed
                        #     T2 = Translation.from_vector(current_frame.xaxis *- (brick_width/2 + mortar_joint_height/2))
                        #     R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(-90), point=current_frame.point) 
                        #     current_frame = current_frame.transformed(T1*T2*R)
                        self.create_brick_and_add_to_assembly(brick_type="insulated",fixed=True,frame = current_frame, **params)
                        
                        R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point) #brick_insulated, fixed
                        T1 = Translation.from_vector(current_frame.xaxis *+ (brick_width/2 + mortar_joint_height))
                        T2 = Translation.from_vector(current_frame.yaxis *- ((brick_length/2 + brick_width/2) + mortar_joint_height/2 + mortar_joint_height))
                        current_frame = current_frame.transformed(R*T1*T2) 
                        brick_type = "insulated"
                        fixed = True
                        # if add_outer_corner and brick == 1:
                        #     T1 = Translation.from_vector(current_frame.xaxis *- (brick_length/2 + mortar_joint_height))
                        #     T2 = Translation.from_vector(current_frame.yaxis *+ (brick_width/3 - mortar_joint_height/2))
                        #     current_frame.transform(T1*T2)
                        #     brick_type = "half"
                        #     fixed = False
                        self.create_brick_and_add_to_assembly(brick_type=brick_type,fixed=fixed,frame = current_frame, **params)

                else: #course_is_even
                    if brick_in_course_is_even:
                        brick_type="full"
                        T = Translation.from_vector(current_frame.yaxis *+ (brick_length/4 + mortar_joint_height/4)) #brick_full, not_fixed
                        R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point)
                        current_frame = current_frame.transformed(T*R) #brick_full, not_fixed
                        # if add_outer_corner and brick == 1:
                        #     T1 = Translation.from_vector(current_frame.yaxis *- (brick_width/4)) #brick_full, not_fixed 
                        #     current_frame.transform(T1)
                        #     brick_type="half"
                        self.create_brick_and_add_to_assembly(brick_type=brick_type, fixed=False,frame = current_frame, **params) 

                        T1 = Translation.from_vector(current_frame.xaxis *+ (brick_length + mortar_joint_height)) #brick_insulated, fixed
                        T2 = Translation.from_vector(current_frame.yaxis *- (mortar_joint_height/6))
                        current_frame = current_frame.transformed(T1*T2)
                        brick_type="insulated"
                        fixed = True
                        # if add_outer_corner and brick == 1:
                        #     T3 = Translation.from_vector(current_frame.yaxis *- (brick_length + mortar_joint_height)) 
                        #     current_frame.transform(T3)
                        #     brick_type="half"
                        #     fixed = True
                        self.create_brick_and_add_to_assembly(brick_type=brick_type, fixed=fixed,frame = current_frame, **params) 
                                    
                    else: #brick_in_course_is_odd, brick_full, fixed
                        # if add_outer_corner and brick == 0:
                        #     T = Translation.from_vector(current_frame.xaxis *+ (brick_width/2)) #brick_full, not_fixed 
                        #     current_frame.transform(T)
                        self.create_brick_and_add_to_assembly(brick_type="full",fixed=True, frame = current_frame, **params)                                             

                        R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point) #brick_insulated, fixed
                        T1 = Translation.from_vector(current_frame.yaxis *+ (brick_width/2 + mortar_joint_height/4))
                        T2 = Translation.from_vector(current_frame.xaxis *+ ((brick_length/2+brick_width/2) + mortar_joint_height))
                        current_frame = current_frame.transformed(R*T1*T2)
                        brick_type="insulated"
                        # if add_outer_corner:
                        #     if brick == 0:
                        #         brick_type="full"
                        #     if brick == 2:
                        #         R1 = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point)
                        #         T3 = Translation.from_vector(current_frame.yaxis *+ (brick_width/2 + mortar_joint_height/4))
                        #         T4 = Translation.from_vector(current_frame.xaxis *- (brick_width/2 + mortar_joint_height/4))
                        #         current_frame = current_frame.transformed(R1*T3*T4)
                        #         brick_type="insulated"
                        self.create_brick_and_add_to_assembly(brick_type=brick_type, fixed=True, frame = current_frame,**params)
                                                                        
                        T = Translation.from_vector(current_frame.yaxis *- (brick_width + mortar_joint_height)) #brick_insulated, fixed
                        current_frame = current_frame.transformed(T)
                        # if add_outer_corner and brick == 2:
                        #     T1 = Translation.from_vector(current_frame.xaxis *+ (brick_width/2 + mortar_joint_height/2))
                        #     current_frame = current_frame.transformed(T1)
                        self.create_brick_and_add_to_assembly(brick_type="insulated", fixed=True, frame = current_frame, **params)

                        R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(90), point=current_frame.point) #brick_insulated, fixed
                        T1 = Translation.from_vector(current_frame.xaxis *+ (brick_width/2 + mortar_joint_height))
                        T2 = Translation.from_vector(current_frame.yaxis *- ((brick_length/2 + brick_width/2) + mortar_joint_height/2 + mortar_joint_height))
                        current_frame = current_frame.transformed(R*T1*T2)
                        brick_type="insulated"
                        # if add_outer_corner:
                        #     if brick == 0:
                        #         brick_type="full"
                        #     elif brick == 2:
                        #         R = Rotation.from_axis_and_angle(current_frame.zaxis, m.radians(-90), point=current_frame.point)
                        #         T1 = Translation.from_vector(current_frame.yaxis *- (brick_length/2 + brick_width/2 + mortar_joint_height+ mortar_joint_height/2))
                        #         T2 = Translation.from_vector(current_frame.xaxis *- (brick_length - brick_width/2 + mortar_joint_height))
                        #         current_frame = current_frame.transformed(R*T1*T2)
                        self.create_brick_and_add_to_assembly(brick_type=brick_type, fixed=True, frame = current_frame, **params)
     
        self.brick_assembly = assembly 




    def generate_brick_assembly(self, 
                                insulated_brick_mesh=None, 
                                color_values=None, 
                                create_self_shading=True
                                ):
        """Function to generate the brick_assembly model."""

        if self.bond_type == "flemish_bond":
            self.generate_brick_assembly_flemish_bond(insulated_brick_mesh=insulated_brick_mesh, 
                                                      color_values=color_values, 
                                                      create_self_shading=create_self_shading
                                                      )
            
    def generate_outer_corner(self, 
                            insulated_brick_mesh=None, 
                            color_values=None, 
                            create_self_shading=True
                            ):
        """Function to generate the outer corner bricks."""

        if self.bond_type == "flemish_bond":
            self.generate_outer_corner_flemish_bond(insulated_brick_mesh=insulated_brick_mesh, 
                                       color_values=color_values, 
                                       create_self_shading=create_self_shading
                                       )
