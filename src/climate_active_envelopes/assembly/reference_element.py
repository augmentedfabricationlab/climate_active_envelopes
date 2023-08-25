from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json
import math as m
#from compas_fab.robots import JointTrajectoryPoint

from compas.datastructures import Mesh
from compas.datastructures import mesh_transform

from compas.geometry import Frame, Translation, Rotation, Transformation
from compas.geometry import centroid_points
from compas.geometry import cross_vectors
from compas.geometry import normalize_vector
from compas.geometry import centroid_polyhedron
from compas.geometry import volume_polyhedron
from compas_rhino.geometry import RhinoMesh
from .utilities import _deserialize_from_data
from .utilities import _serialize_to_data

from .brick_assembly import Assembly
from .brick import Brick


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


    @classmethod
    def from_parameters(cls, frame, mesh, length=3.0, height=3.0, bond_type="stretcher_header_bond"):
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

    def generate_brick_assembly(self, insulated_brick_mesh=None, brick_frame = None):
        """Algorithm to generate the assembly model"""

        if self.bond_type == "stretcher_header_bond":
            self.generate_brick_assembly_stretcher_header_bond(insulated_brick_mesh=insulated_brick_mesh, brick_frame = brick_frame)
        elif self.bond_type == "english_bond":
            self.generate_brick_assembly_english_bond()


    def generate_brick_assembly_stretcher_header_bond(self, brick_dimensions={"length": 0.24, "width": 0.115, "height": 0.075, "joint_height": 0.01}, insulated_brick_mesh=None, brick_frame = None):
        """ Algorithm to generate the brick_assembly model for stretcher header bond brickwork."""

        assembly = Assembly()

        frame = self.frame

        brick_length = brick_dimensions["length"]
        brick_height = brick_dimensions["height"]
        brick_width = brick_dimensions["width"]
        mortar_joint_height = brick_dimensions["joint_height"]

        courses = int(self.height / (brick_height + mortar_joint_height/2))
        bricks_per_course = int(self.length / ((brick_length + brick_width)/2 + mortar_joint_height))

        for course in range(courses):
            z_val = course * (brick_height + mortar_joint_height)
            course_is_odd = course%2 == 0
            course_is_even = course%2 == 1

            for brick in range(bricks_per_course):
                x_val = brick * ((brick_length + brick_width)/2 + mortar_joint_height)
                translation_vector = frame.xaxis * x_val + frame.zaxis * z_val
                T = Translation.from_vector(translation_vector)
                f1 = frame.transformed(T)
                brick_in_course_is_odd = brick%2 == 0
                brick_in_course_is_even = brick%2 == 1

                if course_is_odd:
                    if brick_in_course_is_odd: 
                        T = Translation.from_vector(f1.yaxis *+ (brick_length/4 + mortar_joint_height/4))
                        R = Rotation.from_axis_and_angle(f1.zaxis, m.radians(90), point=f1.point)
                        f1 = f1.transformed(T*R) #brick_full, alternating
                        my_brick = Brick.from_dimensions(f1, brick_dimensions["length"], brick_dimensions["width"], brick_dimensions["height"])
                        assembly.add_element(my_brick, attr_dict={"brick_type": "full", "fixed": False})

                        f4 = f1.copy() #brick insulated
                        T = Translation.from_vector(f4.xaxis *+ (brick_length + mortar_joint_height))
                        f4 = f4.transformed(T)
                        my_insulated_brick = Brick.from_mesh(insulated_brick_mesh, f4)
                        #my_brick = Brick.from_dimensions(f4, brick_dimensions["length"], brick_dimensions["width"], brick_dimensions["height"])
                        assembly.add_element(my_insulated_brick, attr_dict={"brick_type": "insulated", "fixed": True})


                    if brick_in_course_is_even:
                        f0 = f1 #brick_full, fixed
                        my_brick = Brick.from_dimensions(f0, brick_dimensions["length"], brick_dimensions["width"], brick_dimensions["height"])
                        assembly.add_element(my_brick, attr_dict={"brick_type": "full", "fixed": True})

                        f2 = f1.copy() #brick insulated
                        R = Rotation.from_axis_and_angle(f1.zaxis, m.radians(90), point=f2.point)
                        T1 = Translation.from_vector(f2.yaxis * (brick_width/2 + mortar_joint_height/2))
                        T2 = Translation.from_vector(f2.xaxis * ((brick_length/2+brick_width/2) + mortar_joint_height))
                        f2 = f2.transformed(R*T1*T2)
                        my_brick = Brick.from_dimensions(f2, brick_dimensions["length"], brick_dimensions["width"], brick_dimensions["height"])
                        assembly.add_element(my_brick, attr_dict={"brick_type": "insulated", "fixed": True})

                        f3 = f2.copy() #brick insulated
                        T = Translation.from_vector(f3.yaxis*- (brick_width+mortar_joint_height))
                        f3 = f3.transformed(T)
                        my_brick = Brick.from_dimensions(f3, brick_dimensions["length"], brick_dimensions["width"], brick_dimensions["height"])
                        assembly.add_element(my_brick, attr_dict={"brick_type": "insulated", "fixed": True})

                        f5 = f1.copy() #brick insulated
                        T = Translation.from_vector(f5.yaxis*+((brick_length+ brick_width) + mortar_joint_height*2))
                        f5 = f5.transformed(T)
                        my_brick = Brick.from_dimensions(f5, brick_dimensions["length"], brick_dimensions["width"], brick_dimensions["height"])
                        assembly.add_element(my_brick, attr_dict={"brick_type": "insulated", "fixed": True})


                if course_is_even:
                    if brick_in_course_is_even:
                        T = Translation.from_vector(f1.yaxis *+ (brick_length/4 + mortar_joint_height/4) )
                        R = Rotation.from_axis_and_angle(f1.zaxis, m.radians(90), point=f1.point)
                        f1 = f1.transformed(T*R) #brick_full, alternating
                        my_brick = Brick.from_dimensions(f1, brick_dimensions["length"], brick_dimensions["width"], brick_dimensions["height"])
                        assembly.add_element(my_brick, attr_dict={"brick_type": "insulated", "fixed": False})

                        f4 = f1.copy() #brick insulated
                        T = Translation.from_vector(f4.xaxis*+(brick_length + mortar_joint_height))
                        f4 = f4.transformed(T)
                        my_brick = Brick.from_dimensions(f4, brick_dimensions["length"], brick_dimensions["width"], brick_dimensions["height"])
                        assembly.add_element(my_brick, attr_dict={"brick_type": "insulated", "fixed": True})                        

                    if brick_in_course_is_odd: 
                        f0 = f1 #brick_full
                        my_brick = Brick.from_dimensions(f0, brick_dimensions["length"], brick_dimensions["width"], brick_dimensions["height"])
                        assembly.add_element(my_brick, attr_dict={"brick_type": "full", "fixed": True})

                        f2 = f1.copy() #brick insulated
                        R = Rotation.from_axis_and_angle(f2.zaxis, m.radians(90), point=f2.point)
                        T1 = Translation.from_vector(f2.yaxis *+ (brick_width/2 + mortar_joint_height/2))
                        T2 = Translation.from_vector(f2.xaxis *+ ((brick_length/2+brick_width/2) + mortar_joint_height))
                        f2 = f2.transformed(R*T1*T2)

                        my_brick = Brick.from_dimensions(f2, brick_dimensions["length"], brick_dimensions["width"], brick_dimensions["height"])
                        assembly.add_element(my_brick, attr_dict={"brick_type": "insulated", "fixed": True})

                        f3 = f2.copy() #brick insulated
                        T = Translation.from_vector(f3.yaxis*- (brick_width+mortar_joint_height))
                        f3 = f3.transformed(T)
                        my_brick = Brick.from_dimensions(f3, brick_dimensions["length"], brick_dimensions["width"], brick_dimensions["height"])
                        assembly.add_element(my_brick, attr_dict={"brick_type": "insulated", "fixed": True})

                        f5 = f1.copy() #brick insulated
                        T = Translation.from_vector(f5.yaxis*+((brick_length+ brick_width) + mortar_joint_height*2))
                        f5 = f5.transformed(T)
                        my_brick = Brick.from_dimensions(f5, brick_dimensions["length"], brick_dimensions["width"], brick_dimensions["height"])
                        assembly.add_element(my_brick, attr_dict={"brick_type": "insulated", "fixed": True})
        
        self.brick_assembly = assembly


    def generate_brick_assembly_english_bond(self):
    
        #algorithm: 
        pass
