from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json

#from compas_fab.robots import JointTrajectoryPoint

from compas.datastructures import Mesh
from compas.datastructures import mesh_transform

from compas.geometry import Frame
from compas.geometry import Box
from compas.geometry import centroid_points
from compas.geometry import cross_vectors
from compas.geometry import normalize_vector
from compas.geometry import centroid_polyhedron
from compas.geometry import volume_polyhedron

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

    def __init__(self, origin_frame):
        super(ReferenceElement, self).__init__()

        self.frame = origin_frame #origin frame


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


    def generate_brick_assembly(self):
        """Algorithm to generate the assembly model"""

        if self.bond_type == "stretcher_header_bond":
            self.generate_brick_assembly_stretcher_header_bond()
        elif self.bond_type == "english_bond":
            self.generate_brick_assembly_english_bond()


    def generate_brick_assembly_stretcher_header_bond(self, brick_dimensions={"length": 0.2159, "width": 0.1016, "height": 0.05715, "joint_height": 0.0127}):

        #algorithm: 
        # 1. generate plane grid

        brick_length = brick_dimensions["length"]
        brick_height = brick_dimensions["height"]
        brick_width = brick_dimensions["width"]
        mortar_joint_height = brick_dimensions["joint_height"]

        length = self.length
        height = self.height

        courses = int(height / (brick_height + mortar_joint_height/2))
        bricks_per_course = int(length / ((brick_length + brick_width)/2 + mortar_joint_height))
        #total_length = (bricks_per_course - 1) * (bricks_per_course)


        courses = int(height / (brick_height + mortar_joint_height/2))
        bricks_per_course = int(length / ((brick_length + brick_width)/2 + mortar_joint_height))

        frame_grid = []
        for course in range(courses):
            z_val = course * (brick_height + mortar_joint_height)

            planes = []
            for brick in range(bricks_per_course):
                x_val = brick * ((brick_length + brick_width)/2 + mortar_joint_height)

                origin = plane_from_frame.Origin + (x_val + (brick_length - brick_width)/2 + mortar)*plane_from_frame.XAxis + (z_val + (brick_height + mortar)/2)*plane_from_frame.ZAxis
                new_plane = rg.Plane(origin, plane_from_frame.XAxis, plane_from_frame.YAxis)

                planes.append(new_plane)
            plane_grid.append(planes)


        for i, row_planes in enumerate(plane_grid):
            row = []
            for j, plane in enumerate(row_planes):

                z1 = plane
                z1.Translate(z1.ZAxis*-height/2)


                #odd layers
                if i%2 == 0: 
                    if j%2 == 0: 
                        z1.Translate(z1.YAxis*+(length/4 + mortar/4))
                        z1.Rotate(m.radians(90), z1.ZAxis)
                        z4 = rg.Plane(z1)
                        z4.Translate(z4.XAxis*+(length + mortar))
                        row.append(z4)

                    if j%2 == 1:
                        z2 = rg.Plane(z1)
                        z2.Rotate(m.radians(90), z2.ZAxis)
                        z2.Translate(z2.YAxis*+(width/2 + mortar/2))
                        z2.Translate(z2.XAxis*+((length/2+width/2) + mortar))
                        row.append(z2)
                        z3 = rg.Plane(z2)
                        z2.Translate(z2.YAxis*-(width + mortar))
                        row.append(z3)
                        z5 = rg.Plane(z1)
                        z5.Translate(z5.YAxis*+((length+ width) + mortar*2))
                        row.append(z5)


                #even layers
                if i%2 == 1:
                    if j%2 == 1:
                        z1.Translate(z1.YAxis*+(length/4 + mortar/4))
                        z1.Rotate(m.radians(90), z1.ZAxis)
                        z4 = rg.Plane(z1)
                        z4.Translate(z4.XAxis*+(length + mortar))
                        row.append(z4)

                    if j%2 == 0:
                        z2 = rg.Plane(z1)
                        z2.Rotate(m.radians(90), z2.ZAxis)
                        z2.Translate(z2.YAxis*+(width/2 + mortar/2))
                        z2.Translate(z2.XAxis*+((length/2+width/2) + mortar))
                        row.append(z2)
                        z3 = rg.Plane(z2)
                        z2.Translate(z2.YAxis*-(width + mortar))
                        row.append(z3)
                        z5 = rg.Plane(z1)
                        z5.Translate(z5.YAxis*+((length+ width) + mortar*2))
                        row.append(z5)

                row.append(z1)
            grid.append(row)
                


        # 2. generate brick assembly
        assembly = Assembly()
        
        for i, row in enumerate(frame_grid): # number of layers and all planes
            for j, frame in enumerate(row): #j = planes / layer

                T = Transformation.from_frame_to_frame(Frame.worldXY(), frame)

                if i%2 == 1 and j%2 == 1 and j%3 != 1:
                    c = brick_full.transformed(T)
                    my_brick = Brick.from_dimensions(frame, brick_dimensions["length"], brick_dimensions["width"], brick_dimensions["height"], brick_dimensions["joint_height"])
                    assembly.add_element(my_brick, attr_dict={"brick_type": "full"})
                elif i%2 == 0 and j%2 == 1 and j%3 != 0:
                    c = brick_full.transformed(T)
                    assembly.add_element(c)
                else:
                    c = brick_insulated.transformed(T)
                    assembly.add_element(c)

        self.brick_assembly = assembly

    def generate_brick_assembly_english_bond(self):
    
        #algorithm: 
        pass
