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
    def from_parameters(cls, frame, mesh, length, height, bond_type):
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

        element._mesh = mesh
        element._length = length
        element._height = height
        element._bond_type = bond_type

        element.brick_assembly = None

        return element

    @classmethod
    def from_rhinomesh_geometry(cls, rhino_mesh, frame):
        """Class method for constructing a block from a Rhino mesh.

        Parameters
        ----------

        """
        from compas_rhino.geometry import RhinoMesh
        element = cls(frame)
        rhmesh = RhinoMesh.from_geometry(rhino_mesh)
        element._mesh = element._source = rhmesh.to_compas()
        return element
    
    
    @classmethod
    def create_ref_elem(cls, frame, length, height):
        """Class method for constructing a mesh.

        Parameters
        frame, length, height

        :class:`Mesh`
            Mesh datastructure.
        :class:`Frame`
            Origin frame of the element.
        ----------
        """
        element = cls(frame)
        element.mesh = Mesh()
        element._length = length
        element._height = height

        # Add front side vertices
        for i in range(2):
            x = i * length
            for j in range(2):
                z = j * height
                element.mesh.add_vertex(x=x, y=0, z=z)
        # Add front side face
        for i in range(1):
            for j in range(1):
                a = i + j * 2
                b = a + 1
                c = b + 2
                d = a + 2
                element.mesh.add_face([a, b, c, d])
 
        element._source = element.mesh
        return element


    @classmethod
    def create_ref_elem_from_mesh(cls, frame, wall_length, wall_height, brick_length, brick_width, brick_height, mortar):
        """Class method for constructing a mesh.

        Parameters
        frame, length, height

        :class:`Mesh`
            Mesh datastructure.
        :class:`Frame`
            Origin frame of the element.
        ----------
        """

        element = cls(frame)
        element.mesh = Mesh()
        element.length = wall_length
        element.height = wall_height


        step_in_z = brick_height + mortar/2
        step_in_x = (brick_length+brick_width)/2 + mortar
        vertices_per_row = int(wall_length / step_in_x)
        row = int(wall_height / step_in_z)

        #create vertices
        for j in range(row): 
            z = j * step_in_z
            for i in range(vertices_per_row): 
                y = 0
                x = i * step_in_x
                element.mesh.add_vertex(x = x, y = y, z = z) 

        #create faces
        for j in range(row -1):
            for i in range(vertices_per_row - 1):
                a = i + j * vertices_per_row
                b = a + vertices_per_row
                c = b + 1
                d = a + 1
                element.mesh.add_face([a, b, c, d])
        element._source = element.mesh
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
        return self._mesh.centroid()

    @property
    def face_frames(self):
        """Compute the local frame of each face of the element's mesh.

        Returns
        -------
        dict
            A dictionary mapping face identifiers to face frames.
        """
        return {fkey: self.face_frame(fkey) for fkey in self._mesh.faces()}

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
        xyz = self._mesh.face_coordinates(fkey)
        o = self._mesh.face_center(fkey)
        w = self._mesh.face_normal(fkey)
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

        if self._mesh:
            #d['_mesh'] = _serialize_to_data(self._mesh)
            d['_mesh'] = self._mesh.to_data()

        return d

    @data.setter
    def data(self, data):

        #TODO MUST BE EXPANDED WITH ATTRIBUTES

        self.frame = Frame.from_data(data['frame'])
        if '_mesh' in data:
            #self._mesh = _deserialize_from_data(data['_mesh'])
            self._mesh = Mesh.from_data(data['_mesh'])

    def get_attribute(self, key):
        """key : The string of the attribute key."""
        return self.network.attributes.get(key, None)

    
    def set_attribute(self, key, value):
        """key : The string of the attribute key.
            value: the value
        """
        self.network.attributes[key] = value



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
    

    def generate_assembly(self):
        """Algorithm to generate the assembly model"""

        #self.brick_assembly = Assembly()
        print("generated brick assembly!")
