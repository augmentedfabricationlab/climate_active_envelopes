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


__all__ = ['ReferenceElement']


class Cell(object):
    """Data structure representing a building cell of an reference model.

    Attributes
    ----------
    _frame : :class:`compas.geometry.Frame`
        The frame of the element.

    _source : :class:`compas.geometry.Shape`
        The source geometry of the element, e.g., `compas.geometry.Box`.

    _mesh : :class:`compas.geometry.Mesh`
        The mesh geometry of the element.


    Examples
    --------
    >>> from compas.datastructures import Mesh
    >>> from compas.geometry import Box
    >>> cell = Cell.from_box(Box(Frame.worldXY(), ))

    """

    def __init__(self, frame):
        super(Cell, self).__init__()

        self.frame = frame #origin frame

    @classmethod
    def from_dimensions(cls, length=3.0, height=2.5, depth=3.0):
        """Construct a primitive element with the given dimensions.

        Parameters
        ----------
        length : float
            length of the face
        height : float
            height of the face
        width : float
            width of the face.
        Returns
        -------
        :class:`Cell`
            New instance of cell.
        """
        frame = Frame.worldXY()
        
        cell = cls(frame)
        cell.length = length
        cell.height = height
        cell.depth = depth

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

        cell.vertices = vertices
        cell.faces = faces
        return cell

    @classmethod
    def from_box(cls, box):
        """Construct a cell from a box primitive.

        Parameters
        ----------
        box : :class:`compas.geometry.Box`
            Box primitive describing the element.

        Returns
        -------
        :class:`Cell`
            New instance of cell.
        """
        return cls.from_shape(box, box.frame)

    @classmethod
    def from_shape(cls, shape, frame):
        """Construct a cell from a shape primitive.

        Parameters
        ----------
        shape : :class:`compas.geometry.Shape`
            Shape primitive describing the cell.
        frame : :class:`Frame`
            Origin frame of the cell.

        Returns
        -------
        :class:`Cell`
            New instance of cell.
        """
        cell = cls(frame)
        cell._source = shape
        cell._mesh = Mesh.from_shape(cell._source)
        return cell


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


    def transform(self, transformation):
        """Transforms the cell.

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
        >>> cell = Cell.from_box(Box(Frame.worldXY(), 1, 1, 1))
        >>> cell.transform(Translation.from_vector([1, 0, 0]))
        """
        self.frame.transform(transformation)
        if self.mesh:
            mesh_transform(self.mesh, transformation)

    def transformed(self, transformation):
        """Returns a transformed copy of this cell.

        Parameters
        ----------
        transformation : :class:`Transformation`

        Returns
        -------
        Cell

        Examples
        --------
        >>> from compas.geometry import Box
        >>> from compas.geometry import Translation
        >>> cell = Element.from_box(Box(Frame.worldXY(), 1, 1, 1))
        >>> cell2 = element.transformed(Translation.from_vector([1, 0, 0]))
        """
        cell = self.copy()
        cell.transform(transformation)
        return cell

    def copy(self):
        """Returns a copy of this element.

        Returns
        -------
        Element
        """
        #TODO MUST BE EXPANDED WITH ATTRIBUTES
        cell = Cell(self.frame.copy())
        if self.mesh:
            cell.mesh = self.mesh.copy()
        return cell
