from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json
import math as m
#from compas_fab.robots import JointTrajectoryPoint

from compas.datastructures import Mesh
from compas.geometry import Box
from compas.geometry import Frame, Translation, Rotation, Transformation


class CAEComponent(object):
    """Data structure representing a building component of the Cell Network.

    Attributes:
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
    >>> element = Element.from_box(Box(Frame.worldXY(), ))

    """

    def __init__(self, frame):
        super(CAEComponent, self).__init__()

        self.frame = frame #origin frame



    def from_box(self, box):
        """Initialize the component from a box."""
        self._source = box
        self._frame = box.frame
        self._mesh = box.to_mesh()
        self.faces = box.faces()
        self.vertices = box.vertices()

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


    def detect_connections(self):
        """Detect connections between the faces of the box."""
        connections = []
        for i, face1 in enumerate(self.faces):
            for j, face2 in enumerate(self.faces):
                if i >= j:
                    continue
                shared_vertices = set(face1).intersection(set(face2))
                if len(shared_vertices) >= 2:
                    connections.append((i, j))
        return connections