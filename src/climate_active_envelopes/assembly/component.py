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


    
    @classmethod
    def from_shape(cls, shape, frame=None):
        """Construct a component from a shape primitive.

        Parameters
        ----------
        shape : :class:`compas.geometry.Shape`
            Shape primitive describing the component.
        frame : :class:`Frame`
            Origin frame of the component.

        Returns
        -------
        :class:`Component`
            New instance of the component.
        """
        vertex = shape.vertex[0]
        frame = Frame([vertex['x'], vertex['y'], vertex['z']], [1, 0, 0], [0, 1, 0])
        component = cls(frame)
        component._source = shape
        component._mesh = Mesh.from_shape(component._source)
        component.vertices, component.faces = component._mesh.to_vertices_and_faces()  
        component.faces = [sorted(face) for face in component.faces]

        return component