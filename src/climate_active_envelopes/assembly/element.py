from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


from assembly_information_model.assembly import Element

import json


__all__ = ['Element']


class CAEElement(Element):
    """Data structure representing a discrete element of an assembly.

    Attributes
    ----------
    frame : :class:`compas.geometry.Frame`
        The frame of the element.

    Examples
    --------
    >>> from compas.datastructures import Mesh
    >>> from compas.geometry import Box
    >>> element = Element.from_box(Box(Frame.worldXY(), ))

    """

    def __init__(self, frame):
        super(CAEElement, self).__init__(frame)
        self.message = "Hello"
        self.connector_frame_1 = None
        self.connector_frame_2 = None
        self.connector_1_state = True
        self.connector_2_state = True
        self._type = ''
        self._base_frame = None