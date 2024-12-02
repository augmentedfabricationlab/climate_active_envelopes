from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.geometry import Frame
from compas.datastructures import Mesh
from compas.datastructures import Datastructure

from compas.geometry import cross_vectors
from compas.geometry import normalize_vector
from compas.geometry import centroid_polyhedron
from compas.geometry import volume_polyhedron

from assembly_information_model import Part


class CAEPart(Part):
    """A data structure for representing assembly parts.

    Parameters
    ----------
    name : str, optional
        The name of the part.
        The name will be stored in :attr:`Part.attributes`.
    frame : :class:`compas.geometry.Frame`, optional
        The local coordinate system of the part.

    Attributes
    ----------
    attributes : dict[str, Any]
        General data structure attributes that will be included in the data dict and serialization.
    key : int or str
        The identifier of the part in the connectivity graph of the parent assembly.
    frame : :class:`compas.geometry.Frame`
        The local coordinate system of the part.
    features : list(:class:`compas.datastructures.Feature`)
        The features added to the base shape of the part's geometry.

    """

    def __init__(self, name=None, frame=None, **kwargs):
        super(CAEPart, self).__init__()
        self.attributes = {"name": name or "Part"}
        self.attributes.update(kwargs)
        self.key = None
        self.frame = frame or Frame.worldXY()

    @property
    def gripping_frame(self):
        """Returns the gripping frame of the part, if available.

        Returns
        -------
        :class:`Frame`
        """

        if 'gripping_frame' in self.attributes.keys():
            return self.attributes['gripping_frame']
    
    @gripping_frame.setter
    def gripping_frame(self, gf):
        """Sets the gripping frame of the part, if available.

        Parameters
        ----------
        :class:`Frame`
        """
        self.attributes.update({'gripping_frame':gf})

    #@property
    def get_gripping_frame(self):
        """Returns the gripping frame of the part, if available.

        Returns
        -------
        :class:`Frame`
        """

        if 'gripping_frame' in self.attributes.keys():
            return self.attributes['gripping_frame']
    
    #@gripping_frame.setter
    def set_gripping_frame(self, gripping_frame):
        """Sets the gripping frame of the part, if available.

        Parameters
        ----------
        :class:`Frame`
        """
        self.attributes.update({'gripping_frame':gripping_frame})


    def transform(self, T):
            """Transforms the element.

            Parameters
            ----------
            T : :class:`Transformation`

            Returns
            -------
            None

            Examples
            --------
            >>> from compas.geometry import Box
            >>> from compas.geometry import Translation
            >>> part = Part.from_shape(Box(Frame.worldXY(), 1, 1, 1))
            >>> part.transform(Translation.from_vector([1, 0, 0]))
            """
            self.frame.transform(T)

            if 'mesh' in self.attributes.keys():
                self.attributes['mesh'].transform(T)

            if 'shape' in self.attributes.keys():
                self.attributes['shape'].transform(T)

            if 'gripping_frame' in self.attributes.keys():
                self.attributes['gripping_frame'].transform(T)

                
    def transformed(self, T):
        """Returns a transformed copy of this part.

        Parameters
        ----------
        T : :class:`Transformation`

        Returns
        -------
        Part

        Examples
        --------
        >>> from compas.geometry import Box
        >>> from compas.geometry import Translation
        >>> part = Part.from_shape(Box(Frame.worldXY(), 1, 1, 1))
        >>> part2 = part.transformed(Translation.from_vector([1, 0, 0]))
        """
        part = self.copy()
        part.transform(T)
        return part
    
    def copy(self):
        """Returns a copy of this part.

        Returns
        -------
        Part
        """
        part = CAEPart(name=self.attributes['name'], frame=self.frame.copy())
        part.key = self.key
        
        if 'mesh' in self.attributes.keys():
            part.attributes.update({'mesh':self.attributes['mesh'].copy()})
        if 'shape' in self.attributes.keys():
            part.attributes.update({'shape':self.attributes['shape'].copy()})
        if 'gripping_frame' in self.attributes.keys():
            part.attributes.update({'gripping_frame':self.attributes['gripping_frame'].copy()})

        return part


