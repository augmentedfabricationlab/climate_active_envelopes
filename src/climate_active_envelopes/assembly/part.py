from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.geometry import Frame
from compas.geometry import Box
from compas.geometry import Transformation
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
    part_name : str, optional
        The part_name of the part.
        The part_name will be stored in :attr:`Part.attributes`.
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

    def __init__(self, part_name=None, frame=None, **kwargs):
        super(CAEPart, self).__init__()

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
        part = CAEPart(part_name=self.attributes['part_name'], frame=self.frame.copy())
        part.key = self.key
        
        if 'mesh' in self.attributes.keys():
            part.attributes.update({'mesh':self.attributes['mesh'].copy()})
        if 'shape' in self.attributes.keys():
            part.attributes.update({'shape':self.attributes['shape'].copy()})
        if 'gripping_frame' in self.attributes.keys():
            part.attributes.update({'gripping_frame':self.attributes['gripping_frame'].copy()})

        return part

    @classmethod
    def from_dimensions(cls, part_name=None, length=None, width=None, height=None):
        """Construct a part with a box primitive with the given dimensions.

        Parameters
        ----------
        length : float
            length of the box.
        width : float
            width of the box.
        height : float
            height of the box.
        Returns
        -------
        :class:`Part`
            New instance of part.
        """
        frame = Frame([0., 0., height/2], [1, 0, 0], [0, 1, 0])
        part = cls(part_name, frame)
        part.length = length
        part.height = height
        part.width = width
     
        box = Box(length, width, height, frame)
        part._source = box
        return cls.from_shape(box)
    
    @classmethod
    def from_mesh_and_frame(cls, mesh, part_name=None):
        """Construct an part from a mesh and frame.

        Parameters
        ----------
        mesh : :class:`Mesh`
            Mesh datastructure.
        frame : :class:`Frame`
            Origin frame of the part.
        new_frame : :class:`Frame`
            New frame of the part.

        Returns
        -------
        :class:`Part`
            New instance of part.
        """
        t_frame = Frame.worldXY()
        frame = Frame(mesh.centroid(),[1, 0, 0], [0, 1, 0])
        part = cls(part_name, frame)

        T = Transformation.from_frame_to_frame(part.frame, t_frame)
        mesh_transformed = mesh.transformed(T)
        frame = Frame(mesh_transformed.centroid(),[1, 0, 0], [0, 1, 0])
        part.frame = t_frame
        part.attributes.update({'mesh':mesh})
        #part._source = part._mesh = mesh_transformed
        return part