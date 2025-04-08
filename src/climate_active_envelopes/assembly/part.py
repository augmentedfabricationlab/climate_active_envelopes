from __future__ import print_function
from __future__ import absolute_import
from __future__ import division

from compas.geometry import Frame, Box, Translation, Transformation, bounding_box

from compas.datastructures import Mesh
from compas.datastructures import Datastructure

from compas.geometry import cross_vectors
from compas.geometry import normalize_vector
from compas.geometry import centroid_polyhedron
from compas.geometry import volume_polyhedron

from compas_rhino.geometry import RhinoBrep

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


    @property
    def bottom(self):
        """Identify the *bottom* face of the part's mesh.

        Returns
        -------
        int
            The identifier of the face.

        Notes
        -----
        The face with the lowest centroid is considered the *bottom* face.
        """
        mesh = self.attributes['mesh']

        # Compute the centroids of all faces
        fkey_centroid = {fkey: mesh.face_center(fkey) for fkey in mesh.faces()}

        # Find the face with the lowest z-coordinate
        fkey, _ = sorted(fkey_centroid.items(), key=lambda x: x[1][2])[0]
        return fkey

    @property
    def top(self):
        """Identify the *top* face of the part's mesh.

        Returns
        -------
        int
            The identifier of the face.

        Notes
        -----
        The face with the highest centroid is considered the *top* face.
        """
        mesh = self.attributes['mesh']

        fkey_centroid = {fkey: mesh.face_center(fkey) for fkey in mesh.faces()}
        fkey, _ = sorted(fkey_centroid.items(), key=lambda x: x[1][2])[-1]
        return fkey

    @property
    def center(self):
        """Compute the center of mass of the part's mesh..

        Returns
        -------
        point
            The center of mass of the part's mesh..
        """
        mesh = self.attributes['mesh']

        vertices = [mesh.vertex_coordinates(key) for key in mesh.vertices()]
        faces = [mesh.face_vertices(fkey) for fkey in mesh.faces()]
        return centroid_polyhedron((vertices, faces))

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

    @classmethod
    def from_dimensions(cls, name=None, length=None, width=None, height=None):
        """Construct a part with a box primitive with the given dimensions.

        Parameters
        ----------
        name : str, optional
            The name of the part.
        length : float
            length of the box.
        width : float
            width of the box.
        height : float
            height of the box.
        frame : :class:`Frame`, optional
            The local coordinate system of the part.

        Returns
        -------
        :class:`Part`
            New instance of part.
        """   

        frame = Frame([0., 0., 0.], [1, 0, 0], [0, 1, 0])

        part = cls(name=name, frame=frame)
        part.length = length
        part.height = height
        part.width = width

        box = Box(length, width, height, frame)
        part = cls.from_shape(box, name=name)

        part.frame = Frame(part.center, [1, 0, 0], [0, 1, 0])

        # Store the bottom face
        bottom_face = part.bottom
        part.attributes['bottom_face'] = bottom_face

        top_face_center = part.attributes['mesh'].face_center(part.top)
        part.gripping_frame = Frame(top_face_center, [-1, 0, 0], [0, 1, 0])


        part.attributes['name'] = name

        return part

    @classmethod
    def from_mesh_and_frame(cls, mesh, name=None):
        """Construct an part from a mesh and frame.

        Parameters
        ----------
        mesh : :class:`Mesh`
            Mesh datastructure.
        frame : :class:`Frame`
            Origin frame of the part.
        name : str, optional
            The name of the part.

        Returns
        -------
        :class:`Part`
            New instance of part.
        """
        part = cls(name=name)
        part.attributes.update({'mesh':mesh})

        frame = Frame(part.center, [1, 0, 0], [0, 1, 0])
        part.frame = frame

        top_face_center = mesh.face_center(part.top)
        part.gripping_frame = Frame(top_face_center, [-1, 0, 0], [0, 1, 0])

        return part
    
