from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json
import os
from copy import deepcopy
from compas.datastructures import CellNetwork, Network


class CAECellNetwork(CellNetwork):
    """A data structure for the climate active envelopes' reference model

    The reference model is essentially a cellnetwork consisting of a set of cells that contain the information of building component, i.e., outer walls, inner walls, slabs.
    Each building component is represented by a face of the cellnetwork.
    Each interface or connection between components is represented by an edge of the cellnetwork.

    Attributes
    ----------
    network : :class:`compas.Network`, optional
    components : list of :class:`Component`, optional
        A list of assembly components.
    attributes : dict, optional
        User-defined attributes of the assembly.
        Built-in attributes are:
        * name (str) : ``'Assembly'``
    default_component_attribute : dict, optional
        User-defined default attributes of the components of the assembly.
        The built-in attributes are:
        * is_planned (bool) : ``False``
        * is_placed (bool) : ``False``
    default_connection_attributes : dict, optional
        User-defined default attributes of the connections of the assembly.

    Examples
    --------
    >>> assembly = Assembly()
    >>> for i in range(2):
    >>>     component = Component.from_box(Box(Frame.worldXY(), 10, 5, 2))
    >>>     assembly.add_element(element)
    """
    def __init__(self,
                 elements=None,
                 attributes=None,
                 default_element_attributes=None,
                 default_connection_attributes=None):

        self.network = Network()
        self.cellnetwork = CellNetwork()
        self.network.attributes.update({'name': 'Assembly'})
        self.cellnetwork.attributes.update({'name': 'Assembly'})

        if attributes is not None:
            self.network.attributes.update(attributes)
            self.cellnetwork.attributes.update(attributes)
        
        self.network.default_node_attributes.update({'is_planned': False,'is_placed': False})
        self.cellnetwork.default_face_attributes.update({'is_planned': False,'is_placed': False}) #TODO: is that correct?

        if default_element_attributes is not None:
            self.network.default_node_attributes.update(default_element_attributes)
            self.cellnetwork.default_face_attributes.update(default_element_attributes)

        if default_connection_attributes is not None:
            self.network.default_edge_attributes.update(default_connection_attributes)
            self.cellnetwork.default_edge_attributes.update(default_connection_attributes)
        
        if default_connection_attributes is not None:
            self.cellnetwork.default_vertex_attributes.update(default_connection_attributes)

        if elements:
            for element in elements:
                self.add_element(element)

    def add_element(self, element, key=None, attr_dict={}, **kwattr):
        """Add an element to the assembly.

        Parameters
        ----------
        element : Element
            The element to add.
        attr_dict : dict, optional
            A dictionary of element attributes. Default is ``None``.

        Returns
        -------
        hashable
            The identifier of the element.
        """
        attr_dict.update(kwattr)
        x, y, z = element.frame.point
        key = self.network.add_node(key=key, attr_dict=attr_dict,
                                    x=x, y=y, z=z, element=element)
        return key
    

    def add_connection(self, u, v, attr_dict=None, **kwattr):
        """Add a connection between two elements and specify its attributes.

        Parameters
        ----------
        u : hashable
            The identifier of the first element of the connection.
        v : hashable
            The identifier of the second element of the connection.
        attr_dict : dict, optional
            A dictionary of connection attributes.
        kwattr
            Other connection attributes as additional keyword arguments.

        Returns
        -------
        tuple
            The identifiers of the elements.
        """

        if u not in self.cellnetwork.vertices():
            self.cellnetwork.add_vertex(u)
        if v not in self.cellnetwork.vertices():
            self.cellnetwork.add_vertex(v)
        return self.cellnetwork.add_edge(u, v, attr_dict, **kwattr)

        #return self.cellnetwork.add_edge(u, v, attr_dict, **kwattr)

    def copy(self):
        """Returns a copy of this assembly.
        """
        cls = type(self)
        return cls.from_data(deepcopy(self.data))

    def element(self, key, data=False):
        """Get an element by its key."""
        if data:
            return self.network.node[key]['element'], self.network.node[key]
        else:
            return self.network.node[key]['element']

    def elements(self, data=False):
        """Iterate over the elements of the assembly.

        Parameters
        ----------
        data : bool, optional
            If ``True``, yield both the identifier and the attributes.

        Yields
        ------
        2-tuple
            The next element as a (key, element) tuple, if ``data`` is ``False``.
        3-tuple
            The next element as a (key, element, attr) tuple, if ``data`` is ``True``.

        """
        if data:
            for vkey, vattr in self.network.nodes(True):
                yield vkey, vattr['element'], vattr
            for fkey, fattr in self.cellnetwork.faces(True):  
                yield fkey, fattr['element'], fattr

        else:
            for vkey in self.network.nodes(data):
                yield vkey, self.network.node[vkey]['element']
            for fkey, fattr in self.cellnetwork.faces(data):  
                yield fkey, fattr['element']


    def add_connection(self, u, v, attr_dict=None, **kwattr):
        """Add a connection between two elements and specify its attributes.

        Parameters
        ----------
        u : hashable
            The identifier of the first element of the connection.
        v : hashable
            The identifier of the second element of the connection.
        attr_dict : dict, optional
            A dictionary of connection attributes.
        kwattr
            Other connection attributes as additional keyword arguments.

        Returns
        -------
        tuple
            The identifiers of the elements.
        """
        return self.cellnetwork.add_edge(u, v, attr_dict, **kwattr)


    def connections(self, data=False):
        """Iterate over the connections of the network.

        Parameters
        ----------
        data : bool, optional
            If ``True``, yield both the identifier and the attributes.

        Yields
        ------
        2-tuple
            The next connection identifier (u, v), if ``data`` is ``False``.
        3-tuple
            The next connection as a (u, v, attr) tuple, if ``data`` is ``True``.

        """
        return self.cellnetwork.edges(data)