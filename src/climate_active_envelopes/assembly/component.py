from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json
import math as m

#from compas_fab.robots import JointTrajectoryPoint

from compas.datastructures import Mesh, CellNetwork
from compas.geometry import Box
from compas.geometry import Frame, Translation, Rotation, Transformation
from compas.datastructures import CellNetwork, Network
from compas_rhino.conversions import mesh_to_compas

class CAEComponent():
    """A data structure for the climate active envelopes' cellnetwork containing the information of building components.

    The reference model is essentially a cellnetwork consisting of a set of cells that contain the information of building component, i.e., outer walls, inner walls, slabs.
    Each building component is represented by a face of the cellnetwork.
    Each interface or connection between components is represented by an edge of the cellnetwork.

    Attributes
    ----------


    Examples
    --------

    """

    def __init__(self, frame=None):
        self.frame = frame #origin frame
        #self.cell_network = CellNetwork()

    @classmethod
    def find_or_add_vertex(cls, cell_network, x, y, z):
        """Find an existing vertex or add a new one to the cell network.

        Parameters
        ----------
        x : float
            X-coordinate of the vertex.
        y : float
            Y-coordinate of the vertex.
        z : float
            Z-coordinate of the vertex.

        Returns
        -------
        int
            The key of the found or added vertex.
        """
        for vkey in cell_network.vertices():
            vx, vy, vz = cell_network.vertex_coordinates(vkey)
            if vx == x and vy == y and vz == z:
                return vkey
        return cell_network.add_vertex(x=x, y=y, z=z)
    
    @classmethod
    def create_cell_network(cls, meshes, cell_network):
        """Create a cell network from a list of meshes.

        Parameters
        ----------
        meshes : list of :class:`Mesh`
            A list of meshes to create the cell network from.

        Returns
        -------
        :class:`CellNetwork`
            The created cell network.
        """

        for mesh in meshes:
            face_keys = []
            for fkey in mesh.faces():
                face_vertices = []
                for vkey in mesh.face_vertices(fkey):
                    x, y, z = mesh.vertex_coordinates(vkey)
                    vertex = cls.find_or_add_vertex(cell_network, x, y, z)
                    face_vertices.append(vertex)
                
                # Check if the face already exists by comparing vertices
                face_exists = False
                for existing_face in cell_network.faces():
                    existing_face_vertices = cell_network.face_vertices(existing_face)
                    if set(face_vertices) == set(existing_face_vertices):
                        face_key = existing_face
                        face_exists = True
                
                if not face_exists:
                    face_key = cell_network.add_face(face_vertices)
                
                face_keys.append(face_key)
            cell_network.add_cell(face_keys)

        return cell_network
    
    @classmethod
    def select_adjacent_faces_by_edge(cls, cell_network, edge_key):
        """Select adjacent faces by the selected edge and sort them by type.

        Parameters
        ----------
        cell_network : :class:`CellNetwork`
            The cell network data structure.
        edge_key : int
            The key of the edge to select.

        Returns
        -------
        dict
            A dictionary with sorted faces and edges by type.
        """
        

        selected_edge, edge_type = cls.select_edge_by_key(cell_network, edge_key)
        selected_edge_faces = cell_network.edge_faces(selected_edge)

        # Add the faces to cell_network attributes if needed
        cell_network.selected_edge_faces = selected_edge_faces

        sorted_faces = {'wall': [], 'slab': []}
        sorted_edges = {'column': [], 'beam': []}

        for face in selected_edge_faces:
            normal = cell_network.face_normal(face)
            if normal[1] in [-1, 1] or normal[0] in [-1, 1]:
                face_type = 'wall'
            elif normal[2] in [-1, 1]:
                face_type = 'slab'
            else:
                continue
            sorted_faces[face_type].append(face)
            cell_network.face_attribute(face, 'face_type', face_type)

        sorted_edges[edge_type].append(selected_edge)

        return sorted_faces, sorted_edges

    @classmethod
    def create_face_cell_dict(cls, cell_network):
        """Sort faces to corresponding cells.

        Parameters:
        ----------
        cell_network : :class:`CellNetwork`
            The cell network data structure.
        
        Returns
        -------
        dict
            A dictionary with faces and corresponding cells.
        """
        #List of the faces in the corresponding cells
        sort_faces_in_cells = []
        for cell in cell_network.cells():
            for face in cell_network.cell_faces(cell):
                sort_faces_in_cells.append((face, cell))

        # Store faces with its corresponding cell
        faces_to_cell_dict = {}
        for face, cell in sort_faces_in_cells:
            if face not in faces_to_cell_dict:
                faces_to_cell_dict[face] = []
            faces_to_cell_dict[face].append(cell)
             
        return faces_to_cell_dict

    @classmethod
    def sort_faces_by_type(cls, cell_network):
        """Sort faces by building type.

        Parameters
        ----------
        cell_network : :class:`CellNetwork`
            The cell network data structure.
        
        Returns
        -------
        list
            A list of faces sorted by type.
        """

        inner_walls = []
        outer_walls = []
        slabs = []

        # Create a dictionary mapping faces to cells
        faces_to_cells_dict = cls.create_face_cell_dict(cell_network)


        # Classify faces as 'outer walls' or 'slabs'
        for face, cell in faces_to_cells_dict.items():
            normal = cell_network.face_normal(face)
            if len(cell) >= 2 and (normal[1] in [-1, 1] or normal[0] in [-1, 1]): #face between minimum two cells 
                face_type = 'inner wall'
                inner_walls.append((face, cell))
                
            elif normal[1] in [-1, 1] or normal[0] in [-1, 1]:  # vertical faces
                face_type = 'outer wall'
                outer_walls.append((face, cell))
            else: #normal[2] in [-1, 1] as horizontal faces
                face_type = 'slab'
                slabs.append((face, cell))

            cell_network.face_attribute(face, 'face_type', face_type)

        all_faces = inner_walls + outer_walls + slabs

        return all_faces

    @classmethod
    def outer_wall_attributes(cls, cell_network, window_curve=None):
        """Assign attributes to outer walls.
        
        Parameters
        ----------
        cell_network : :class:`CellNetwork`
            The cell network data structure.
        window_curve : :class:`Curve`
            The curve representing the window.
        Returns
        -------
        dict
            A dictionary of outer walls with attributes.
        """
        outer_wall_attributes = {}
        all_faces = cell_network.faces()
        for face in all_faces:
            face_type = cell_network.face_attribute(face, 'face_type')
            if face_type == 'outer wall':
                attributes = {}
                if window_curve:
                    attributes['window'] = window_curve
                # if door_curve:
                #     attributes['door'] = door_curve
                if attributes:
                    cell_network.face_attributes(face, attributes)
                outer_wall_attributes[face] = attributes
        return outer_wall_attributes

        # all_faces = cell_network.faces()
        # for face in all_faces:
        #     face_type = cell_network.face_attribute(face, 'face_type')
        #     if face_type == 'outer wall':
        #         if window_curve:
        #             cell_network.face_attribute(face, 'window', window_curve)
        #             #cell_network.face_attribute(face, 'door', door_curve)



    @classmethod
    def select_face_by_fkey(cls, cell_network, face_key):
        """Select the face by key from the cell network.

        Parameters
        ----------
        cell_network : :class:`CellNetwork`
            The cell network data structure.  
        face_key : int
            The key of the face to select.

        Returns
        -------
        object
            The selected face.
        """

        for key, face in enumerate(cell_network.faces()):
            if key == face_key:
                selected_face = face
                face_type = cell_network.face_attribute(face, 'face_type')

        return selected_face, face_type

    @classmethod
    def select_face_neighbors(cls, cell_network, face_key):
        """Select the face neighbors of a face in the cell.
        
        Parameters
        ----------
        cell_network : :class:`CellNetwork`
            The cell network data structure.
            
        Returns
        -------
        dict
            A dictionary of face neighbors.
        """
        current_face, face_type = cls.select_face_by_fkey(cell_network, face_key)
        
        neighbor_face_types = []
        neighbors = []

        # Find all neighboring faces of the current_face through its edges
        for edge in cell_network.face_edges(current_face):
            edge_faces = cell_network.edge_faces(edge)
            for neighbor in edge_faces:
                if neighbor != current_face: # and neighbor not in all_neighbors
                    neighbors.append(neighbor)
                    face_type = cell_network.face_attribute(neighbor, 'face_type')
                    neighbor_face_types.append((neighbor, face_type))

        return neighbors, neighbor_face_types

    @classmethod
    def get_shared_edge(cls, cell_network, current_face, face_neighbors, edge_key):
        """Get the shared edge of two neighboring faces in the cell network.

        Parameters
        ----------
        cell_network : :class:`CellNetwork`
            The cell network data structure.
        edge_key : int
            The key of the edge to select.

        Returns
        -------
        int
            The key and edge type of the shared edge.
        """

        for i, face in enumerate(face_neighbors): #i is the index of the face in the list
            if i < edge_key: #if the face is before the current face
                neighbor_face = face 

        neighbor_face_edges = cell_network.face_edges(neighbor_face)
        current_face_edges = cell_network.face_edges(current_face)

        for u, v in neighbor_face_edges:
            if (u, v) in current_face_edges or (v, u) in current_face_edges:
                shared_edge = (u, v)

                edge_faces = cell_network.edge_faces(shared_edge)
                current_face_normal = cell_network.face_normal(current_face)
                neighbor_face_normal = cell_network.face_normal(neighbor_face)

                ux, uy, uz = cell_network.vertex_coordinates(u)
                vx, vy, vz = cell_network.vertex_coordinates(v)

                if ux == vx and uz != vz:
                    edge_type = 'corner'
                    if current_face_normal == neighbor_face_normal:
                        edge_type = 'outer wall joint'
                    elif len(edge_faces) > 2:
                        edge_type = 'inner wall joint'
                        
                else:
                    edge_type = 'beam'

                cell_network.edge_attribute(shared_edge, 'edge_type', edge_type)
        return shared_edge, edge_type

