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
        #super(CAEComponent, self).__init__()
        self.frame = frame #origin frame

    
    # @classmethod
    # def from_shape(cls, shape, frame=None):
    #     """Construct a component from a shape primitive.

    #     Parameters
    #     ----------
    #     shape : :class:`compas.geometry.Shape`
    #         Shape primitive describing the component.
    #     frame : :class:`Frame`
    #         Origin frame of the component.
    #     Returns
    #     -------
    #     :class:`Component`
    #         New instance of the component.
    #     """
    #     vertex = shape.vertex[0]
    #     frame = Frame([vertex['x'], vertex['y'], vertex['z']], [1, 0, 0], [0, 1, 0])
    #     component = cls(frame)
    #     component._source = shape
    #     component._mesh = Mesh.from_shape(component._source)
    #     component.vertices, component.faces = component._mesh.to_vertices_and_faces()  
    #     #component.faces = [sorted(face) for face in component.faces]

    #     # Extract boundary edges
    #     boundary_edges = set()
    #     for face in component.faces:
    #         for i in range(len(face)):
    #             u = face[i]
    #             v = face[(i + 1) % len(face)]
    #             edge = tuple(sorted((u, v)))
    #             boundary_edges.add(edge)

    #     component.boundary_edges = list(boundary_edges)

    #     return component
    

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
    def select_edge_by_key(cls, cell_network, edge_key):
        """Select an edge by its key and assign a building type.

        Parameters
        ----------
        cell_network : :class:`CellNetwork`
            The cell network data structure.
        edge_key : int
            The key of the edge to select.

        Returns
        -------
        tuple
            The selected edge (e.g. (1,3)) and its type ('column' or 'beam').
        """

        for key, edge in enumerate(cell_network.edges()):
            if key == edge_key:
                selected_edge = edge
        
        # Add the selected edge to the cell_network
        cell_network.selected_edge = selected_edge

        # building component type of the selected edge
        u, v = selected_edge
        ux, uy, uz = cell_network.vertex_coordinates(u)
        vx, vy, vz = cell_network.vertex_coordinates(v)

        if ux == vx and uz == vz:
            edge_type = 'column' if uy != vy else 'beam'
        else:
            edge_type = 'beam'

        cell_network.edge_attribute(selected_edge, 'edge_type', edge_type)
        print(f"Selected edge is a {edge_type}  for key {edge_key}: {selected_edge}")
        return selected_edge, edge_type


    # @classmethod
    # def select_face_by_key(cls, cell_network, face_key):
    #     """Determine the building component type of the selected face.

    #     Parameters
    #     ----------
    #     cell_network : :class:`CellNetwork`
    #         The cell network data structure.  
    #     face_key : int
    #         The key of the face to select.

    #     Returns
    #     -------
    #     tuple
    #         The selected face (e.g. 4) and its type ('wall' or 'slab').
    #     """

    #     for key, face in enumerate(cell_network.faces()):
    #         if key == face_key:
    #             selected_face = face
    #     # Add the selected face to the cell_network
    #     cell_network.selected_face = selected_face

    #     normal = cell_network.face_normal(cell_network.selected_face)
    #     if normal[1] in [-1, 1] or normal[0] in [-1,1]:
    #         face_type = 'wall'
    #     elif normal[2] in [-1, 1]:
    #         face_type = 'slab'
    #     else:
    #         face_type = 'other'

    #     cell_network.face_attribute(selected_face, 'face_type', face_type)
    #     print(f"Selected face is a {face_type} for key {face_key}: {selected_face}")

    #     return selected_face, face_type
    
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
        #print(f"Selected edge and adjacent faces for key {edge_key} : {selected_edge_faces}")

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

        # Create a dictionary to group faces by their vertices
        face_vertices_dict = {}
        for face in cell_network.faces():
            vertices = tuple(sorted(cell_network.face_vertices(face)))
            if vertices not in face_vertices_dict:
                face_vertices_dict[vertices] = []
            face_vertices_dict[vertices].append(face)

        # Classify faces with the same vertices as 'inner walls'
        for faces in face_vertices_dict.values():
            if len(faces) > 1:
                for face in faces:
                    cell = faces_to_cells_dict[face]
                    face_type = 'inner wall'
                    inner_walls.append((face, cell))
                    cell_network.face_attribute(face, 'face_type', face_type)

        # Classify faces as 'outer walls' or 'slabs'
        for face, cell in faces_to_cells_dict.items():
            if (face, cell) not in inner_walls:
                normal = cell_network.face_normal(face)
                if normal[1] in [-1, 1] or normal[0] in [-1, 1]:  # vertical faces
                    face_type = 'outer wall'
                    outer_walls.append((face, cell))
                else: #normal[2] in [-1, 1] as horizontal faces
                    face_type = 'slab'
                    slabs.append((face, cell))

                cell_network.face_attribute(face, 'face_type', face_type)

        all_faces = inner_walls + outer_walls + slabs
        return all_faces

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
        selected_face = None
        face_type = None

        for key, face in enumerate(cell_network.faces()):
            if key == face_key:
                selected_face = face
                face_type = cell_network.face_attribute(face, 'face_type')
                break  # Exit the loop once the face is found

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
        neighbor_face_types = []

        for cell in cell_network.cells():
            valid_faces = []
            for key, face in enumerate(cell_network.faces()):
                if key == face_key:
                    if face in cell_network.cell_faces(cell):
                        valid_faces.append(face)

            for face in valid_faces:
                neighbors = cell_network.cell_face_neighbors(cell, face)
                for neighbor in neighbors:
                    face_type = cell_network.face_attribute(neighbor, 'face_type')
                    neighbor_face_types.append((neighbor, face_type))

        return neighbors, neighbor_face_types