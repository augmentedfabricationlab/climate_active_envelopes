from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import json
import math as m

#from compas_fab.robots import JointTrajectoryPoint

from compas.datastructures import CellNetwork
from compas.geometry import Frame, Vector
from compas_rhino.conversions import mesh_to_compas, point_to_rhino, mesh_to_rhino, vector_to_rhino
import Rhino.Geometry as rg

class CAECellNetwork(CellNetwork):
    """A data structure for the climate active envelopes' cellnetwork 
    containing the information of building components.

    The reference model is essentially a cellnetwork consisting of a 
    set of cells that contain the information of building component, 
    i.e., outer walls, inner walls, slabs.
    Each building component is represented by a face of the cellnetwork.
    Each interface or connection between components is represented by an edge of the cellnetwork.

    Attributes
    ----------


    Examples
    --------

    """

    def __init__(self, frame=None):
        self.frame = frame #origin frame
        CellNetwork.__init__(self)

    def initialize_network(self):
        """Initialize the cell network by sorting faces and edges by type."""
        self.sort_faces_by_type(self)
        self.sort_edges_by_type(self)

    def find_or_add_vertex(self, cell_network, x, y, z):
        for vkey in cell_network.vertices():
            vx, vy, vz = cell_network.vertex_coordinates(vkey)
            if vx == x and vy == y and vz == z:
                return vkey
        return cell_network.add_vertex(x=x, y=y, z=z)

    def create_cell_network(self, meshes):
        """Create a cell network from a list of meshes.
        
        Parameters
        ----------
        meshes : list
            A list of meshes to create the cell network.
        
        Returns
        -------
        object
            The cell network data structure.        
        """

        for mesh in meshes:
            face_keys = []
            for fkey in mesh.faces():
                face_vertices = []
                for vkey in mesh.face_vertices(fkey):
                    x, y, z = mesh.vertex_coordinates(vkey)
                    vertex = self.find_or_add_vertex(self, x, y, z) 
                    face_vertices.append(vertex)
                
                face_exists = False
                for existing_face in self.faces(): 
                    existing_face_vertices = self.face_vertices(existing_face) 
                    if set(face_vertices) == set(existing_face_vertices):
                        face_key = existing_face
                        face_exists = True
                
                if not face_exists:
                    face_key = self.add_face(face_vertices) 
                
                face_keys.append(face_key)
            self.add_cell(face_keys) 

        return self 
    
    def create_face_cell_dict(self, cell_network):
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
    
    def sort_faces_by_type(self, cell_network):
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

        face_types = {'walls': [], 'slabs': []}

        # Create a dictionary mapping faces to cells
        faces_to_cells_dict = self.create_face_cell_dict(cell_network)

        # Classify faces as 'outer walls' or 'slabs'
        for face, cell in faces_to_cells_dict.items():
            normal = cell_network.face_normal(face)
            #face between minimum two cells and within the vertical faces
            if len(cell) >= 2 and (normal[1] in [-1, 1] or normal[0] in [-1, 1]): 
                face_type = 'inner wall'
                face_types['walls'].append((face, face_type))

            # within vertical faces and not between two cells    
            elif normal[1] in [-1, 1] or normal[0] in [-1, 1]:  
                face_type = 'outer wall'
                face_types['walls'].append((face, face_type))

            # all horizontal faces
            else: #normal[2] in [-1, 1] as horizontal faces
                face_type = 'slab'
                face_types['slabs'].append((face, face_type))

            cell_network.face_attribute(face, 'face_type', face_type)

        cell_network.face_types = face_types

        return face_types
    

    def get_face_types(self, cell_network):
        """Get the face types from the cell network.

        Parameters
        ----------
        cell_network : :class:`CellNetwork`
            The cell network data structure.
        
        Returns
        -------
        dict
            A dictionary with face types.
        """
        return getattr(cell_network, 'edge_types', {'walls': [], 'slabs': []})
    
    
    def create_edge_cell_dict(self, cell_network):
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
        sort_edges_in_cells = []
        for cell in cell_network.cells():
            cell_edges = []
            for edge in cell_network.cell_edges(cell):
                u, v = edge
                if (u, v) not in cell_edges and (v, u) not in cell_edges:
                    cell_edges.append((u, v))
                    sort_edges_in_cells.append((edge, cell))

        edges_to_cell_dict = {}
        for edge, cell in sort_edges_in_cells:
            if edge not in edges_to_cell_dict and (edge[1], edge[0]) not in edges_to_cell_dict:
                edges_to_cell_dict[edge] = []
            if edge in edges_to_cell_dict:
                edges_to_cell_dict[edge].append(cell)
            elif (edge[1], edge[0]) in edges_to_cell_dict:
                edges_to_cell_dict[(edge[1], edge[0])].append(cell)
             
        return edges_to_cell_dict
    
    def sort_edges_by_type(self, cell_network):
        """Sort edges by type.

        Parameters
        ----------
        cell_network : :class:`CellNetwork`
            The cell network data structure.
        
        Returns
        -------
        list
            A list of edges sorted by type.
        """

        edge_types = {'vertical_edges': [], 'horizontal_edges': []}

        # Create a dictionary mapping faces to cells
        edges_to_cells_dict = self.create_edge_cell_dict(cell_network)

        # Classify faces as 'outer walls' or 'slabs'
        for edge, cell in edges_to_cells_dict.items():
            direction_vector = cell_network.edge_vector(edge)
            direction_vector.unitize()

            if len(cell) >= 2 and (direction_vector[2] in [-1, 1]):
                edge_type = 'joint'
                edge_types['vertical_edges'].append((edge, edge_type))

            elif direction_vector[2] in [-1, 1]:
                edge_type = 'corner'
                edge_types['vertical_edges'].append((edge, edge_type))
            else:
                edge_type = 'beam'
                edge_types['horizontal_edges'].append((edge, edge_type))

            cell_network.edge_attribute(edge, 'edge_type', edge_type)

        cell_network.edge_types = edge_types

        return edge_types
    

    def get_edge_types(self, cell_network):
        """Get the edge types from the cell network.

        Parameters
        ----------
        cell_network : :class:`CellNetwork`
            The cell network data structure.
        
        Returns
        -------
        dict
            A dictionary with edge types.
        """
        return getattr(cell_network, 'edge_types', {'vertical_edges': [], 'horizontal_edges': []})



    def select_face_by_fkey(self, cell_network, face_key):
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
    

    def select_edge_by_ekey(self, cell_network, edge_key):
        """Select the edge by key from the cell network.

        Parameters
        ----------
        cell_network : :class:`CellNetwork`
            The cell network data structure.
        edge_key : int
            The key of the edge to select.

        Returns
        -------
        object
            The selected edge.
        """

        for key, edge in enumerate(cell_network.edges()):
            if key == edge_key:
                selected_edge = edge
                edge_type = cell_network.edge_attribute(edge, 'edge_type')

        return selected_edge, edge_type


    def select_edge_and_get_adjacent_faces(self, cell_network, edge_key):
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
        
        selected_edge, edge_type = self.select_edge_by_ekey(cell_network, edge_key)
        selected_edge_faces = cell_network.edge_faces(selected_edge)

        # Create a dictionary to store the selected edge and its adjacent faces
        edge_and_faces = {
            'selected_edge': selected_edge,
            'edge_type': edge_type,
            'selected_edge_faces': []
        }

        # Add edge_type for each selected_edge_face
        for face in selected_edge_faces:
            face_type = cell_network.face_attribute(face, 'face_type')
            edge_and_faces['selected_edge_faces'].append({
                'face': face,
                'face_type': face_type
            })

        return edge_and_faces


    def select_face_and_get_adjacent_edges(self, cell_network, face_key):
        """Select adjacent edges by the selected face and sort them by type.

        Parameters
        ----------
        cell_network : :class:`CellNetwork`
            The cell network data structure.
        face_key : int
            The key of the face to select.
        
        Returns
        -------
        dict
            A dictionary with sorted edges and faces by type.
        """
        # Select the face by key
        current_face, face_type = self.select_face_by_fkey(cell_network, face_key)

        # Get the edges of the selected face
        face_edges = self.face_edges(current_face)

        edge_types = self.get_edge_types(cell_network)

        face_and_edges = {
            'selected_face': current_face,
            'face_type': face_type,
            #'face_edges': [],
            'vertical_edges': [],
            'horizontal_edges': []
        }

        # Add edge_type for each face_edge and classify them as vertical or horizontal
        for edge in face_edges:
            edge_type = cell_network.edge_attribute(edge, 'edge_type')
            # face_and_edges['face_edges'].append({
            #     'edge': edge,
            #     'edge_type': edge_type
            # })
            if edge_type in ['joint', 'corner']:  # Assuming 'joint' and 'corner' are vertical
                face_and_edges['vertical_edges'].append({
                    'edge': edge,
                    'edge_type': edge_type
                })
            else:  # Assuming other types are horizontal
                face_and_edges['horizontal_edges'].append({
                    'edge': edge,
                    'edge_type': edge_type
                })

        return face_and_edges

    def select_face_neighbors(self, cell_network, face_key):
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
        current_face, face_type = self.select_face_by_fkey(cell_network, face_key)
        
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
    
    def get_shared_edge(self, cell_network, current_face, face_neighbors, edge_key):
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

        for i, face in enumerate(face_neighbors): 
            if i < edge_key: #if the face is before the current face
                neighbor_face = face 

        neighbor_face_edges = cell_network.face_edges(neighbor_face)
        current_face_edges = cell_network.face_edges(current_face)

        for u, v in neighbor_face_edges:
            if (u, v) in current_face_edges or (v, u) in current_face_edges:
                shared_edge = (u, v)

                edge_type = cell_network.edge_attribute(shared_edge, 'edge_type')

        return shared_edge, edge_type


    def outer_wall_attributes(self, cell_network, window_curve=None):
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

        pass
    
    
    def generate_assembly_data_from_cellnetwork(self, cell_network, course_height):
        """Calculate the properties of the wall faces in the cell network.

        Parameters
        ----------
        cell_network : :class:`CellNetwork`
            The cell network data structure.
        face : int
            The current_face to select.
        course_height : float
            The height of the course.
        
        Returns
        -------
        dict
            A dictionary of wall face edges.
        """
        face = cell_network.current_face
        face_normal = vector_to_rhino(cell_network.face_normal(face))

        face_edges_data = self.select_face_and_get_adjacent_edges(cell_network, face)
        
        # Get the height of the vertical edge
        vertical_edges = [edge['edge'] for edge in face_edges_data['vertical_edges']]
        edge_height = abs(cell_network.edge_length(vertical_edges[0]))

        # Identify the starting and ending edges
        start_edge = vertical_edges[-1]
        end_edge = vertical_edges[0]
        start_edge_type = cell_network.edge_attribute(start_edge, 'edge_type')
        end_edge_type = cell_network.edge_attribute(end_edge, 'edge_type')

        # Get the length of the horizontal edge
        horizontal_edges = [edge['edge'] for edge in face_edges_data['horizontal_edges']]
        edge_length = abs(cell_network.edge_length(horizontal_edges[0]))

        # Get the direction vector of the first horizontal edge
        horizontal_edge_vector = cell_network.edge_vector(horizontal_edges[0])
        horizontal_edge_vector.unitize()

        # Handle different orientations explicitly
        direction_vector = vector_to_rhino(horizontal_edge_vector)

        curve_start_point = cell_network.edge_start(horizontal_edges[0])
        curve_end_point = cell_network.edge_end(horizontal_edges[0])

        print(f"Initial curve_start_point: {edge_length}")
        print(f"Initial curve_end_point: {edge_height}")

        num_courses = abs(edge_height / course_height)

        # Add starting and ending edges and their types as attributes to the current face
        cell_network.face_attribute(face, 'start_edge', start_edge)
        cell_network.face_attribute(face, 'start_edge_type', start_edge_type)
        cell_network.face_attribute(face, 'end_edge', end_edge)
        cell_network.face_attribute(face, 'end_edge_type', end_edge_type)
        cell_network.face_attribute(face, 'direction_vector', direction_vector)

        return {
            'face': face_edges_data['selected_face'],
            'face_type': face_edges_data['face_type'],
            'start_edge': start_edge,
            'start_edge_type': start_edge_type,
            'end_edge': end_edge,
            'end_edge_type': end_edge_type,
            'direction_vector': direction_vector,
            'edge_height': edge_height,
            'edge_length': edge_length,
            'num_courses': num_courses,
            'curve_start_point': curve_start_point,
            'curve_end_point': curve_end_point,
            'face_normal': face_normal
        } 
    
    def generate_assembly_data_from_mesh_face(self, mesh, face_key, course_height):
        """Calculate the properties of the wall faces in the COMPAS mesh.

        Parameters
        ----------
        mesh : :class:`Mesh`
            The COMPAS mesh data structure.
        face_key : int
            The key of the face to select.
        course_height : float
            The height of the course.
        
        Returns
        -------
        dict
            A dictionary of wall face edges.
        """
        face = face_key
        face_normal = vector_to_rhino(mesh.face_normal(face))

        face_edges = mesh.face_halfedges(face)
        
        # Get the height of the vertical edge
        vertical_edges = [edge for edge in face_edges if abs(mesh.edge_vector(*edge)[2]) > 0]
        edge_height = abs(mesh.edge_length(*vertical_edges[0]))

        # Identify the starting and ending edges
        start_edge = vertical_edges[-1]
        end_edge = vertical_edges[0]
        start_edge_type = mesh.edge_attribute(start_edge, 'edge_type')
        end_edge_type = mesh.edge_attribute(end_edge, 'edge_type')

        # Get the length of the horizontal edge
        horizontal_edges = [edge for edge in face_edges if abs(mesh.edge_vector(*edge)[2]) == 0]
        edge_length = abs(mesh.edge_length(*horizontal_edges[0]))

        # Get the direction vector of the first horizontal edge
        horizontal_edge_vector = mesh.edge_vector(*horizontal_edges[0])
        horizontal_edge_vector.unitize()

        # Handle different orientations explicitly
        direction_vector = vector_to_rhino(horizontal_edge_vector)

        curve_start_point = mesh.vertex_coordinates(horizontal_edges[0][0])
        curve_end_point = mesh.vertex_coordinates(horizontal_edges[0][1])

        print(f"Initial curve_start_point: {edge_length}")
        print(f"Initial curve_end_point: {edge_height}")

        num_courses = abs(edge_height / course_height)

        # Add starting and ending edges and their types as attributes to the current face
        mesh.face_attribute(face, 'start_edge', start_edge)
        mesh.face_attribute(face, 'start_edge_type', start_edge_type)
        mesh.face_attribute(face, 'end_edge', end_edge)
        mesh.face_attribute(face, 'end_edge_type', end_edge_type)
        mesh.face_attribute(face, 'direction_vector', direction_vector)

        return {
            'face': face,
            'face_type': mesh.face_attribute(face, 'face_type'),
            'start_edge': start_edge,
            'start_edge_type': start_edge_type,
            'end_edge': end_edge,
            'end_edge_type': end_edge_type,
            'direction_vector': direction_vector,
            'edge_height': edge_height,
            'edge_length': edge_length,
            'num_courses': num_courses,
            'curve_start_point': curve_start_point,
            'curve_end_point': curve_end_point,
            'face_normal': face_normal
        }