from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from compas.datastructures import Datastructure
from compas.datastructures import Graph
from compas.datastructures import AssemblyError
from compas.geometry import Frame, Translation, Rotation, Transformation, Vector, Point, normalize_vector
from compas_rhino.conversions import plane_to_compas_frame, point_to_compas, mesh_to_rhino, point_to_rhino, vector_to_rhino

from collections import deque

__all__ = []    



def add_part(self, part, key=None, attr_dict=None, **kwargs):
    """Add a part to the assembly.

    Parameters
    ----------
    part : :class:`compas.datastructures.Part`
        The part to add.
    key : int | str, optional
        The identifier of the part in the assembly.
        Note that the key is unique only in the context of the current assembly.
        Nested assemblies may have the same `key` value for one of their parts.
        Default is None in which case the key will be an automatically assigned integer value.
    **kwargs: dict[str, Any], optional
        Additional named parameters collected in a dict.

    Returns
    -------
    int | str
        The identifier of the part in the current assembly graph.

    """
    if part.guid in self._parts:
        raise AssemblyError("Part already added to the assembly")
    
    key = self.graph.add_node(key=key, part=part, x=part.frame.point.x, y=part.frame.point.y, z=part.frame.point.z, **kwargs)
    part.key = key
    self._parts[part.guid] = part.key

    if attr_dict:
        for attr, value in attr_dict.items():
            part.attributes[attr] = value
            #self.graph.node_attribute(key, attr, value)

    return key

def add_connection(self, a_key, b_key, **kwargs):
    """Add a connection between two parts.

    Parameters
    ----------
    a_key : int | str
        The identifier of the "from" part.
    b_key : int | str
        The identifier of the "to" part.
    **kwargs : dict[str, Any], optional
        Attribute dict compiled from named arguments.

    Returns
    -------
    tuple[int | str, int | str]
        The tuple of node identifiers that identifies the connection.

    Raises
    ------
    :class:`AssemblyError`
        If `a_key` and/or `b_key` are not in the assembly.

    """
    error_msg = "Both parts have to be added to the assembly before a connection can be created."
    if not self.graph.has_node(a_key) or not self.graph.has_node(b_key):
        raise AssemblyError(error_msg)
    #print(f"Adding connection between {a_key} and {b_key}")
    return self.graph.add_edge(a_key, b_key, **kwargs)

def assembly_courses_xy(self, tol=0.01):
    """Identify the courses in a wall of bricks.

    Parameters
    ----------
    tol : float, optional
        Tolerance for identifying courses.

    Examples
    --------
    .. code-block:: python

        pass

    """
    courses = []

    # all element keys
    elements = set(self.graph.nodes())

    # base course keys
    c_min = min(self.graph.nodes_attribute('z'))
    #base = set(assembly.network.nodes_where({'z': c_min}))

    base = set()
    for e in elements:
        z = self.graph.node_attribute(key=e, name='z')
        if (z - c_min) ** 2 < tol:
            base.add(e)
    # print(base)

    if base:
        courses.append(list(base))

        elements -= base

        while elements:  # and counter<1000:

            c_min = min([self.graph.node_attribute(key=key, name='z') for key in elements])
            # print(c_min)
            #base = set(assembly.network.nodes_where({'z': c_min}))
            # print(base)
            base = set()
            for e in elements:
                z = self.graph.node_attribute(key=e, name='z')
                if (z - c_min) ** 2 < tol:
                    base.add(e)

            courses.append(list(base))
            elements -= base

    # assign course id's to the corresponding blocks
    for i, course in enumerate(courses):
        self.graph.nodes_attribute(name='course', value=i, keys=course)

    # Sort nodes within each course by proximity
    for i, course in enumerate(courses):
        sorted_course = []
        remaining_nodes = set(course)
        current_node = remaining_nodes.pop()
        sorted_course.append(current_node)

        while remaining_nodes:
            next_node = min(remaining_nodes, key=lambda node: self.distance_xy(current_node, node))
            sorted_course.append(next_node)
            remaining_nodes.remove(next_node)
            current_node = next_node

        courses[i] = sorted_course

        # Add connections between nodes within the same course
        for j in range(len(sorted_course) - 1):
            self.add_connection(sorted_course[j], sorted_course[j + 1])


    # # Add connections between courses
    # for i in range(1, len(courses)):
    #     current_course = courses[i]
    #     previous_course = courses[i - 1]
    #     for node in current_course:
    #         closest_node = min(previous_course, key=lambda n: self.distance_xy(node, n))
    #         self.add_connection(node, closest_node)


    return courses

def distance_xy(self, node1, node2):
    """Calculate the distance between two nodes in the x and y directions."""
    x1, y1, _ = self.graph.node_attributes(node1, ['x', 'y', 'z'])
    x2, y2, _ = self.graph.node_attributes(node2, ['x', 'y', 'z'])
    return abs(x1 - x2) + abs(y1 - y2)
    #((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

def assembly_courses_xyz(self, tol=0.01, z_tol=0.04):

    """Identify the courses in a wall of bricks.

    Parameters
    ----------
    tol : float, optional
        Tolerance for identifying courses.

    Examples
    --------
    .. code-block:: python

        pass

    """

    courses = []

    # all element keys
    elements = set(self.graph.nodes())

    # base course keys
    c_min = min(self.graph.nodes_attribute('z'))

    base = set()
    for e in elements:
        z = self.graph.node_attribute(key=e, name='z')
        if abs(z - c_min) < tol:
            base.add(e)

    if base:
        courses.append(list(base))
        elements -= base

        while elements:
            c_min = min([self.graph.node_attribute(key=key, name='z') for key in elements])
            base = set()
            for e in elements:
                z = self.graph.node_attribute(key=e, name='z')
                if abs(z - c_min) < tol:
                    base.add(e)

            if base:
                courses.append(list(base))
                elements -= base

    # Sort courses by their minimum z value
    courses.sort(key=lambda course: min(self.graph.node_attribute(key, 'z') for key in course))

    # assign course id's to the corresponding blocks
    for i, course in enumerate(courses):
        self.graph.nodes_attribute(name='course', value=i, keys=course)

    # Sort nodes within each course by proximity using graph.neighbors
    for i, course in enumerate(courses):
        sorted_course = []
        remaining_nodes = set(course)
        current_node = remaining_nodes.pop()
        sorted_course.append(current_node)

        while remaining_nodes:
            neighbors = set(self.graph.neighbors(current_node))
            next_node = neighbors.intersection(remaining_nodes)
            if next_node:
                next_node = next_node.pop()
                sorted_course.append(next_node)
                remaining_nodes.remove(next_node)
                current_node = next_node
            else:
                # If no direct neighbor is found, pick the closest remaining node
                closest_node = min(remaining_nodes, key=lambda node: self.distance_xy(current_node, node))
                sorted_course.append(closest_node)
                remaining_nodes.remove(closest_node)
                current_node = closest_node

        courses[i] = sorted_course

        # Add connections between each node and its next closest neighbor in either x or y axis
        for j in range(len(sorted_course) - 1):
            node = sorted_course[j]
            next_node = min(
                sorted_course[j + 1:],
                key=lambda n: self.distance_xy(node, n)
            )
            self.add_connection(node, next_node)

    # Add connections to the closest neighbor in the course below
    for i in range(1, len(courses)):
        current_course = courses[i]
        below_course = courses[i - 1]
        connected_nodes = set()
        for node in current_course:
            z_node = self.graph.node_attribute(key=node, name='z')
            x_node, y_node = self.graph.node_attributes(node, ['x', 'y'])
            available_neighbors = [
                n for n in below_course if n not in connected_nodes and
                abs(self.graph.node_attribute(key=n, name='x') - x_node) < z_tol and
                abs(self.graph.node_attribute(key=n, name='y') - y_node) < z_tol
            ]
            if available_neighbors:
                closest_neighbor = min(
                    available_neighbors,
                    key=lambda n: abs(z_node - self.graph.node_attribute(key=n, name='z'))
                )
                self.add_connection(node, closest_neighbor)
                connected_nodes.add(closest_neighbor)

    return courses
    
def assembly_courses(self, tol=0.001):
    """Identify the courses in a wall of bricks.

    Parameters
    ----------
    wall : Assembly
        The wall assembly data structure.

    Examples
    --------
    .. code-block:: python

        pass

    """
    courses = []

    # all part keys
    parts = set(self.graph.nodes())

    # base course keys
    c_min = min(self.graph.nodes_attribute('z'))

    base = set()
    for e in parts:
        z = self.graph.node_attribute(key=e, name='z')
        if (z - c_min) ** 2 < tol:
            base.add(e)

    if base:
        courses.append(list(base))
        parts -= base
        while parts:  # and counter<1000:
            c_min = min([self.graph.node_attribute(key=key, name='z') for key in parts])
            base = set()
            for p in parts:
                z = self.graph.node_attribute(key=p, name='z')
                if (z - c_min) ** 2 < tol:
                    base.add(p)
            courses.append(list(base))
            parts -= base

    # assign course id's to the corresponding blocks
    for i, course in enumerate(courses):
        self.graph.nodes_attribute(name='course', value=i, keys=course)

    return courses   

def assembly_building_sequence(self, key):
    """Determine the sequence of bricks that need to be assembled to be able to
    place a target brick.

    Parameters
    ----------
    assembly : Assembly
        An assembly data structure.
    key : hashable
        The block identifier.

    Returns
    -------
    list
        A sequence of block identifiers.
    Notes
    -----
    This will only work for properly supported *wall* assemblies of which the
    interfaces and courses have been identified.

    Examples
    --------
    .. code-block:: python

        # this code only works in Rhino

        assembly = Assembly.from_json(...)

        placed = list(assembly.nodes_where({'is_placed': True}))

        artist = AssemblyArtist(assembly, layer="Assembly")

        artist.clear_layer()
        artist.draw_nodes()
        artist.draw_blocks(show_faces=False, show_edges=True)

        if placed:
            artist.draw_blocks(keys=placed, show_faces=True, show_edges=False)

        artist.redraw()

        key = AssemblyHelper.select_node(assembly)

        sequence = assembly_block_building_sequence(assembly, key)

        print(sequence)

        keys = list(set(sequence) - set(placed))

        artist.draw_blocks(keys=keys, show_faces=True, show_edges=False)
        artist.redraw()

    """

    course = self.graph.node_attribute(key, 'course')

    if course is None:
        raise Exception("The courses of the assembly have not been identified.")

    sequence = []
    seen = set()
    tovisit = deque([(key, course + 1)])

    while tovisit:
        k, course_above = tovisit.popleft()

        if k not in seen:
            seen.add(k)
            course = self.graph.node_attribute(k, 'course')

            if course_above == course + 1:
                sequence.append(k)
                for nbr in self.graph.neighbors(k):
                    if nbr not in seen:
                        tovisit.append((nbr, course))

    for i in range(len(sequence) - 1):
        self.add_connection(sequence[i], sequence[i + 1])

    for i in range(len(sequence)):
        current_node = sequence[i]
        current_z = self.graph.node_attribute(current_node, 'z')
        for j in range(i + 1, len(sequence)):
            next_node = sequence[j]
            next_z = self.graph.node_attribute(next_node, 'z')
            if next_z > current_z:
                self.add_connection(current_node, next_node)
                break

    return sequence[::-1]