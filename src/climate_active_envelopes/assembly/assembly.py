from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from assembly_information_model.assembly import Assembly


from climate_active_envelopes.export import Exporter
from .element import CAEElement

class CAEAssembly(Assembly):
    """A data structure for discrete element assemblies for human robot collaboration
    """

    def __init__(self,
                 elements=None,
                 attributes=None,
                 default_element_attribute=None,
                 default_connection_attributes=None):

        super(CAEAssembly, self).__init__()