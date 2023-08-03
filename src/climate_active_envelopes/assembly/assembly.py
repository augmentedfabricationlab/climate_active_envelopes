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

    # @property
    # def data(self):
    #     """Return a data dictionary of the assembly.
    #     """
    #     # Network data does not recursively serialize to data...
    #     d = self.network.data

    #     # so we need to trigger that for elements stored in nodes
    #     node = {}
    #     for vkey, vdata in d['node'].items():
    #         node[vkey] = {key: vdata[key] for key in vdata.keys() if key != 'element'}
    #         node[vkey]['element'] = vdata['element'].to_data()

    #         if 'frame_est' in vdata:
    #             if node[vkey]['frame_est']:
    #                 node[vkey]['frame_est'] = node[vkey]['frame_est'].to_data()

    #     d['node'] = node

    #     return d
