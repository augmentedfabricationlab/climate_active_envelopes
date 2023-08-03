"""

Intro to project ...


Setup
=====

In order to use this library, ...


Main concepts
=============

Describe typical classes found in project

.. autoclass:: SampleClassName
   :members:


"""

from .sample_module import SampleClassName
from .assembly import wall_assembly, brick_assembly, CAEElement
from .export import Exporter

__all__ = ['SampleClassName', 'wall_assembly', 'brick_assembly', 'CAEElement', 'Exporter']
