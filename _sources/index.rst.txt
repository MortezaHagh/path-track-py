.. s documentation master file, created by
   sphinx-quickstart on Fri Jun 23 11:55:54 2023.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Path Tracking documentation!
========================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   scripts.tracking
   scripts.controllers
   scripts.pt_scripts


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


Doc
==================

sphinx-quickstart --ext-autodoc

sphinx-apidoc -o docs scripts

make html