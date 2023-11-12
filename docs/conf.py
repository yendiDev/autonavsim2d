# conf.py

import os
import sys
sys.path.insert(0, os.path.abspath('../src'))  # Adjust the path to your source code

# -- Project information -----------------------------------------------------

project = 'AutoNavSim2D'  # project name
author = 'Clinton Anani'  # name

# -- General configuration ---------------------------------------------------

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
]

# Add any other configuration settings you need.

# -- Options for HTML output -------------------------------------------------

html_theme = 'alabaster'  # Choose a theme that you like

# Add any other HTML configuration settings you need.