# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information
import os
import sys
basedir = os.path.abspath('..')
#basedir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'ROS_Stack'))
sys.path.insert(0, basedir)
print(sys.executable)
project = 'Jetracer Speedway'
copyright = '2023, Marcos Fernández'
author = 'Marcos Fernández'
release = '1.0.0'
master_doc = 'index'
# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
            'myst_parser',
            'sphinx.ext.todo',
            'sphinx.ext.viewcode', 
            'sphinx.ext.autodoc',
            'sphinx_copybutton',
            ]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

autodoc_mock_imports = [
                        "utils",
                        "pyrealsense2"
]

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

#html_theme = 'sphinx_rtd_theme'
html_theme = 'furo'
html_static_path = ['_static']
