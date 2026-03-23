# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'finelpy'
copyright = '2026, Heitor Lopes'
author = 'Heitor Lopes'
release = '0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

import os
import sys
sys.path.insert(0,os.path.abspath("../../src"))

import finelpy
print("Loaded:", finelpy)

project = "finelpy"

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "breathe",
]

templates_path = ['_templates']
exclude_patterns = []

breathe_projects = {
    "finelpy": "../build/doxygen/xml"
}


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
