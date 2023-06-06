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



def setup(app):
    app.add_css_file('css/custom.css')


source_suffix = ['.rst', '.md']

templates_path = ['_templates']

exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

master_doc = 'index'


project = 'Jetracer Speedway'
copyright = '2023, Marcos Fernández'
author = 'Marcos Fernández'
release = '1.0.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
            'myst_parser',
            'sphinx.ext.todo',
            'sphinx.ext.viewcode', 
            'sphinx.ext.autodoc',
            'sphinx_copybutton',
            "sphinx.ext.intersphinx",
            ]


autodoc_mock_imports = [
                        "jetracer_speedway_srvs",
                        "std_msgs",
                        "geometry_msgs",
                        "cv_bridge",
                        'jetracer_speedway_msgs',
                        'sensor_msgs',
                        "rospy",
                        "utils",
                        "pyrealsense2",
                        "README.md"
]

intersphinx_mapping = {
    "sphinx": ("https://www.sphinx-doc.org/en/master", None),
}




# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

#html_theme = 'sphinx_rtd_theme'
html_theme = 'furo'


html_static_path = ['_static']


html_title = "JetRacer Speedway"


html_css_files = [
      "https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/fontawesome.min.css",
      "https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/solid.min.css",
      "https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/brands.min.css",
]


html_theme_options = {

    "announcement": "🚀 🏎️<em>New Release JetRacer Autonomous Driving!</em>🏎️ 🚀",

    "sidebar_hide_name": False,

    "light_css_variables": {
        "color-brand-primary": "#140062",
        "color-brand-content": "#7C4DFF",
    },

    "footer_icons": [
        {
            "name": "GitHub",
            "url": "https://github.com/marqinhos/JetRacer_Autonomous_Driving",
            "html": "",
            "class": "fa-brands fa-solid fa-github fa-2x",
        },
    ],

    "navigation_with_keys": True,


}


html_sidebars = {
    "**": [
        "sidebar/scroll-start.html",
        "sidebar/brand.html",
        "sidebar/search.html",
        "sidebar/navigation.html",
        "sidebar/ethical-ads.html",
        "sidebar/scroll-end.html",
    ]
}


templates = {
    'module': 'template_api.md',
}








"""
htmlhelp_basename = "Datasettedoc"

man_pages = [(master_doc, "datasette", "Datasette Documentation", [author], 1)]
    'navigation_depth': 1,

html_sidebars = {
    "**": [
        "sidebar/scroll-start.html",
        "sidebar/brand.html",
        "sidebar/search.html",
        "sidebar/navigation.html",
        "sidebar/ethical-ads.html",
        "sidebar/scroll-end.html",
    ]
}

html_additional_pages = {
    "index": "your-custom-landing-page.html"
}

html_theme_options = {
    "sidebar_hide_name": True,
}

html_theme_options = {
    "top_of_page_button": "edit",
}

"""
## pygments_style = "sphinx"
## pygments_dark_style = "monokai"



