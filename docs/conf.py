# Configuration file for the Sphinx documentation builder.
import os
import sys
from datetime import datetime
from pathlib import Path

import last_letter_lib
from last_letter_lib import aerodynamics
from last_letter_lib import propulsion
from last_letter_lib import simulation
from last_letter_lib import systems


project = "last_letter_lib"
author = "George Zogopoulos"
copyright = f"{datetime.now().year}, Avy B.V."
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx_click",
]
autodoc_typehints = "description"
html_theme = "furo"

# # Ignore broken references warnings when using nitpicky
# nitpick_ignore = [
#     ("py:class", "enum.Enum"),
#     ("py:class", "pydantic.main.BaseModel"),
#     ("py:class", "pydantic.types.FilePath"),
# ]
