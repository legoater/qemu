# -*- coding: utf-8 -*-
#
# QEMU documentation build configuration file, created by
# sphinx-quickstart on Thu Jan 31 16:40:14 2019.
#
# This config file can be used in one of two ways:
# (1) as a common config file which is included by the conf.py
# for each of QEMU's manuals: in this case sphinx-build is run multiple
# times, once per subdirectory.
# (2) as a top level conf file which will result in building all
# the manuals into a single document: in this case sphinx-build is
# run once, on the top-level docs directory.
#
# QEMU's makefiles take option (1), which allows us to install
# only the ones the user cares about (in particular we don't want
# to ship the 'devel' manual to end-users).
# Third-party sites such as readthedocs.org will take option (2).
#
#
# This file is execfile()d with the current directory set to its
# containing dir.
#
# Note that not all possible configuration values are present in this
# autogenerated file.
#
# All configuration values have a default; values that are commented out
# serve to show the default.

import os
import sys
import sphinx
from sphinx.errors import ConfigError

# The per-manual conf.py will set qemu_docdir for a single-manual build;
# otherwise set it here if this is an entire-manual-set build.
# This is always the absolute path of the docs/ directory in the source tree.
try:
    qemu_docdir
except NameError:
    qemu_docdir = os.path.abspath(".")

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use an absolute path starting from qemu_docdir.
#
# Our extensions are in docs/sphinx; the qapidoc extension requires
# the QAPI modules from scripts/.
sys.path.insert(0, os.path.join(qemu_docdir, "sphinx"))
sys.path.insert(0, os.path.join(qemu_docdir, "../scripts"))


# -- General configuration ------------------------------------------------

# If your documentation needs a minimal Sphinx version, state it here.
#
# 3.4.3 is the oldest version of Sphinx that ships on a platform we
# pledge build support for.
needs_sphinx = '3.4.3'

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'depfile',
    'hxtool',
    'kerneldoc',
    'qapi_domain',
    'qapidoc',
    'qmp_lexer',
]

if sphinx.version_info[:3] > (4, 0, 0):
    tags.add('sphinx4')
    extensions += ['dbusdoc']
else:
    extensions += ['fakedbusdoc']

# Add any paths that contain templates here, relative to this directory.
templates_path = [os.path.join(qemu_docdir, '_templates')]

# The suffix(es) of source filenames.
# You can specify multiple suffix as a list of string:
#
# source_suffix = ['.rst', '.md']
source_suffix = '.rst'

# The master toctree document.
master_doc = 'index'

# Interpret `single-backticks` to be a cross-reference to any kind of
# referenceable object. Unresolvable or ambiguous references will emit a
# warning at build time.
default_role = 'any'

# General information about the project.
project = u'QEMU'
copyright = u'2025, The QEMU Project Developers'
author = u'The QEMU Project Developers'

# The version info for the project you're documenting, acts as replacement for
# |version| and |release|, also used in various other places throughout the
# built documents.

# Extract this information from the VERSION file, for the benefit of
# standalone Sphinx runs as used by readthedocs.org. Builds run from
# the Makefile will pass version and release on the sphinx-build
# command line, which override this.
try:
    extracted_version = None
    with open(os.path.join(qemu_docdir, '../VERSION')) as f:
        extracted_version = f.readline().strip()
except:
    pass
finally:
    if extracted_version:
        version = release = extracted_version
    else:
        version = release = "unknown version"

bits = version.split(".")

major = int(bits[0])
minor = int(bits[1])
micro = int(bits[2])

# Check for a dev snapshot, so we can adjust to next
# predicted release version.
#
# This assumes we do 3 releases per year, so must bump
# major if minor == 2
if micro >= 50:
    micro = 0
    if minor == 2:
        major += 1
        minor = 0
    else:
        minor += 1

# These thresholds must match the constants
# MACHINE_VER_DELETION_MAJOR  & MACHINE_VER_DEPRECATION_MAJOR
# defined in include/hw/boards.h and the introductory text in
# docs/about/deprecated.rst
ver_machine_deprecation_version = "%d.%d.0" % (major - 3, minor)
ver_machine_deletion_version = "%d.%d.0" % (major - 6, minor)

# The language for content autogenerated by Sphinx. Refer to documentation
# for a list of supported languages.
#
# This is also used if you do content translation via gettext catalogs.
# Usually you set "language" from the command line for these cases.
language = 'en'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This patterns also effect to html_static_path and html_extra_path
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'

# If true, `todo` and `todoList` produce output, else they produce nothing.
todo_include_todos = False

# Sphinx defaults to warning about use of :option: for options not defined
# with "option::" in the document being processed. Turn that off.
suppress_warnings = ["ref.option"]

# The rst_epilog fragment is effectively included in every rST file.
# We use it to define substitutions based on build config that
# can then be used in the documentation. The fallback if the
# environment variable is not set is for the benefit of readthedocs
# style document building; our Makefile always sets the variable.
confdir = os.getenv('CONFDIR', "/etc/qemu")

vars = {
    "CONFDIR": confdir,
    "VER_MACHINE_DEPRECATION_VERSION": ver_machine_deprecation_version,
    "VER_MACHINE_DELETION_VERSION": ver_machine_deletion_version,
}

rst_epilog = "".join([
    ".. |" + key + "| replace:: ``" + vars[key] + "``\n"
    for key in vars.keys()
])

# We slurp in the defs.rst.inc and literally include it into rst_epilog,
# because Sphinx's include:: directive doesn't work with absolute paths
# and there isn't any one single relative path that will work for all
# documents and for both via-make and direct sphinx-build invocation.
with open(os.path.join(qemu_docdir, 'defs.rst.inc')) as f:
    rst_epilog += f.read()


# Normally, the QAPI domain is picky about what field lists you use to
# describe a QAPI entity. If you'd like to use arbitrary additional
# fields in source documentation, add them here.
qapi_allowed_fields = {
    "see also",
}

# Due to a limitation in Sphinx, we need to know which indices to
# generate in advance. Adding a namespace here allows that generation.
qapi_namespaces = {
    "QGA",
    "QMP",
    "QSD",
}

# -- Options for HTML output ----------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
try:
    import sphinx_rtd_theme
except ImportError:
    raise ConfigError(
        'The Sphinx \'sphinx_rtd_theme\' HTML theme was not found.\n'
    )

html_theme = 'sphinx_rtd_theme'

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
html_theme_options = {
    "style_nav_header_background": "#802400",
    "navigation_with_keys": True,
}

html_logo = os.path.join(qemu_docdir, "../ui/icons/qemu_128x128.png")

html_favicon = os.path.join(qemu_docdir, "../ui/icons/qemu_32x32.png")

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = [os.path.join(qemu_docdir, "sphinx-static")]

html_css_files = [
    'theme_overrides.css',
]

html_js_files = [
    'custom.js',
]

html_context = {
    "source_url_prefix": "https://gitlab.com/qemu-project/qemu/-/blob/master/docs/",
    "gitlab_user": "qemu-project",
    "gitlab_repo": "qemu",
    "gitlab_version": "master",
    "conf_py_path": "/docs/", # Path in the checkout to the docs root
}

# Custom sidebar templates, must be a dictionary that maps document names
# to template names.
#html_sidebars = {}

# Don't copy the rST source files to the HTML output directory,
# and don't put links to the sources into the output HTML.
html_copy_source = False

# -- Options for HTMLHelp output ------------------------------------------

# Output file base name for HTML help builder.
htmlhelp_basename = 'QEMUdoc'


# -- Options for LaTeX output ---------------------------------------------

latex_elements = {
    # The paper size ('letterpaper' or 'a4paper').
    #
    # 'papersize': 'letterpaper',

    # The font size ('10pt', '11pt' or '12pt').
    #
    # 'pointsize': '10pt',

    # Additional stuff for the LaTeX preamble.
    #
    # 'preamble': '',

    # Latex figure (float) alignment
    #
    # 'figure_align': 'htbp',
}

# Grouping the document tree into LaTeX files. List of tuples
# (source start file, target name, title,
#  author, documentclass [howto, manual, or own class]).
latex_documents = [
    (master_doc, 'QEMU.tex', u'QEMU Documentation',
     u'The QEMU Project Developers', 'manual'),
]


# -- Options for manual page output ---------------------------------------
# Individual manual/conf.py can override this to create man pages
man_pages = [
    ('interop/qemu-ga', 'qemu-ga',
     'QEMU Guest Agent',
     ['Michael Roth <mdroth@linux.vnet.ibm.com>'], 8),
    ('interop/qemu-ga-ref', 'qemu-ga-ref',
     'QEMU Guest Agent Protocol Reference',
     [], 7),
    ('interop/qemu-qmp-ref', 'qemu-qmp-ref',
     'QEMU QMP Reference Manual',
     [], 7),
    ('interop/qemu-storage-daemon-qmp-ref', 'qemu-storage-daemon-qmp-ref',
     'QEMU Storage Daemon QMP Reference Manual',
     [], 7),
    ('system/qemu-manpage', 'qemu',
     'QEMU User Documentation',
     ['Fabrice Bellard'], 1),
    ('system/qemu-block-drivers', 'qemu-block-drivers',
     'QEMU block drivers reference',
     ['Fabrice Bellard and the QEMU Project developers'], 7),
    ('system/qemu-cpu-models', 'qemu-cpu-models',
     'QEMU CPU Models',
     ['The QEMU Project developers'], 7),
    ('tools/qemu-img', 'qemu-img',
     'QEMU disk image utility',
     ['Fabrice Bellard'], 1),
    ('tools/qemu-nbd', 'qemu-nbd',
     'QEMU Disk Network Block Device Server',
     ['Anthony Liguori <anthony@codemonkey.ws>'], 8),
    ('tools/qemu-pr-helper', 'qemu-pr-helper',
     'QEMU persistent reservation helper',
     [], 8),
    ('tools/qemu-storage-daemon', 'qemu-storage-daemon',
     'QEMU storage daemon',
     [], 1),
    ('tools/qemu-trace-stap', 'qemu-trace-stap',
     'QEMU SystemTap trace tool',
     [], 1),
]
man_make_section_directory = False

# We use paths starting from qemu_docdir here so that you can run
# sphinx-build from anywhere and the kerneldoc extension can still
# find everything.
kerneldoc_bin = ['perl', os.path.join(qemu_docdir, '../scripts/kernel-doc')]
kerneldoc_srctree = os.path.join(qemu_docdir, '..')
hxtool_srctree = os.path.join(qemu_docdir, '..')
qapidoc_srctree = os.path.join(qemu_docdir, '..')
dbusdoc_srctree = os.path.join(qemu_docdir, '..')
dbus_index_common_prefix = ["org.qemu."]
