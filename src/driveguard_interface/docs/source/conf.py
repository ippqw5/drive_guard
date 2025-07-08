# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys
sys.path.insert(0, os.path.abspath('../../'))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'DriveGuard API'
copyright = '2025, DriveGuard Team'
author = 'Qian.Cheng'
release = '1.0.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.napoleon',
    'sphinx.ext.intersphinx',
    'sphinx.ext.coverage',
    'sphinx.ext.autosummary'
]

templates_path = ['_templates']
exclude_patterns = [
    'driveguard_interface.driveguard_carla.rst',
    'driveguard_interface.common.rst',
    'driveguard_interface.rst',
    'modules.rst',
]



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# -- Options for autodoc ----------------------------------------------------
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,  # 改为 True，显示所有成员
    'exclude-members': '__weakref__',
    'inherited-members': True,  # 添加这行，显示继承的方法
    'show-inheritance': True,   # 添加这行，显示继承关系
}

# 简化显示
add_module_names = False  # 不显示完整模块路径
autodoc_typehints = 'description'
autodoc_typehints_format = 'fully-qualified'  # 使用全限定名显示类型提示
autodoc_class_signature = 'mixed'  # 将类签名与类名分开

# -- Napoleon settings -------------------------------------------------------
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = False
napoleon_include_private_with_doc = False
napoleon_use_param = True
napoleon_use_rtype = True

# 安装: pip install sphinx-book-theme

html_theme = 'sphinx_book_theme'

html_theme_options = {
    "repository_url": "https://github.com/ippqw5/drive_guard",
    "use_repository_button": True,
    "use_issues_button": True,
    "use_download_button": True,
    "show_toc_level": 2,  # 添加这行，在右侧栏显示更深层级
}
