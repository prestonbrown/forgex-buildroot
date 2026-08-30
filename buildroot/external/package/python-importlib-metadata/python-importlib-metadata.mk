################################################################################
#
# python-importlib-metadata
#
################################################################################

PYTHON_IMPORTLIB_METADATA_VERSION = 8.2.0
PYTHON_IMPORTLIB_METADATA_SOURCE = importlib_metadata-$(PYTHON_IMPORTLIB_METADATA_VERSION).tar.gz
PYTHON_IMPORTLIB_METADATA_SITE = https://files.pythonhosted.org/packages/f6/a1/db39a513aa99ab3442010a994eef1cb977a436aded53042e69bee6959f74
PYTHON_IMPORTLIB_METADATA_SETUP_TYPE = setuptools
PYTHON_IMPORTLIB_METADATA_LICENSE = Apache-2.0
PYTHON_IMPORTLIB_METADATA_LICENSE_FILES = LICENSE
PYTHON_IMPORTLIB_METADATA_DEPENDENCIES = \
	host-python-setuptools-scm \
	python-zipp

$(eval $(python-package))
