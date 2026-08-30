################################################################################
#
# python-streaming-form-data
#
################################################################################

PYTHON_STREAMING_FORM_DATA_VERSION = 1.15.0
PYTHON_STREAMING_FORM_DATA_SOURCE = streaming-form-data-$(PYTHON_STREAMING_FORM_DATA_VERSION).tar.gz
PYTHON_STREAMING_FORM_DATA_SITE = https://files.pythonhosted.org/packages/ed/ef/a8d74a9521cfea78b3b2509ae7f4d7d04f1afd2e2317e61e90115e9a5a11
PYTHON_STREAMING_FORM_DATA_SETUP_TYPE = setuptools
PYTHON_STREAMING_FORM_DATA_LICENSE = MIT
PYTHON_STREAMING_FORM_DATA_LICENSE_FILES = LICENSE.txt
PYTHON_STREAMING_FORM_DATA_DEPENDENCIES = python-smart-open

$(eval $(python-package))
