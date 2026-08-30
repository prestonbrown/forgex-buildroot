################################################################################
#
# python-smart-open
#
################################################################################

PYTHON_SMART_OPEN_VERSION = 7.0.5
PYTHON_SMART_OPEN_SOURCE = smart_open-$(PYTHON_SMART_OPEN_VERSION).tar.gz
PYTHON_SMART_OPEN_SITE = https://files.pythonhosted.org/packages/a3/d8/1481294b2d110b805c0f5d23ef34158b7d5d4283633c0d34c69ea89bb76b
PYTHON_SMART_OPEN_SETUP_TYPE = setuptools
PYTHON_SMART_OPEN_LICENSE = MIT
PYTHON_SMART_OPEN_LICENSE_FILES = LICENSE
PYTHON_SMART_OPEN_DEPENDENCIES = python-wrapt

$(eval $(python-package))
