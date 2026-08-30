################################################################################
#
# python-preprocess-cancellation
#
################################################################################

PYTHON_PREPROCESS_CANCELLATION_VERSION = 0.2.1
PYTHON_PREPROCESS_CANCELLATION_SOURCE = preprocess_cancellation-$(PYTHON_PREPROCESS_CANCELLATION_VERSION).tar.gz
PYTHON_PREPROCESS_CANCELLATION_SITE = https://files.pythonhosted.org/packages/4a/56/7e18b0336c1e6c6622411dd0d3a7634b171e4d156a13b1ceaa048682454a
PYTHON_PREPROCESS_CANCELLATION_SETUP_TYPE = poetry
PYTHON_PREPROCESS_CANCELLATION_LICENSE = GPL-3.0
PYTHON_PREPROCESS_CANCELLATION_LICENSE_FILES = LICENSE

$(eval $(python-package))
