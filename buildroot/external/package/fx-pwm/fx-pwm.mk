# fx-pwm: AD5X buzzer/PWM control via the stock soc_pwm.ko ioctl ABI.
#
# Small enough that an in-tree source dir beats a fetch; the buildroot
# 'local' site method rsyncs src/ into the build tree.
FX_PWM_VERSION = 1.0.0
FX_PWM_SITE = $(BR2_EXTERNAL_FORGEX_PATH)/package/fx-pwm/src
FX_PWM_SITE_METHOD = local
FX_PWM_LICENSE = GPL-3.0-or-later

define FX_PWM_BUILD_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) \
		CC="$(TARGET_CC)" CFLAGS="$(TARGET_CFLAGS) -Wall -Wextra"
endef

define FX_PWM_INSTALL_TARGET_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D) install DESTDIR="$(TARGET_DIR)"
endef

$(eval $(generic-package))
