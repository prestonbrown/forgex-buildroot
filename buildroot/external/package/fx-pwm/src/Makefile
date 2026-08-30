CC      ?= cc
CFLAGS  ?= -O2 -Wall -Wextra
PREFIX  ?= /usr

all: fx-pwm

fx-pwm: fx_pwm.c pwm_abi.h
	$(CC) $(CFLAGS) -o $@ fx_pwm.c

install: fx-pwm
	install -D -m 0755 fx-pwm $(DESTDIR)$(PREFIX)/bin/fx-pwm

clean:
	rm -f fx-pwm

.PHONY: all install clean
