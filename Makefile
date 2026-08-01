# SPDX-FileCopyrightText: GARDENA GmbH
#
# SPDX-License-Identifier: GPL-2.0-or-later

CROSS_COMPILE ?=

CC     = $(CROSS_COMPILE)gcc
CFLAGS += -Wall -Wextra -Werror -O2 -std=gnu11 -MMD -MP

# Overridable so CI can pin the same version the tree was formatted with;
# clang-format output differs between major releases.
CLANG_FORMAT ?= clang-format
CLANG_TIDY   ?= clang-tidy

TARGET = sim3u-flasher
OBJDIR = build

SRCS = main.c swd.c gpio.c gpio_mt7688.c gpio_at91sam9x5.c sim3u_flash.c
OBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))

# The real tool against an emulated target, with no GPIO layer under it
MOCK = $(OBJDIR)/sim3u-flasher-test
MOCK_SRCS = main.c swd.c sim3u_flash.c gpio_mock.c

.PHONY: all clean format format-check lint test check

all: $(TARGET)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: %.c | $(OBJDIR)
	$(CC) $(CFLAGS) -I. -c $< -o $@

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@

-include $(OBJS:.o=.d)

clean:
	rm -f $(TARGET)
	rm -rf $(OBJDIR)

format:
	$(CLANG_FORMAT) -i *.c *.h

format-check:
	$(CLANG_FORMAT) --dry-run --Werror *.c *.h

# gpio_mock.c replaces the accessors gpio.h defines inline, so it only makes
# sense to a compiler that has GPIO_MOCK set
lint:
	$(CLANG_TIDY) $(SRCS) -- -I. -std=gnu11
	$(CLANG_TIDY) gpio_mock.c -- -I. -std=gnu11 -DGPIO_MOCK

$(MOCK): $(MOCK_SRCS) | $(OBJDIR)
	$(CC) $(CFLAGS) -DGPIO_MOCK -I. $(MOCK_SRCS) -o $@

test: $(MOCK)
	./test.sh $(MOCK)

check: format-check lint
