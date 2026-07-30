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

SRCS = main.c swd.c gpio.c sim3u_flash.c
OBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))

.PHONY: all clean format format-check lint check

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

lint:
	$(CLANG_TIDY) *.c -- -I. -std=gnu11

check: format-check lint
