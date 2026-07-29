CROSS_COMPILE ?=

CC     = $(CROSS_COMPILE)gcc
CFLAGS += -Wall -Wextra -Werror -O2 -std=gnu11 -MMD -MP

TARGET = sim3u-flasher
OBJDIR = build

SRCS = main.c swd.c gpio.c sim3u_flash.c
OBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))

.PHONY: all clean

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
