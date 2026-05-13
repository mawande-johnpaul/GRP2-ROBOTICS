# ================================================================
# Makefile — astar_robot controller
# CSC 2207 Robotics | GRP2 | Lecture 4 Practical
#
# Webots on Linux looks for the compiled binary at:
#   controllers/astar_robot/astar_robot
#
# Usage:
#   make          — build
#   make clean    — remove build artefacts
# ================================================================

# Webots installation path (adjust if yours differs)
WEBOTS_HOME ?= /usr/local/webots

CC      = gcc
TARGET  = astar_robot
SRCS    = astar_robot.c astar.c
OBJS    = $(SRCS:.c=.o)

CFLAGS  = -Wall -Wextra -O2 \
          -I$(WEBOTS_HOME)/include/controller/c

LDFLAGS = -L$(WEBOTS_HOME)/lib/controller \
          -lController \
          -Wl,-rpath,$(WEBOTS_HOME)/lib/controller \
          -lm

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
