CC      ?= gcc
CFLAGS  ?= -Wall -Wextra -O2

MAVLINK_INC := ./c_library_v2

# Source files
SRC_DRONE := main_drone.c mav_fc_link.c
OBJ_DRONE := $(SRC_DRONE:.c=.o)

SRC_GROUND := main_ground.c
OBJ_GROUND := $(SRC_GROUND:.c=.o)

TARGET_DRONE := drone_mav
TARGET_GROUND := ground_mav

CFLAGS += -I$(MAVLINK_INC) -Ihal

.PHONY: all clean

all: $(TARGET_DRONE) $(TARGET_GROUND)

$(TARGET_DRONE): $(OBJ_DRONE)
	$(CC) $(CFLAGS) -o $@ $^

$(TARGET_GROUND): $(OBJ_GROUND)
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c mav_fc_link.h
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o $(TARGET_DRONE) $(TARGET_GROUND)