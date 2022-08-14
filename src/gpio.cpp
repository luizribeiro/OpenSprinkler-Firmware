/* OpenSprinkler Unified (AVR/RPI/BBB/LINUX) Firmware
 * Copyright (C) 2014 by Ray Wang (ray@opensprinkler.com)
 *
 * GPIO functions
 * Feb 2015 @ OpenSprinkler.com
 *
 * This file is part of the OpenSprinkler library
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see
 * <http://www.gnu.org/licenses/>. 
 */
 
#include "gpio.h"
#include <gpiod.h>


#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <poll.h>
#include <pthread.h>

#define BUFFER_MAX 64
#define GPIO_MAX	 64

// GPIO file descriptors
static int sysFds[GPIO_MAX] = {
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
} ;

// Interrupt service routine functions
static void (*isrFunctions [GPIO_MAX])(void);

static volatile int		 pinPass = -1 ;
static pthread_mutex_t pinMutex ;
static struct gpiod_chip *chip = NULL;

struct gpiod_chip *get_chip() {
  if (chip == NULL) {
    chip = gpiod_chip_open_by_name("gpiochip0");
  }
  return chip;
}

/** Set pin mode, in or out */
void pinMode(int pin, byte mode) {
  struct gpiod_chip *chip = get_chip();
  struct gpiod_line *line = gpiod_chip_get_line(chip, pin);
  if (mode == INPUT || mode == INPUT_PULLUP) {
    gpiod_line_request_input_flags(
        line,
        "opensprinkler",
        mode == INPUT
          ? GPIOD_CTXLESS_FLAG_BIAS_PULL_UP
          : GPIOD_CTXLESS_FLAG_BIAS_DISABLE
    );
  } else {
    gpiod_line_request_output(line, "opensprinkler", 0);
  }
}

/** Open file for digital pin */
int gpio_fd_open(int pin, int mode) {
  return 0;
}

/** Close file */
void gpio_fd_close(int fd) {
}

/** Read digital value */
byte digitalRead(int pin) {
  struct gpiod_chip *chip = get_chip();
  struct gpiod_line *line = gpiod_chip_get_line(chip, pin);
  return gpiod_line_get_value(line);
}

/** Write digital value given file descriptor */
void gpio_write(int fd, byte value) {
}

/** Write digital value */
void digitalWrite(int pin, byte value) {
  struct gpiod_chip *chip = get_chip();
  struct gpiod_line *line = gpiod_chip_get_line(chip, pin);
  gpiod_line_set_value(line, value);
}

#include "utils.h"
