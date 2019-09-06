/*
 * Copyright (C) 2019 [450362910]
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TRAFFIC_H
#define TRAFFIC_H

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include "sync.h"

/* Helper to deal with fatal errors. */
#define bail(fmt, ...)														\
	do {																	\
		fprintf(stderr, fmt ": %s\n", ##__VA_ARGS__, strerror(errno));		\
		exit(1);															\
	} while (0)

/*
 * Return the length of an array. This only works on *arrays* (where sizeof
 * gives you the amount of memory taken up by the entire array rather than
 * sizeof(void *)). Note that "array function arguments" in C are actually
 * pointers and will thus give you the wrong result.
 */
#define ARRAY_LENGTH(A) (sizeof(A) / sizeof(*(A)))

typedef enum {
	NORTH = 0,
	EAST  = 1,
	SOUTH = 2,
	WEST  = 3,
} dir_t;
#define NUM_DIRECTIONS 4

/*
 * Represents a (start, end) direction in which a vehicle can travel. It's
 * represented as a single number so we can array-index headings. Only some
 * headings are valid (right-turns on the minor road are disallowed).
 */
typedef int heading_t;
#define PACK_HEADING(start, end)	((heading_t)((start)*NUM_DIRECTIONS + (end)))
#define HEADING_START(packed)		((dir_t)((packed) / NUM_DIRECTIONS))
#define HEADING_END(packed)			((dir_t)((packed) % NUM_DIRECTIONS))

/* Meta-structure for each controller. */
struct light_controller_t {
	/* Controller identifier (heading pair). */
	heading_t id[2];
	/* What is the next controller in the sequence? */
	struct light_controller_t *next;

	/* How long between cars entrying the intersection (in seconds)? */
	int intersection_gap;
	/* How long does this light stay green (in seconds)? */
	int green_interval;

	/*
	 * Barrier to indicate that all lights are ready. We can't use
	 * pthread_barrier_t (not permitted in assignment description) so we use
	 * our own implementation.
	 */
	barrier_t *ready;

	/*
	 * Mailbox for indicating that it's this controller's turn to work.
	 * Triggered by the _previous_ controller after it has finished (after the
	 * 2-second all-lights-red gap).
	 */
	signal_mailbox_t wake;

	/*
	 * Mailbox used by vehicles to decide whether or not they can travel
	 * through the intersection. Only one car can be in one lane in the
	 * intersection at a time (though cars in different lanes can overlap).
	 * Since left-turn cars are in the same lane as "forward" cars, this
	 * lane-based mutual exclusion is grouped by vehicle starting direction.
	 *
	 * To make life simpler, we just have NUM_DIRECTIONS (four) groups for all
	 * light controllers (even though only two are necessary). The unused ones
	 * don't really cost enough to be an issue, and it allows us to index using
	 * HEADING_START().
	 */
	signal_mailbox_t entry[NUM_DIRECTIONS];
};

/* Meta-structure for a vehicle -- thread has the responsibility to free it. */
struct vehicle_t {
	/* Vehicle identifier (unique for a given heading). */
	int id;
	/* What is the (start, end) of the vehicle. */
	heading_t heading;
};

#endif /* !TRAFFIC_H */
