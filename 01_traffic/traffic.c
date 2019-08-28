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

#define _GNU_SOURCE
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/*
 *     |   |   |     .n.
 *     |   |   |     w e
 *     |   |↲  |     .s.
 * ----+ - - - +----
 *     :       :
 *     :       :
 * ----:       :----
 *     :       :
 *     :       :
 * ----+ - - - +----
 *     |  ↱|   |
 *     |   |   |
 *     |   |   |
 */

#define bail(fmt, ...)														\
	do {																	\
		fprintf(stderr, fmt ": %s\n", ##__VA_ARGS__, strerror(errno));		\
		exit(1);															\
	} while (0)

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
#define PACK_HEADING(start, end) ((heading_t)((start)*NUM_DIRECTIONS + (end)))
#define HEADING_START(packed)	((dir_t)((packed) / NUM_DIRECTIONS))
#define HEADING_END(packed)	  ((dir_t)((packed) % NUM_DIRECTIONS))

static const heading_t VALID_HEADINGS[] = {
#  	define HEADING_GENERIC(start, end, _) PACK_HEADING(start, end),
#	include "heading-list.h"
};

static const char *heading_to_string(heading_t heading)
{
	switch (heading) {
#	define HEADING_GENERIC(start, end, name)								\
	case PACK_HEADING(start, end):											\
		return name;
#	include "heading-list.h"
	}
	return "invalid-heading";
}

static heading_t random_heading()
{
	/*
	 * (rand() % RANGE) gives you bad random distribution (it's usually skewed
	 * heavily towards 0, especially if RANGE is non-prime). See
	 * <http://www.azillionmonkeys.com/qed/random.html> for more details.
	 *
	 * Instead, we use drand48(3) to give us a uniformly-distributed value in
	 * [0,1) and then multiply it to match the choice. There are hacks you can
	 * do to make rand() produce a better distribution, but drand48(3) is much
	 * simpler to get right.
	 */
	size_t choice = floor(ARRAY_LENGTH(VALID_HEADINGS) * drand48());
	return VALID_HEADINGS[choice];
}

/* Meta-structure for each controller. */
struct light_controller_t {
	/* Controller identifier (heading pair). */
	heading_t id[2];
	/* What is the next controller in the sequence? */
	struct light_controller_t *next;

	/* How long between cars entrying the intersection (in seconds)? */
	int car_interval;
	/* How long does this light stay green (in seconds)? */
	int green_interval;

	/*
	 * Condition variable (and mutex) used to wake up this controller.
	 * Triggered by the _previous_ controller after it has finished.
	 */
	pthread_cond_t wake;
	pthread_mutex_t wake_lock;

	/*
	 * Condition variable (and mutex) used by vehicles to decide whether or not
	 * they can travel through the intersection. Only one car can be in one
	 * lane in the intersection at a time (though cars in different lanes can
	 * overlap). Since left-turn cars are in the same lane as "forward" cars,
	 * the mutual exclusion is grouped by vehicle starting direction.
	 *
	 * To make life simpler, we just have NUM_DIRECTIONS (four) groups for all
	 * light controllers (even though only two are necessary). The unused ones
	 * don't really cost enough to be an issue, and it allows us to index using
	 * HEADING_START().
	 */
	struct {
		pthread_cond_t cond;
		pthread_mutex_t lock;
	} entry[NUM_DIRECTIONS];
};

/* Meta-structure for a vehicle. */
struct vehicle_t {
	/* Vehicle identifier (unique for a given heading). */
	int id;
	/* What is the (start, end) of the vehicle. */
	heading_t heading;
};

static struct light_controller_t trunk_fwd_light;
static struct light_controller_t minor_fwd_light;
static struct light_controller_t trunk_right_light;

static struct light_controller_t trunk_fwd_light = {
	.id = { PACK_HEADING(NORTH, SOUTH), PACK_HEADING(SOUTH, NORTH) }, /* (n2s, s2n) */
	.next = &minor_fwd_light,
	.wake = PTHREAD_COND_INITIALIZER, .wake_lock = PTHREAD_MUTEX_INITIALIZER,
	.entry = {
		[NORTH] = { .cond = PTHREAD_COND_INITIALIZER, .lock = PTHREAD_MUTEX_INITIALIZER },
		[SOUTH] = { .cond = PTHREAD_COND_INITIALIZER, .lock = PTHREAD_MUTEX_INITIALIZER },
	},
};

static struct light_controller_t minor_fwd_light = {
	.id = { PACK_HEADING(EAST, WEST), PACK_HEADING(WEST, EAST) }, /* (e2w, w2e) */
	.next = &trunk_right_light,
	.wake = PTHREAD_COND_INITIALIZER, .wake_lock = PTHREAD_MUTEX_INITIALIZER,
	.entry = {
		[EAST] = { .cond = PTHREAD_COND_INITIALIZER, .lock = PTHREAD_MUTEX_INITIALIZER },
		[WEST] = { .cond = PTHREAD_COND_INITIALIZER, .lock = PTHREAD_MUTEX_INITIALIZER },
	},
};

static struct light_controller_t trunk_right_light = {
	.id = { PACK_HEADING(NORTH, WEST), PACK_HEADING(SOUTH, EAST) }, /* (n2w, s2e) */
	.next = &trunk_fwd_light,
	.wake = PTHREAD_COND_INITIALIZER, .wake_lock = PTHREAD_MUTEX_INITIALIZER,
	.entry = {
		[NORTH] = { .cond = PTHREAD_COND_INITIALIZER, .lock = PTHREAD_MUTEX_INITIALIZER },
		[EAST]  = { .cond = PTHREAD_COND_INITIALIZER, .lock = PTHREAD_MUTEX_INITIALIZER },
	},
};

struct light_controller_t *CONTROLLERS[] = {
#	define HEADING_TRUNK_FWD(start, end, _)									\
	[PACK_HEADING(start, end)] = &trunk_fwd_light,
#	define HEADING_MINOR_FWD(start, end, _)									\
	[PACK_HEADING(start, end)] = &minor_fwd_light,
#	define HEADING_TRUNK_RIGHT(start, end, _)								\
	[PACK_HEADING(start, end)] = &trunk_right_light,
#	include "heading-list.h"
};

static void *light_controller(void *arg)
{
	char *id = NULL;
	dir_t lane1, lane2;
	struct light_controller_t *self = arg;

	if (asprintf(&id, "(%s, %s)", heading_to_string(self->id[0]),
	                              heading_to_string(self->id[1])) < 0)
		bail("asprintf(light controller id) failed");

	lane1 = HEADING_START(self->id[0]);
	lane2 = HEADING_START(self->id[1]);

	printf("Traffic light mini-controller %s: "
	       "Initialization complete. I am ready.\n", id);

	for (;;) {
		time_t red_deadline;

		/* Wait for our turn. */
		pthread_mutex_lock(&self->wake_lock);
		pthread_cond_wait(&self->wake, &self->wake_lock);

		/* When do we need to turn red again? */
		red_deadline = time(NULL) + self->green_interval;

		/* Until the deadline is reached, allow cars to pass. */
		printf("The traffic lights %s have changed to green.\n", id);
		while (time(NULL) < red_deadline) {
			/* Signal both lanes to allow one vehicle to pass through. */
			pthread_cond_signal(&self->entry[lane1].cond);
			pthread_cond_signal(&self->entry[lane2].cond);
			sleep(self->car_interval);
		}
		printf("The traffic lights %s will change to red now.\n", id);
		pthread_mutex_unlock(&self->wake_lock);

		/* We pause for 2 seconds before triggering the next controller. */
		sleep(2);
		pthread_cond_signal(&self->next->wake);
	}

	free(id);
	return NULL;
}

static void *vehicle(void *arg)
{
	struct vehicle_t *self = arg;
	struct light_controller_t *master = CONTROLLERS[self->heading];
	dir_t lane = HEADING_START(self->heading);

	printf("Vehicle %d %s has arrived at the intersection.\n",
	       self->id, heading_to_string(self->heading));

	pthread_mutex_lock(&master->entry[lane].lock);
	pthread_cond_wait(&master->entry[lane].cond, &master->entry[lane].lock);

	printf("Vehicle %d %s is proceeding through the intersection.\n",
	       self->id, heading_to_string(self->heading));

	pthread_mutex_unlock(&master->entry[lane].lock);

	free(self);
	return NULL;
}

int main(void)
{
	int n_vehicles, max_vehicle_period, car_interval;
	pthread_t trunk_fwd, minor_fwd, trunk_right, *vehicles = NULL;

	int vehicle_counts[NUM_DIRECTIONS * NUM_DIRECTIONS] = { 0 };
	time_t vehicle_last_spawn[NUM_DIRECTIONS * NUM_DIRECTIONS] = { 0 };

	/* Seed PRNG. */
	srand48(time(NULL) ^ getpid());

	printf("Enter the total number of vehicles (int): ");
	scanf("%d", &n_vehicles);
	printf("Enter vehicles arrival rate (int): ");
	scanf("%d", &max_vehicle_period);
	printf("Enter minimum interval between two consecutive vehicles (int): ");
	scanf("%d", &car_interval);

	trunk_fwd_light.car_interval = car_interval;
	minor_fwd_light.car_interval = car_interval;
	trunk_right_light.car_interval = car_interval;

	printf("Enter green time for forward-moving vehicles on trunk road (int): ");
	scanf("%d", &trunk_fwd_light.green_interval);
	printf("Enter green time for vehicles on minor road (int): ");
	scanf("%d", &minor_fwd_light.green_interval);
	printf("Enter green time for right-turning vehicles on trunk road (int): ");
	scanf("%d", &trunk_right_light.green_interval);

	/* Spawn light controllers. */
	if (pthread_create(&trunk_fwd, NULL, light_controller, &trunk_fwd_light) < 0)
		bail("pthread_create(trunk_fwd) failed");
	if (pthread_create(&minor_fwd, NULL, light_controller, &minor_fwd_light) < 0)
		bail("pthread_create(minor_fwd) failed");
	if (pthread_create(&trunk_right, NULL, light_controller, &trunk_right_light) < 0)
		bail("pthread_create(trunk_right) failed");

	/* Trigger the default state. */
	pthread_cond_signal(&trunk_fwd_light.wake);

	/* Spawn vehicle threads. */
	vehicles = calloc(n_vehicles, sizeof(*vehicles));
	if (!vehicles)
		bail("calloc(vehicles) failed");
	for (ssize_t i = 0; i < n_vehicles; i++) {
		int delay;
		struct vehicle_t *current;

		current = malloc(sizeof(*current));
		if (!current)
			bail("malloc(vehicle_t[%ld]) failed", i);

		current->heading = random_heading();
		current->id = vehicle_counts[current->heading]++;

		/*
		 * Delay thread spawning based on whether
		 * We need to delay spawning the thread depending on whether we spawned
		 * a thread for the same direction in the past second.
		 */
		if (vehicle_last_spawn[current->heading] < time(NULL))
			delay = (1 + max_vehicle_period) * drand48();
		else
			delay = 1 + (max_vehicle_period * drand48());

		sleep(delay);
		vehicle_last_spawn[current->heading] = time(NULL);

		if (pthread_create(&vehicles[i], NULL, vehicle, current) < 0)
			bail("pthread_create(vehicle[%ld]) failed", i);
	}

	/* Wait for all the vehicles to pass. */
	for (ssize_t i = 0; i < n_vehicles; i++)
		pthread_join(vehicles[i], NULL);

	/* Kill the controllers. */
	pthread_cancel(trunk_fwd);
	pthread_cancel(minor_fwd);
	pthread_cancel(trunk_right);

	puts("Main thread: There are no more vehicles to serve. "
	     "The simulation will end now.");
	return 0;
}
