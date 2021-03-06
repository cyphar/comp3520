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

#include "traffic.h"
#include "sync.h"

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

/* What are the valid (start, end) pairs? */
static const heading_t VALID_HEADINGS[] = {
#  	define HEADING_GENERIC(start, end, _) PACK_HEADING(start, end),
#	include "heading-list.h"
};

/* Represent a (start, end) pair as "x2y" for debugging output. */
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

/* Choose a (uniformly) random heading from VALID_HEADINGS. */
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

static struct light_controller_t trunk_fwd_light;
static struct light_controller_t minor_fwd_light;
static struct light_controller_t trunk_right_light;

static struct light_controller_t trunk_fwd_light = {
	.id = { PACK_HEADING(NORTH, SOUTH), PACK_HEADING(SOUTH, NORTH) }, /* (n2s, s2n) */
	.next = &minor_fwd_light,
	.wake = SIGNAL_MAILBOX_INITIALIZER,
	.entry = {
		[NORTH] = SIGNAL_MAILBOX_INITIALIZER,
		[SOUTH] = SIGNAL_MAILBOX_INITIALIZER,
	},
};

static struct light_controller_t minor_fwd_light = {
	.id = { PACK_HEADING(EAST, WEST), PACK_HEADING(WEST, EAST) }, /* (e2w, w2e) */
	.next = &trunk_right_light,
	.wake = SIGNAL_MAILBOX_INITIALIZER,
	.entry = {
		[EAST] = SIGNAL_MAILBOX_INITIALIZER,
		[WEST] = SIGNAL_MAILBOX_INITIALIZER,
	},
};

static struct light_controller_t trunk_right_light = {
	.id = { PACK_HEADING(NORTH, WEST), PACK_HEADING(SOUTH, EAST) }, /* (n2w, s2e) */
	.next = &trunk_fwd_light,
	.wake = SIGNAL_MAILBOX_INITIALIZER,
	.entry = {
		[NORTH] = SIGNAL_MAILBOX_INITIALIZER,
		[EAST]  = SIGNAL_MAILBOX_INITIALIZER,
	},
};

/* Set all of all light controllers. */
struct light_controller_t *ALL_CONTROLLERS[] = {
	&trunk_fwd_light,
	&minor_fwd_light,
	&trunk_right_light,
};

/* Mapping from (start, end) to the associated light_controller_t. */
struct light_controller_t *HEADING_CONTROLLERS[] = {
#	define HEADING_TRUNK_FWD(start, end, _)									\
	[PACK_HEADING(start, end)] = &trunk_fwd_light,
#	define HEADING_MINOR_FWD(start, end, _)									\
	[PACK_HEADING(start, end)] = &minor_fwd_light,
#	define HEADING_TRUNK_RIGHT(start, end, _)								\
	[PACK_HEADING(start, end)] = &trunk_right_light,
#	include "heading-list.h"
};

static void *light_start(void *arg)
{
	char *id = NULL;
	dir_t lane1, lane2;
	struct light_controller_t *self = arg;

	if (asprintf(&id, "(%s, %s)", heading_to_string(self->id[0]),
	                              heading_to_string(self->id[1])) < 0)
		bail("asprintf(light controller id) failed");

	/* Clean up the id string when pthread_cancell'd. */
	pthread_cleanup_push(free, id);

	lane1 = HEADING_START(self->id[0]);
	lane2 = HEADING_START(self->id[1]);

	printf("Traffic light mini-controller %s: "
	       "Initialization complete. I am ready.\n", id);

	barrier_wait(self->ready);

	for (;;) {
		struct timespec red_deadline = { 0 };

		/* Wait for our turn. */
		mailbox_wait_lock(&self->wake);

		/* When do we need to turn red again? */
		red_deadline.tv_sec = time(NULL) + self->green_interval;

		/* Until the deadline is reached, allow cars to pass. */
		printf("The traffic lights %s have changed to green.\n", id);
		while (time(NULL) < red_deadline.tv_sec) {
			/*
			 * Create a new semaphore for each iteration so we can be sure that
			 * we catch a crossing after we "send" new signals -- there isn't
			 * any other fool-proof way to set the sempahore back to 0.
			 */
			arcsem_t *receipt = arcsem_new(0);

			/*
			 * Signal both lanes to allow one vehicle to pass through -- if
			 * there's already a pending signal then this just updates the
			 * receipt semaphore (freeing the old one).
			 */
			mailbox_signal(&self->entry[lane1], receipt);
			mailbox_signal(&self->entry[lane2], receipt);

			/* Wait for one of them to have passed. */
			sem_timedwait(&receipt->inner, &red_deadline);

			/*
			 * We're done waiting -- the only references still alive are the
			 * ones in the mailboxes (which will be cleared on our next loop or
			 * by mailbox_retract).
			 */
			arcsem_put(receipt);
		}
		/* Retract any remaining signals -- and free the semaphores. */
		mailbox_retract(&self->entry[lane1]);
		mailbox_retract(&self->entry[lane2]);

		/* No more car crossings from here on. */
		printf("The traffic lights %s will change to red now.\n", id);

		/* We pause for 2 seconds before triggering the next controller. */
		sleep(2);
		mailbox_unlock(&self->wake);
		mailbox_signal(&self->next->wake, NULL);
	}

	/* Should never be reached. */
	pthread_cleanup_pop(true);
	return NULL;
}

static void *vehicle_start(void *arg)
{
	struct vehicle_t *self = arg;
	struct light_controller_t *master = HEADING_CONTROLLERS[self->heading];
	dir_t lane = HEADING_START(self->heading);

	printf("Vehicle %d %s has arrived at the intersection.\n",
	       self->id, heading_to_string(self->heading));

	mailbox_wait_lock(&master->entry[lane]);

	printf("Vehicle %d %s is proceeding through the intersection.\n",
	       self->id, heading_to_string(self->heading));
	sleep(master->intersection_gap);

	mailbox_unlock(&master->entry[lane]);

	free(self);
	return NULL;
}

static void readint(const char *prompt, int *value)
{
	printf("Enter %s (int): ", prompt);
	if (scanf("%d", value) != 1)
		bail("expected integer input to prompt!");
}

int main(void)
{
	int num_vehicles, max_arrival_gap, intersection_gap;

	int next_vehicle_id[NUM_DIRECTIONS * NUM_DIRECTIONS] = { 0 };
	time_t last_vehicle_spawn[NUM_DIRECTIONS * NUM_DIRECTIONS] = { 0 };

	barrier_t ready_barrier;
	pthread_t controllers[ARRAY_LENGTH(ALL_CONTROLLERS)] = { 0 };
	pthread_t *vehicles = NULL;

	/* Seed PRNG. */
	srand48(time(NULL) ^ getpid());

	readint("the total number of vehicles", &num_vehicles);
	readint("vehicles arrival rate", &max_arrival_gap);
	readint("minimum interval between two consecutive vehicles", &intersection_gap);

	readint("green time for forward-moving vehicles on trunk road",
			&trunk_fwd_light.green_interval);
	readint("green time for vehicles on minor road",
			&minor_fwd_light.green_interval);
	readint("green time for right-turning vehicles on trunk road",
			&trunk_right_light.green_interval);

	/* We need all controllers and the main thread to be ready. */
	barrier_init(&ready_barrier, ARRAY_LENGTH(ALL_CONTROLLERS) + 1);

	/* Set up the controllers. */
	for (size_t i = 0; i < ARRAY_LENGTH(ALL_CONTROLLERS); i++) {
		struct light_controller_t *current = ALL_CONTROLLERS[i];
		current->ready = &ready_barrier;
		current->intersection_gap = intersection_gap;
	}

	/* Spawn light controllers. */
	for (size_t i = 0; i < ARRAY_LENGTH(ALL_CONTROLLERS); i++) {
		struct light_controller_t *current = ALL_CONTROLLERS[i];
		if (pthread_create(&controllers[i], NULL, light_start, current) < 0)
			bail("pthread_create(controller[%ld]) failed", i);
	}

	/* Wait until all controllers are ready ... */
	barrier_wait(&ready_barrier);
	/* ... then trigger the default state. */
	mailbox_signal(&trunk_fwd_light.wake, NULL);

	/* Spawn vehicle threads. */
	vehicles = calloc(num_vehicles, sizeof(*vehicles));
	if (!vehicles)
		bail("calloc(vehicles) failed");
	for (ssize_t i = 0; i < num_vehicles; i++) {
		int delay;
		struct vehicle_t *current;

		current = malloc(sizeof(*current));
		if (!current)
			bail("malloc(vehicle_t[%ld]) failed", i);

		current->heading = random_heading();
		current->id = next_vehicle_id[current->heading]++;

		/*
		 * Delay thread spawning based on when the last vehicle (with the same
		 * heading) was spawned. This is all single-threaded, as required by
		 * the assignment description (to be more physically accurate you would
		 * have a separate thread for each possible heading, or some other
		 * scheduling system to not block spawning other headings if the
		 * current one needs a longer delay).
		 */
		if (last_vehicle_spawn[current->heading] < time(NULL))
			/* Last vehicle spawned >1s ago -- [0,max_arrival_gap). */
			delay = (1 + max_arrival_gap) * drand48();
		else
			/* Last vehicle spawned <=1s ago -- [1,max_arrival_gap). */
			delay = 1 + (max_arrival_gap * drand48());

		sleep(delay);
		last_vehicle_spawn[current->heading] = time(NULL);

		if (pthread_create(&vehicles[i], NULL, vehicle_start, current) < 0)
			bail("pthread_create(vehicle[%ld]) failed", i);
	}

	/* Wait for all the vehicles to pass. */
	for (ssize_t i = 0; i < num_vehicles; i++)
		pthread_join(vehicles[i], NULL);
	free(vehicles);

	/* Kill the controllers (first cancel, then join). */
	for (size_t i = 0; i < ARRAY_LENGTH(ALL_CONTROLLERS); i++)
		pthread_cancel(controllers[i]);
	for (size_t i = 0; i < ARRAY_LENGTH(ALL_CONTROLLERS); i++)
		pthread_join(controllers[i], NULL);

	printf("Main thread: There are no more vehicles to serve. "
	       "The simulation will end now.\n");
	return 0;
}
