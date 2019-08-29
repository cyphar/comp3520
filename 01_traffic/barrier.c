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

/*
 * The assignment description doesn't allow us to use pthread's built-in
 * barriers, so we implement our own using permitted primitives (condition
 * variables and mutexes).
 */

#include <pthread.h>
#include "barrier.h"

int barrier_init(barrier_t *barrier, int required)
{
	barrier->remaining = required;

	if (pthread_mutex_init(&barrier->lock, NULL))
		return -1;
	if (pthread_cond_init(&barrier->cond, NULL))
		return -1;
	return 0;
}

void barrier_destory(barrier_t *barrier)
{
	pthread_mutex_destroy(&barrier->lock);
	pthread_cond_destroy(&barrier->cond);
}

void barrier_wait(barrier_t *barrier)
{
	pthread_mutex_lock(&barrier->lock);
	/* The last thread to hit the barrier wakes up the rest. */
	if (--barrier->remaining == 0)
		pthread_cond_broadcast(&barrier->cond);
	/* Loop to avoid spurrious wake-ups. */
	while (barrier->remaining > 0)
		pthread_cond_wait(&barrier->cond, &barrier->lock);
	pthread_mutex_unlock(&barrier->lock);
}
