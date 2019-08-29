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

#ifndef BARRIER_H
#define BARRIER_H

typedef struct {
	int remaining;
	pthread_mutex_t lock;
	pthread_cond_t cond;
} barrier_t;

int barrier_init(barrier_t *barrier, int required);
void barrier_destory(barrier_t *barrier);

void barrier_wait(barrier_t *barrier);

#endif /* !BARRIER_H */
