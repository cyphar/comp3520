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

/* Synchronisation helpers. */

#include <stdarg.h>
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>

#include "sync.h"

arcsem_t *arcsem_new(unsigned int value)
{
	arcsem_t *sem = malloc(sizeof(*sem));
	if (!sem)
		return NULL;

	*sem = (arcsem_t) {
		.lock = PTHREAD_MUTEX_INITIALIZER,
		.count = 1,
	};
	if (sem_init(&sem->inner, 0, value) < 0)
		goto err;

	return sem;

err:
	free(sem);
	return NULL;
}

arcsem_t *arcsem_get(arcsem_t *sem)
{
	if (!sem)
		return NULL;

	pthread_mutex_lock(&sem->lock);
	sem->count++;
	pthread_mutex_unlock(&sem->lock);
	return sem;
}

void arcsem_put(arcsem_t *sem)
{
	int count;

	if (!sem)
		return;

	pthread_mutex_lock(&sem->lock);
	count = --sem->count;
	pthread_mutex_unlock(&sem->lock);

	/* This is a bug -- every put() must be paired with get(). */
	if (count < 0)
		abort();
	/* No users left, time to free. */
	if (count == 0) {
		sem_destroy(&sem->inner);
		free(sem);
	}
}

void mailbox_init(signal_mailbox_t *mbox)
{
	*mbox = (signal_mailbox_t) SIGNAL_MAILBOX_INITIALIZER;
}

void mailbox_signal(signal_mailbox_t *mbox, arcsem_t *receipt)
{
	arcsem_t *old, *new = arcsem_get(receipt);

	pthread_mutex_lock(&mbox->lock);
	/* Swap receipt semaphore. */
	old = mbox->receipt;
	mbox->receipt = new;
	/* Pending signal. */
	mbox->pending = true;
	pthread_cond_signal(&mbox->cond);
	pthread_mutex_unlock(&mbox->lock);
	/* Free old semaphore. */
	arcsem_put(old);
}

void mailbox_retract(signal_mailbox_t *mbox)
{
	arcsem_t *old;

	pthread_mutex_lock(&mbox->lock);
	/* Swap receipt semaphore. */
	old = mbox->receipt;
	mbox->receipt = NULL;
	/* Clear pending signals. */
	mbox->pending = false;
	pthread_mutex_unlock(&mbox->lock);
	/* Free old semaphore. */
	arcsem_put(old);
}

void mailbox_wait_lock(signal_mailbox_t *mbox)
{
	pthread_mutex_lock(&mbox->lock);
	while (!mbox->pending)
		pthread_cond_wait(&mbox->cond, &mbox->lock);
	mbox->pending = false;
}

void mailbox_unlock(signal_mailbox_t *mbox)
{
	if (mbox->receipt)
		sem_post(&mbox->receipt->inner);
	pthread_mutex_unlock(&mbox->lock);
}

void barrier_init(barrier_t *barrier, int required)
{
	*barrier = (barrier_t) {
		.remaining = required,
		.lock = PTHREAD_MUTEX_INITIALIZER,
		.cond = PTHREAD_COND_INITIALIZER,
	};
}

void barrier_wait(barrier_t *barrier)
{
	pthread_mutex_lock(&barrier->lock);
	/* The last thread to hit the barrier wakes up the rest. */
	if (--barrier->remaining == 0)
		pthread_cond_broadcast(&barrier->cond);
	/* Wait until the group wake-up -- loop to avoid spurrious wake-ups. */
	while (barrier->remaining > 0)
		pthread_cond_wait(&barrier->cond, &barrier->lock);
	pthread_mutex_unlock(&barrier->lock);
}
