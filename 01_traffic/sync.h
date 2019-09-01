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

#ifndef SYNC_H
#define SYNC_H

#include <stdbool.h>
#include <pthread.h>
#include <semaphore.h>

/*
 * A atomically-reference-counted semaphore, to allow for sem_destroy() to be
 * called in circumstances where you can't be sure which thread will be the
 * last user of the semaphore.
 *
 * This is required in order for signal_mailbox_t's ->receipt to work properly
 * for the controller threads, otherwise you could end up with vehicle threads
 * that are trying to sem_post() on a free()'d semaphore.
 */
typedef struct {
	/* Protects the reference count. */
	pthread_mutex_t lock;
	int count;
	/* Inner object. */
	sem_t inner;
} arcsem_t;

arcsem_t *arcsem_new(unsigned int value);
arcsem_t *arcsem_get(arcsem_t *sem);
void arcsem_put(arcsem_t *sem);

/*
 * Condition variables suffer from not "mailboxing" signals (if the receiver is
 * not wait()ing at the time the signal is sent, it gets lost). This solves the
 * problem by storing a "pending signal" variable, as well as providing a
 * mechanism to get read receipts (from multiple mailboxes) through an arcsem_t.
 */
typedef struct {
	/* Signalling. */
	pthread_mutex_t lock;
	pthread_cond_t cond;
	/* Is there a pending signal? */
	bool pending;
	/* sem_post()ed when mailbox_wait_unlock() returns. */
	arcsem_t *receipt;
} signal_mailbox_t;

#define SIGNAL_MAILBOX_INITIALIZER \
	{ .lock = PTHREAD_MUTEX_INITIALIZER, .cond = PTHREAD_COND_INITIALIZER }

void mailbox_init(signal_mailbox_t *mbox);

/* Send a signal to the mailbox (and store the receipt semaphore). */
void mailbox_signal(signal_mailbox_t *mbox, arcsem_t *receipt);
/* Rescind a previously sent signal. */
void mailbox_retract(signal_mailbox_t *mbox);
/* Wait for a signal (or take the pending one) and take the mbox lock */
void mailbox_wait_lock(signal_mailbox_t *mbox);
void mailbox_unlock(signal_mailbox_t *mbox);

/*
 * The assignment description doesn't allow us to use pthread's built-in
 * barriers, so we implement our own using permitted primitives (condition
 * variables and mutexes).
 */
typedef struct {
	/* Signalling. */
	pthread_mutex_t lock;
	pthread_cond_t cond;
	/* How many remaining threads are there before the barrier opens? */
	int remaining;
} barrier_t;

void barrier_init(barrier_t *barrier, int required);
void barrier_wait(barrier_t *barrier);

#endif /* !SYNC_H */
