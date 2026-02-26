/**
 * scheduler.h
 *
 * TDMA slot scheduling utilities.
 * Slot timing is managed by the leader_election module; this module provides
 * supplementary scheduling primitives.
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <stdint.h>

void scheduler_init(void);
void scheduler_start(void);

#endif // SCHEDULER_H
