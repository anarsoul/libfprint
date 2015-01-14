/*
 * Functions to assist with asynchronous driver <---> library communications
 * Copyright (C) 2007-2008 Daniel Drake <dsd@gentoo.org>
 * Copyright (C) 2015 Per-Ola Gustavsson <pelle@marsba.se>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#define FP_COMPONENT "drv"

#include <config.h>
#include <errno.h>

#include "fp_internal.h"

enum fpi_ssm_events {
    FPI_SSM_NEXT_STATE,
    FPI_SSM_JUMP_TO_STATE,
    FPI_SSM_MARK_COMPLETED,
    FPI_SSM_MARK_ABORTED,
    FPI_SSM_MARK_IDLE,
    FPI_SSM_ASYNC_ABORT,
};
struct __fpi_ssm_event {
    enum fpi_ssm_events event;
    int state;
    int error;
    struct fpi_ssm *ssm;
};

/* Invoke the state handler */
static void __ssm_call_handler(struct fpi_ssm *machine)
{
	machine->idle = FALSE;
    if (machine->cancelling) {
        fp_dbg("The SSM has been asked to abort, complying");
        fpi_ssm_mark_completed(machine);
        return;
    }
	fp_dbg("%p entering state %d", machine, machine->cur_state);
    machine->handler(machine);
}
static void __subsm_complete(struct fpi_ssm *ssm)
{
	struct fpi_ssm *parent = ssm->parentsm;
	int error = ssm->error;
	BUG_ON(!parent);
	fpi_ssm_free(ssm);
	if (error)
		fpi_ssm_mark_aborted(parent, error);
	else
		fpi_ssm_next_state(parent);
}
static void __fpi_ssm_thread (struct fp_dev *dev, gpointer user_data)
{
    struct __fpi_ssm_event *data =  user_data;
    struct fpi_ssm *ssm = data->ssm;
    int state = data->state;
    int error = data->error;

    switch (data->event) {
    case FPI_SSM_NEXT_STATE:
        BUG_ON(ssm->childsm);
        BUG_ON(ssm->completed);
        ssm->cur_state++;
        if (ssm->cur_state == ssm->nr_states) {
            fpi_ssm_mark_completed(ssm);
        } else {
            __ssm_call_handler(ssm);
        }
        break;

    case FPI_SSM_JUMP_TO_STATE:
        BUG_ON(ssm->childsm);
        BUG_ON(ssm->completed);
        BUG_ON(state >= ssm->nr_states);
        ssm->cur_state = state;
        __ssm_call_handler(ssm);
        break;

    case FPI_SSM_MARK_COMPLETED:
        BUG_ON(ssm->completed);
        if(ssm->childsm) {
            fp_dbg("The SSM (with child) has been asked to complete, complying");
            fpi_ssm_mark_completed(ssm->childsm);
        } else {
            ssm->completed = TRUE;
            if (ssm->callback) {
                fp_dbg("%p completed with status %d", ssm, ssm->error);
                ssm->callback(ssm);
            }
        }
        break;

    case FPI_SSM_MARK_ABORTED:
        BUG_ON(ssm->childsm);
        fp_dbg("error %d from state %d", error, ssm->cur_state);
        BUG_ON(error == 0);
        if(ssm->childsm) {
            fp_dbg("The SSM (with child) has been asked to abort, complying");
            fpi_ssm_mark_aborted(ssm->childsm, error);
        } else {
            ssm->error = error;
            ssm->completed = TRUE;
            if (ssm->callback) {
                fp_dbg("%p completed with status %d", ssm, ssm->error);
                ssm->callback(ssm);
            }
        }
        break;

    case FPI_SSM_ASYNC_ABORT:
        if(ssm->abort_handler) {
            fp_dbg("Calling abort handler");
            ssm->abort_handler(ssm, error);
        }
        else {
            if(ssm->idle)
                if(error)
                    fpi_ssm_mark_aborted(ssm, error);
                else
                    fpi_ssm_mark_completed(ssm);
            else
                ssm->cancelling = TRUE;
        }
        break;

    case FPI_SSM_MARK_IDLE:
        fp_dbg("Idle");
        ssm->idle = TRUE;
        break;

    default:
        BUG_ON(TRUE);
        break;
    }
    g_slice_free(struct __fpi_ssm_event, data);
}

static void __fpi_ssm_event_push (struct fpi_ssm *ssm, enum fpi_ssm_events event, int error, int state)
{
    struct __fpi_ssm_event *data = g_slice_new(struct __fpi_ssm_event); // TODO: Error handling.
    data->error = error;
    data->event = event;
    data->ssm = ssm;
    data->state = state;
    fpi_event_push_custom(ssm->dev, data, __fpi_ssm_thread);
}



/* SSM: sequential state machine
 * Asynchronous driver design encourages some kind of state machine behind it.
 * In most cases, the state machine is entirely linear - you only go to the
 * next state, you never jump or go backwards. The SSM functions help you
 * implement such a machine.
 *
 * e.g. S1 --> S2 --> S3 --> S4
 * S1 is the start state
 * There is also an implicit error state and an implicit accepting state
 * (both with implicit edges from every state).
 *
 * You can also jump to any arbitrary state (while marking completion of the
 * current state) while the machine is running. In other words there are
 * implicit edges linking one state to every other state. OK, we're stretching
 * the "state machine" description at this point.
 *
 * To create a ssm, you pass a state handler function and the total number of
 * states (4 in the above example).
 *
 * To start a ssm, you pass in a completion callback function which gets
 * called when the ssm completes (both on error and on failure).
 *
 * To iterate to the next state, call fpi_ssm_next_state(). It is legal to
 * attempt to iterate beyond the final state - this is equivalent to marking
 * the ssm as successfully completed.
 *
 * To mark successful completion of a SSM, either iterate beyond the final
 * state or call fpi_ssm_mark_completed() from any state.
 *
 * To mark failed completion of a SSM, call fpi_ssm_mark_aborted() from any
 * state. You must pass a non-zero error code.
 *
 * Your state handling function looks at ssm->cur_state in order to determine
 * the current state and hence which operations to perform (a switch statement
 * is appropriate).
 * Typically, the state handling function fires off an asynchronous libusb
 * transfer, and the callback function iterates the machine to the next state
 * upon success (or aborts the machine on transfer failure).
 *
 * Your completion callback should examine ssm->error in order to determine
 * whether the ssm completed or failed. An error code of zero indicates
 * successful completion.
 */

/* Allocate a new ssm with an abort handler */
/* TODO: Mostly pointless. */
struct fpi_ssm *fpi_ssm_new_a(struct fp_dev *dev, ssm_handler_fn handler,
	ssm_abort_fn abort_fn, int nr_states)
{
	struct fpi_ssm *machine;
	BUG_ON(nr_states < 1);

	machine = g_malloc0(sizeof(*machine));
	machine->handler = handler;
	machine->abort_handler;
	machine->nr_states = nr_states;
	machine->dev = dev;
	machine->completed = TRUE;
	return machine;
}
/* Allocate a new ssm with an abort handler */
struct fpi_ssm *fpi_ssm_new(struct fp_dev *dev, ssm_handler_fn handler,
	int nr_states)
{
    return fpi_ssm_new_a(dev, handler, NULL, nr_states);
}

/* Free a ssm */
void fpi_ssm_free(struct fpi_ssm *machine)
{
	if (!machine)
		return;
    if(machine->parentsm)
        machine->parentsm->childsm = NULL;
	g_free(machine);
}

/* Start a ssm. You can also restart a completed or aborted ssm. */
void fpi_ssm_start(struct fpi_ssm *ssm, ssm_completed_fn callback)
{
	BUG_ON(!ssm->completed);
	ssm->callback = callback;
	ssm->completed = FALSE;
	ssm->error = 0;
	ssm->cur_state = 0;
	fpi_ssm_jump_to_state(ssm, 0);
}


/* start a SSM as a child of another. if the child completes successfully, the
 * parent will be advanced to the next state. if the child aborts, the parent
 * will be aborted with the same error code. the child will be automatically
 * freed upon completion/abortion. */
void fpi_ssm_start_subsm(struct fpi_ssm *parent, struct fpi_ssm *child)
{
    BUG_ON(parent->childsm);
	child->parentsm = parent;
	parent->childsm = child;
	fpi_ssm_start(child, __subsm_complete);
}

/* The following must be thread-safe */

/* Mark a ssm as completed successfully. */
void fpi_ssm_mark_completed(struct fpi_ssm *machine)
{
    __fpi_ssm_event_push(machine, FPI_SSM_MARK_COMPLETED, 0, 0);
}

/* Mark a ssm as aborted with error. */
void fpi_ssm_mark_aborted(struct fpi_ssm *machine, int error)
{
    __fpi_ssm_event_push(machine, FPI_SSM_MARK_ABORTED, error, 0);
}

/* Iterate to next state of a ssm */
void fpi_ssm_next_state(struct fpi_ssm *machine)
{
    __fpi_ssm_event_push(machine, FPI_SSM_NEXT_STATE, 0, 0);
}

void fpi_ssm_jump_to_state(struct fpi_ssm *machine, int state)
{
    __fpi_ssm_event_push(machine, FPI_SSM_JUMP_TO_STATE, 0, state);
}

void fpi_ssm_mark_idle(struct fpi_ssm *ssm)
{
    /* Not sure this is appropriate programming */
    __fpi_ssm_event_push(ssm, FPI_SSM_MARK_IDLE, 0, 0);
}

int fpi_ssm_get_current_state(struct fpi_ssm *ssm)
{
    return ssm->cur_state;
}
/* This should be called to cancel the ssm if the current thread is unknown.
 * This is the kill signal. Do not try to change the state from
 * the outside if you are not aware of the state.
 * Raceconditions makes it unknown when the ssm actually aborts. */
void fpi_ssm_async_abort(struct fpi_ssm *ssm, int error)
{
    BUG_ON(ssm->completed);
    if(ssm->childsm) {
        fp_dbg("The SSM has been asynchronously asked to abort, passing on the task to the child...");
        fpi_ssm_async_abort(ssm->childsm, error);
    }
    else {
        fp_dbg("The SSM has been asynchronously asked to abort, passing on the task...");
        __fpi_ssm_event_push(ssm, FPI_SSM_ASYNC_ABORT, error, 0);
    }
}

void fpi_ssm_async_complete(struct fpi_ssm *ssm)
{
    BUG_ON(ssm->completed);
    if(ssm->childsm) {
        fp_dbg("The SSM has been asynchronously asked to abort, passing on the task to the child...");
        fpi_ssm_async_complete(ssm->childsm);
    }
    else {
        fp_dbg("The SSM has been asynchronously asked to complete, passing on the task...");
        __fpi_ssm_event_push(ssm, FPI_SSM_ASYNC_ABORT, 0, 0);
    }
}

