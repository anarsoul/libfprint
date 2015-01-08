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


/* Thread pool
 *
 * The main thing is that we do not want to hi-jack the thread of a calling external
 * program or library. This also makes the libfprint more responsive and reliable.
 *
 * Callbacks from libusb should be routed through the threadpool before any significant
 * work is done.
 * Async calls to libfprint should be routed here also.
 * Internal calls might want to use the thread pool to avoid too many recursive calls
 * and a messed up stack.
 */

GThreadPool *thread_pool = NULL;

// Send task to thread pool
int fpi_event_push(enum fpi_event event, gpointer data)
{
    if (thread_pool == NULL)
    {
        thread_pool = g_thread_pool_new(fpi_thread_pool, NULL, 1, TRUE, NULL);
        if (thread_pool == 0)
            return -1;
    }

    struct fpi_event_data *event_data = g_malloc0(sizeof(*event_data)); // TODO: Error handling
    event_data->event = event;
    event_data->data = data;
    //event_data->user_data = user_data;
    //event_data->callback = callback;
    g_thread_pool_push(thread_pool, event_data, NULL); // TODO: Error handling.
    return 0;
}

void fpi_thread_pool (gpointer event_data, gpointer user_data)
{
    struct fpi_event_data *l_event_data = event_data;
    enum fpi_event event = l_event_data->event;
    gpointer data = l_event_data->data;
    //gpointer callback_user_data = l_event_data->user_data;
    struct fp_dev *dev = data;
    struct fp_driver *drv = NULL;
    struct fpi_ssm *machine = data;

    fp_dbg("%d", event);
    switch (event)
    {
    case FP_ASYNC_DEV_OPEN:
        drv = dev->drv;
        if (!drv->open)
        {
            fpi_drvcb_open_complete(dev, 0);
            break;
        }
        int r = drv->open(dev, dev->driver_data);
        if (r)
        {
            fpi_drvcb_open_complete(dev, r);
            libusb_close(dev->udev);
            g_free(dev);
        }
        break;

    case FP_ASYNC_DEV_CLOSE:
        drv = dev->drv;
        if (!drv->close)
        {
            fpi_drvcb_close_complete(dev);
            break;
        }

        dev->state = DEV_STATE_DEINITIALIZING;
        drv->close(dev);
        break;

    case FP_ASYNC_ENROLL_START:
        drv = dev->drv;
        r = drv->enroll_start(dev);
        if (r < 0) {
            fp_err("failed to start enrollment");
            dev->state = DEV_STATE_ERROR;
            fpi_drvcb_enroll_started(dev, r);
            dev->enroll_stage_cb = NULL;
        }
        break;

    case FP_ASYNC_ENROLL_STOP:
        drv = dev->drv;
        if (!drv->enroll_stop) {
            fpi_drvcb_enroll_stopped(dev);
            break;
        }

        r = drv->enroll_stop(dev);
        if (r < 0) {
            fp_err("failed to stop enrollment");
            fpi_drvcb_enroll_stopped(dev); // TODO: The callback is not aware of failure
            dev->enroll_stop_cb = NULL;
        }
        break;

    case FP_ASYNC_VERIFY_START:
        drv = dev->drv;
        r = drv->verify_start(dev);
        if (r < 0) {
            fp_err("failed to start verification, error %d", r);
            dev->state = DEV_STATE_ERROR;
            fpi_drvcb_verify_started(dev, r);
            dev->verify_cb = NULL;
        }
        break;

    case FP_ASYNC_VERIFY_STOP:
        drv = dev->drv;
        if (!drv->verify_stop) {
            dev->state = DEV_STATE_INITIALIZED;
            fpi_drvcb_verify_stopped(dev);
            break;
        }

        r = drv->verify_stop(dev, (dev->state == DEV_STATE_VERIFYING));
        if (r < 0) {
            fp_err("failed to stop verification");
            fpi_drvcb_verify_stopped(dev); // TODO: The callback is not aware of failure
            dev->verify_stop_cb = NULL;
        }
        break;

    case FP_ASYNC_IDENTIFY_START:
        drv = dev->drv;
        r = drv->identify_start(dev);
        if (r < 0) {
            fp_err("identify_start failed with error %d", r);
            dev->state = DEV_STATE_ERROR;
            fpi_drvcb_identify_started(dev, r);
            dev->identify_cb = NULL;
        }
        break;

    case FP_ASYNC_IDENTIFY_STOP:
        drv = dev->drv;
        if (!drv->identify_stop) {
            dev->state = DEV_STATE_INITIALIZED;
            fpi_drvcb_identify_stopped(dev);
            break;
        }

        r = drv->identify_stop(dev, (dev->state == DEV_STATE_VERIFYING));
        if (r < 0) {
            fp_err("failed to stop identification");
            fpi_drvcb_identify_stopped(dev); // TODO: The callback is not aware of failure
            dev->identify_stop_cb = NULL;
        }
        break;

    case FP_ASYNC_CAPTURE_START:
        drv = dev->drv;
        r = drv->capture_start(dev);
        if (r < 0) {
            dev->state = DEV_STATE_ERROR;
            fp_err("failed to start verification, error %d", r);
            fpi_drvcb_capture_started(dev, r);
            dev->capture_cb = NULL;
        }
        break;

    case FP_ASYNC_CAPTURE_STOP:
        drv = dev->drv;
        if (!drv->capture_stop) {
            dev->state = DEV_STATE_INITIALIZED;
            fpi_drvcb_capture_stopped(dev);
            break;
        }

        r = drv->capture_stop(dev);
        if (r < 0) {
            fp_err("failed to stop capture");
            fpi_drvcb_capture_stopped(dev); // TODO: The callback is not aware of failure
            dev->capture_stop_cb = NULL;
        }
        break;

    case FPI_EVENT_SSM_CALL_HANDLER:
        machine->handler(machine);      // TODO: We have race issues.
        break;

    case FPI_EVENT_SSM_CALLBACK:
        machine->callback(machine);
        break;
    }

    g_free(event_data);
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

/* Allocate a new ssm */
struct fpi_ssm *fpi_ssm_new(struct fp_dev *dev, ssm_handler_fn handler,
	int nr_states)
{
	struct fpi_ssm *machine;
	BUG_ON(nr_states < 1);

	machine = g_malloc0(sizeof(*machine));
	machine->handler = handler;
	machine->nr_states = nr_states;
	machine->dev = dev;
	machine->completed = TRUE;
	return machine;
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
    fpi_event_push(FPI_EVENT_SSM_CALL_HANDLER, machine);
}

/* Start a ssm. You can also restart a completed or aborted ssm. */
void fpi_ssm_start(struct fpi_ssm *ssm, ssm_completed_fn callback)
{
	BUG_ON(!ssm->completed);
	ssm->callback = callback;
	ssm->cur_state = 0;
	ssm->completed = FALSE;
	ssm->error = 0;
	__ssm_call_handler(ssm);
}

static void __subsm_complete(struct fpi_ssm *ssm)
{
	struct fpi_ssm *parent = ssm->parentsm;
	BUG_ON(!parent);
    ssm->parentsm->childsm = NULL;
	if (ssm->error)
		fpi_ssm_mark_aborted(parent, ssm->error);
	else
		fpi_ssm_next_state(parent);
	fpi_ssm_free(ssm);
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

/* Mark a ssm as completed successfully. */
void fpi_ssm_mark_completed(struct fpi_ssm *machine)
{
    BUG_ON(machine->childsm);
	BUG_ON(machine->completed);
	machine->idle = FALSE;
	machine->completed = TRUE;
	fp_dbg("%p completed with status %d", machine, machine->error);
	if (machine->callback)
        fpi_event_push(FPI_EVENT_SSM_CALLBACK, machine);
}

/* Mark a ssm as aborted with error. */
void fpi_ssm_mark_aborted(struct fpi_ssm *machine, int error)
{
    BUG_ON(machine->childsm);
	fp_dbg("error %d from state %d", error, machine->cur_state);
	BUG_ON(error == 0);
	machine->error = error;
	fpi_ssm_mark_completed(machine);
}

/* Iterate to next state of a ssm */
void fpi_ssm_next_state(struct fpi_ssm *machine)
{
    BUG_ON(machine->childsm);
	BUG_ON(machine->completed);
	machine->cur_state++;
	if (machine->cur_state == machine->nr_states) {
		fpi_ssm_mark_completed(machine);
	} else {
		__ssm_call_handler(machine);
	}
}

void fpi_ssm_jump_to_state(struct fpi_ssm *machine, int state)
{
    BUG_ON(machine->childsm);
	BUG_ON(machine->completed);
	BUG_ON(state >= machine->nr_states);
	machine->cur_state = state;
	__ssm_call_handler(machine);
}

void fpi_ssm_idle(struct fpi_ssm *ssm)
{
    /* This is really bad programming */
    ssm->idle = TRUE;
}

/* This should be called to cancel the ssm from the outside.
 * This is the kill signal. Do not try to change the state from
 * the outside if you are not aware of the state.
 * Raceconditions makes it unknown when the ssm actually aborts. */
void fpi_ssm_async_abort(struct fpi_ssm *ssm, int error)
{
    BUG_ON(ssm->completed);
    ssm->cancelling = TRUE;
    ssm->error = error;     // Should not be needed if there is a child.
    if(ssm->childsm)
        fpi_ssm_async_abort(ssm->childsm, error);
    else
        if (ssm->idle) {
            fp_dbg("The SSM (idle) has been asked to abort, complying");
            fpi_ssm_mark_completed(ssm);
        }
}

void fpi_ssm_async_complete(struct fpi_ssm *ssm)
{
    BUG_ON(ssm->completed);
    ssm->cancelling = TRUE;
    if(ssm->childsm)
        fpi_ssm_async_complete(ssm->childsm);
    else
        if (ssm->idle) {
            fp_dbg("The SSM (idle) has been asked to complete, complying");
            fpi_ssm_mark_completed(ssm);
        }
}
