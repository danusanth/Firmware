/**
 * @file mc_force_control.c
 * Offboard controller for the variable pitch multicopter 
 *
 * @author Danusanth Sri <srikantd@student.ethz.ch>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/rotor_force.h>

extern "C" __EXPORT int mc_force_control_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int mc_force_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

/* Main Thread */
int mc_force_control_thread_main(int argc, char *argv[])
{
    /* welcome user (warnx prints a line, including an appended\n, with variable arguments */
    warnx("[mc_force_control] started");

    int force_sub_fd = orb_subscribe(ORB_ID(rotor_force));

    /* one could wait for multiple topics with this technique, just using one here */
    struct pollfd fds[1] = {};
    fds[0].fd = force_sub_fd;
    fds[0].events = POLLIN;

    while (!thread_should_exit) {

        int poll_ret = poll(fds, 1, 1000);

        if (poll_ret == 0)
        {
            warnx("Got no data within 100 milisecond");
        }
        else if (poll_ret < 0)
        {
                warnx("ERROR return value from poll(): %d", poll_ret);
        }
        else
        {
            if (fds[0].revents & POLLIN) {
                struct rotor_force_s input;
                orb_copy(ORB_ID(rotor_force), force_sub_fd, &input);

                warnx("Recieved Force");
             }
        }
    }

    printf("[mc_force_control] exiting.\n");
    thread_running = false;

    return 0;
}

/* Startup Functions */
static void
usage(const char *reason)
{
        if (reason) {
                fprintf(stderr, "%s\n", reason);
        }

        fprintf(stderr, "usage: mc_force_control {start|stop|status}\n\n");
        return;
}

int mc_force_control_main(int argc, char *argv[])
{
    if (argc < 2) {
            usage("missing command");
            return 1;
    }

    if (!strcmp(argv[1], "start")) {

            if (thread_running) {
                    printf("mc_force_control already running\n");
                    /* this is not an error */
                    return 0;
            }

            thread_should_exit = false;
            deamon_task = px4_task_spawn_cmd("mc_force_control",
                                             SCHED_DEFAULT,
                                             SCHED_PRIORITY_MAX - 5,
                                             2048,
                                             mc_force_control_thread_main,
                                             (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
            thread_running = true;
            return 0;
    }

    if (!strcmp(argv[1], "stop")) {
            thread_should_exit = true;
            return 0;
    }

    if (!strcmp(argv[1], "status")) {
            if (thread_running) {
                    printf("\tmc_force_control is running\n");

            } else {
                    printf("\tmc_force_control not started\n");
            }

            return 0;
    }

    usage("unrecognized command");
    return 1;
}
