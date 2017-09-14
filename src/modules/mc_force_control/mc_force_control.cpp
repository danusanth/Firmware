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
#include <math.h>
#include <algorithm>

#include <uORB/uORB.h>
#include <uORB/topics/rotor_force.h>

extern "C" __EXPORT int mc_force_control_main(int argc, char *argv[]);

class MulticopterForceControl
{
public:
    /**
     * Constructor
     */
    MulticopterForceControl();

    /**
     * Destructor, also kills the main task
     */
    ~MulticopterForceControl();

    /**
     * Start the multicopter attitude control task.
     *
     * @return		OK on success.
     */
    int		start();

private:
    bool	_task_should_exit;		/**< if true, task_main() should exit */
    int         _control_task;                  /**< task handle */

    int		_force_sub;                     /**< rotor force subscription */

    struct rotor_force_s		_rotor_forces;		/**< incoming rotor force */

    const double  _lift_const = 5.9467E-07;      /**< rotor lift const based on system idef. of the quad */
    const double  _alpha_bound = 0.349065850000000;     /**< max/min rotor pitch angle of the quad */

    /**
     * Check for changes in rotor force inputs.
     */
    void		rotor_force_poll();

    /**
      *Calculate average motorspeed based on lift const and rotor pitch boundaries.
      *@return average motor speed
      */
    double              motorspeed_average(float arr[], int size);

    /**
      *Calculate quad pitch angles based on rotorspeed and force.
      *@return array with pitch angles [0,3,4965]
      */
    double*             pitch_angles(float arr[], int size, double rpm);

    /**
      *Calculate the motorspeed command based on motorspeed and pitch angle
      *@return motorspeed command [-1,1]
      */
    double              motorspeed_command(double pitch[], int size, double rpm);

    /**
     * Shim for calling task_main from task_create.
     */
    static void         task_main_trampoline(int argc, char *argv[]);

    /**
     * Main force control task.
     */
    void		task_main();
};

namespace mc_force_control
{

MulticopterForceControl	*f_control;
}

MulticopterForceControl::MulticopterForceControl() :

    _task_should_exit(false),
    _control_task(-1),

    /* subscriptions */
    _force_sub(-1),
    _rotor_forces{}
{

}

MulticopterForceControl::~MulticopterForceControl()
{
        if (_control_task != -1) {
                /* task wakes up every 100ms or so at the longest */
                _task_should_exit = true;

                /* wait for a second for the task to quit at our request */
                unsigned i = 0;

                do {
                        /* wait 20ms */
                        usleep(20000);

                        /* if we have given up, kill it */
                        if (++i > 50) {
                                px4_task_delete(_control_task);
                                break;
                        }
                } while (_control_task != -1);
        }

        mc_force_control::f_control = nullptr;
}

void
MulticopterForceControl::rotor_force_poll()
{
    bool updated;

    /* get rotor force inputs */
    orb_check(_force_sub, &updated);

    if (updated) {
            orb_copy(ORB_ID(rotor_force), _force_sub, &_rotor_forces);
    }
}

double
MulticopterForceControl::motorspeed_average(float arr[], int size){
    float forcemax = *std::max_element(arr,arr+size);
    float forcemin = *std::min_element(arr,arr+size);
    double omegamax = sqrt(fabs((double)forcemax)/(_lift_const*_alpha_bound));
    double omegamin = sqrt(fabs((double)forcemin)/(_lift_const*_alpha_bound));
    return std::max(omegamax,omegamin);
}

double*
MulticopterForceControl::pitch_angles(float arr[], int size, double rpm){
    static double pitch[4];
    if(rpm > 0){
        for(int i=0;i<size;i++){
            pitch[i] = ((double)arr[i]/(rpm*rpm*_lift_const));
        }
    }else{
        for(int i=0;i<size;i++){
            pitch[i] = 0;
        }
    }
    return pitch;
}

double
MulticopterForceControl::motorspeed_command(double pitch[], int size, double rpm){
    double sum = 0;
    for (int i = 0; i < size; i++){
        sum += pitch[i];
    }
    sum = sum /size;
    return ((rpm/(76.30+45.4928*sum))/50)-1;
}

void
MulticopterForceControl::task_main_trampoline(int argc, char *argv[])
{
    mc_force_control::f_control->task_main();
}

void
MulticopterForceControl::task_main()
{
    /*
     * do subscriptions
     */
    _force_sub = orb_subscribe(ORB_ID(rotor_force));

    px4_pollfd_struct_t poll_fds = {};
    poll_fds.events = POLLIN;

    while (!_task_should_exit) {
        poll_fds.fd = _force_sub;

        /* wait for up to 100ms for data */
        int pret = px4_poll(&poll_fds, 1, 100);

        /* timed out - periodic check for _task_should_exit */
        if (pret == 0) {
                continue;
        }

        /* this is undesirable but not much we can do - might want to flag unhappy status */
        if (pret < 0) {
                warn("mc force ctrl: poll error %d, %d", pret, errno);
                /* sleep a bit before next try */
                usleep(100000);
                continue;
        }

        if (poll_fds.revents & POLLIN) {

            //first check if offboard & publish if offboard

            rotor_force_poll();

            //calculate the pitch angles and motor speed command
            double* rotor_pitch_angles;
            double rpm = motorspeed_average(_rotor_forces.force,4);
            rotor_pitch_angles = pitch_angles(_rotor_forces.force,4,rpm);
            double command = motorspeed_command(rotor_pitch_angles,4,rpm);

            //publish actuator_controls command


            //for debugging
            PX4_INFO("Rotor angles:\t%8.4f\t%8.4f\t%8.4f\t%8.4f average speed: \t%8.4f command: \t%8.4f",
                    rotor_pitch_angles[0],
                    rotor_pitch_angles[1],
                    rotor_pitch_angles[2],
                    rotor_pitch_angles[3],
                    rpm,
                    command
                    );
        }


    }
    _control_task = -1;
}

int
MulticopterForceControl::start()
{
        ASSERT(_control_task == -1);

        /* start the task */
        _control_task = px4_task_spawn_cmd("mc_force_control",
                                           SCHED_DEFAULT,
                                           SCHED_PRIORITY_MAX - 5,
                                           1700,
                                           (px4_main_t)&MulticopterForceControl::task_main_trampoline,
                                           nullptr);

        if (_control_task < 0) {
                warn("task start failed");
                return -errno;
        }

        return OK;
}

int mc_force_control_main(int argc, char *argv[])
{
        if (argc < 2) {
                warnx("usage: mc_force_control {start|stop|status}");
                return 1;
        }

        if (!strcmp(argv[1], "start")) {

                if (mc_force_control::f_control != nullptr) {
                        warnx("already running");
                        return 1;
                }

                mc_force_control::f_control = new MulticopterForceControl;

                if (mc_force_control::f_control == nullptr) {
                        warnx("alloc failed");
                        return 1;
                }

                if (OK != mc_force_control::f_control->start()) {
                        delete mc_force_control::f_control;
                        mc_force_control::f_control = nullptr;
                        warnx("start failed");
                        return 1;
                }

                return 0;
        }

        if (!strcmp(argv[1], "stop")) {
                if (mc_force_control::f_control == nullptr) {
                        warnx("not running");
                        return 1;
                }

                delete mc_force_control::f_control;
                mc_force_control::f_control = nullptr;
                return 0;
        }

        if (!strcmp(argv[1], "status")) {
                if (mc_force_control::f_control) {
                        warnx("running");
                        return 0;

                } else {
                        warnx("not running");
                        return 1;
                }
        }

        warnx("unrecognized command");
        return 1;
}
