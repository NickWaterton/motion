/*    track.c
 *
 *    Experimental motion tracking.
 *
 *    Copyright 2000, Jeroen Vreeken
 *    This program is published under the GNU Public license
 */
#include <math.h>
#include "motion.h"
#include <curl/curl.h>

#ifdef HAVE_V4L2
#include <linux/videodev2.h>
#include "pwc-ioctl.h"
#endif


struct trackoptions track_template = {
    .dev =             -1,             /* dev open */
    .port =            NULL,           /* char *port */
    .motorx =          0,              /* int motorx */
    .motory =          0,              /* int motory */
    .maxx =            0,              /* int maxx; */
    .maxy =            0,              /* int maxy; */
    .minx =            0,              /* int minx; */
    .miny =            0,              /* int miny; */
    .homex =           128,            /* int homex; */
    .homey =           128,            /* int homey; */
    .motorx_reverse =  0,              /* int reversed x servo; */
    .motory_reverse =  0,              /* int reversed y servo; */
    .speed =           TRACK_SPEED,    /* speed */
    .stepsize =        TRACK_STEPSIZE, /* stepsize */
    .active =          0,              /* auto tracking active */
    .minmaxfound =     0,              /* flag for minmax values stored for pwc based camera */
    .step_angle_x =    10,             /* UVC step angle in degrees X-axis that camera moves during auto tracking */
    .step_angle_y =    10,             /* UVC step angle in degrees Y-axis that camera moves during auto tracking */
    .move_wait =       10,              /* number of frames to disable motion detection after camera moving */
    .track_home_pos_name = FOSCAM_HD_HOME       /* char *track_home_pos_name home position name for Foscam ("0" if undefined) */
};




/* Add your own center and move functions here: */

static unsigned int servo_position(struct context *cnt, unsigned int motor);

static unsigned int servo_center(struct context *cnt, int xoff, int yoff ATTRIBUTE_UNUSED);
static unsigned int stepper_center(struct context *cnt, int xoff, int yoff ATTRIBUTE_UNUSED);
static unsigned int iomojo_center(struct context *cnt, int xoff, int yoff);

static unsigned int stepper_move(struct context *cnt, struct coord *cent, struct images *imgs);
static unsigned int servo_move(struct context *cnt, struct coord *cent,
                                     struct images *imgs, unsigned int manual);
static unsigned int iomojo_move(struct context *cnt, int dev, struct coord *cent, struct images *imgs);
//foscam
static unsigned int FOSCAM_HD_center(struct context *cnt, int xoff, int yoff);
static unsigned int FOSCAM_HD_move(struct context *cnt, struct coord *cent, struct images *imgs);

#ifdef HAVE_V4L2
static unsigned int lqos_center(struct context *cnt, int dev, int xoff, int yoff);
static unsigned int lqos_move(struct context *cnt, int dev, struct coord *cent,
                                    struct images *imgs, unsigned int manual);
static unsigned int uvc_center(struct context *cnt, int dev, int xoff, int yoff);
static unsigned int uvc_move(struct context *cnt, int dev, struct coord *cent,
                                   struct images *imgs, unsigned int manual);
#endif /* HAVE_V4L2 */

/* Add a call to your functions here: */
unsigned int track_center(struct context *cnt, int dev ATTRIBUTE_UNUSED,
                                unsigned int manual, int xoff, int yoff)
{
    if (!manual && !cnt->track.active)
        return 0;

    if (cnt->track.type == TRACK_TYPE_STEPPER) {
        unsigned int ret;
        ret = stepper_center(cnt, xoff, yoff);
        if (!ret) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: internal error");
            return 0;
        }
        else return ret;
    } else if (cnt->track.type == TRACK_TYPE_SERVO) {
        return servo_center(cnt, xoff, yoff);
    }
#ifdef HAVE_V4L2
    else if (cnt->track.type == TRACK_TYPE_PWC)
        return lqos_center(cnt, dev, xoff, yoff);
    else if (cnt->track.type == TRACK_TYPE_UVC)
        return uvc_center(cnt, dev, xoff, yoff);
#endif
    else if (cnt->track.type == TRACK_TYPE_IOMOJO)
        return iomojo_center(cnt, xoff, yoff);
    else if (cnt->track.type == TRACK_TYPE_FOSCAM_HD)
        return FOSCAM_HD_center(cnt, xoff, yoff);
    else if (cnt->track.type == TRACK_TYPE_GENERIC)
        return 10; // FIX ME. I chose to return something reasonable.

    MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: internal error, %hu is not a known track-type",
               cnt->track.type);

    return 0;
}

/* Add a call to your functions here: */
unsigned int track_move(struct context *cnt, int dev, struct coord *cent, struct images *imgs,
                              unsigned int manual)
{

    if (!manual && !cnt->track.active)
        return 0;

    if (cnt->track.type == TRACK_TYPE_STEPPER)
        return stepper_move(cnt, cent, imgs);
    else if (cnt->track.type == TRACK_TYPE_SERVO)
        return servo_move(cnt, cent, imgs, manual);
#ifdef HAVE_V4L2
    else if (cnt->track.type == TRACK_TYPE_PWC)
        return lqos_move(cnt, dev, cent, imgs, manual);
    else if (cnt->track.type == TRACK_TYPE_UVC)
        return uvc_move(cnt, dev, cent, imgs, manual);
#endif
    else if (cnt->track.type == TRACK_TYPE_IOMOJO)
        return iomojo_move(cnt, dev, cent, imgs);
    else if (cnt->track.type == TRACK_TYPE_FOSCAM_HD)
        return FOSCAM_HD_move(cnt, cent, imgs);
    else if (cnt->track.type == TRACK_TYPE_GENERIC)
        return cnt->track.move_wait; // FIX ME. I chose to return something reasonable.

    MOTION_LOG(WRN, TYPE_TRACK, SHOW_ERRNO, "%s: internal error, %hu is not a known track-type",
               cnt->track.type);

    return 0;
}

/******************************************************************************
    Stepper motor on serial port
    http://www.lavrsen.dk/twiki/bin/view/Motion/MotionTracking
    http://www.lavrsen.dk/twiki/bin/view/Motion/MotionTrackerAPI
******************************************************************************/

static unsigned int stepper_command(struct context *cnt, unsigned int motor,
                                    unsigned int command, unsigned int data)
{
    char buffer[3];
    time_t timeout = time(NULL);

    buffer[0] = motor;
    buffer[1] = command;
    buffer[2] = data;

    if (write(cnt->track.dev, buffer, 3) != 3) {
        MOTION_LOG(NTC, TYPE_TRACK, SHOW_ERRNO, "%s: port %s dev fd %i, motor %hu command %hu data %hu",
                   cnt->track.port, cnt->track.dev, motor, command, data);
        return 0;
    }

    while (read(cnt->track.dev, buffer, 1) != 1 && time(NULL) < timeout + 1);

    if (time(NULL) >= timeout + 2) {
        MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Status byte timeout!");
        return 0;
    }

    return buffer[0];
}


static unsigned int stepper_status(struct context *cnt, unsigned int motor)
{
    return stepper_command(cnt, motor, STEPPER_COMMAND_STATUS, 0);
}


static unsigned int stepper_center(struct context *cnt, int x_offset, int y_offset)
{
    struct termios adtio;

    if (cnt->track.dev < 0) {
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Try to open serial device %s", cnt->track.port);

        if ((cnt->track.dev = open(cnt->track.port, O_RDWR | O_NOCTTY)) < 0) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Unable to open serial device %s",
                       cnt->track.port);
            return 0;
        }

        bzero (&adtio, sizeof(adtio));
        adtio.c_cflag= STEPPER_BAUDRATE | CS8 | CLOCAL | CREAD;
        adtio.c_iflag= IGNPAR;
        adtio.c_oflag= 0;
        adtio.c_lflag= 0;    /* non-canon, no echo */
        adtio.c_cc[VTIME] = 0;    /* timer unused */
        adtio.c_cc[VMIN] = 0;    /* blocking read until 1 char */
        tcflush (cnt->track.dev, TCIFLUSH);

        if (tcsetattr(cnt->track.dev, TCSANOW, &adtio) < 0) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Unable to initialize serial device %s",
                       cnt->track.port);
            cnt->track.dev = -1;
            return 0;
        }
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Opened serial device %s and initialize, fd %i",
                   cnt->track.port, cnt->track.dev);
    }

    /* x-axis */

    stepper_command(cnt, cnt->track.motorx, STEPPER_COMMAND_SPEED, cnt->track.speed);
    stepper_command(cnt, cnt->track.motorx, STEPPER_COMMAND_LEFT_N, cnt->track.maxx);

    while (stepper_status(cnt, cnt->track.motorx) & STEPPER_STATUS_LEFT);

    stepper_command(cnt, cnt->track.motorx, STEPPER_COMMAND_RIGHT_N,
                    cnt->track.maxx / 2 + x_offset * cnt->track.stepsize);

    while (stepper_status(cnt, cnt->track.motorx) & STEPPER_STATUS_RIGHT);

    /* y-axis */

    stepper_command(cnt, cnt->track.motory, STEPPER_COMMAND_SPEED, cnt->track.speed);
    stepper_command(cnt, cnt->track.motory, STEPPER_COMMAND_UP_N, cnt->track.maxy);

    while (stepper_status(cnt, cnt->track.motory) & STEPPER_STATUS_UP)

    stepper_command(cnt, cnt->track.motory, STEPPER_COMMAND_DOWN_N,
                    cnt->track.maxy / 2 + y_offset * cnt->track.stepsize);

    while (stepper_status(cnt, cnt->track.motory) & STEPPER_STATUS_DOWN);

    return cnt->track.move_wait;
}

static unsigned int stepper_move(struct context *cnt,
                                       struct coord *cent, struct images *imgs)
{
    unsigned int command = 0, data = 0;

    if (cnt->track.dev < 0) {
        MOTION_LOG(WRN, TYPE_TRACK, NO_ERRNO, "%s: No device %s started yet , trying stepper_center()",
                   cnt->track.port);

        if (!stepper_center(cnt, 0, 0)) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: failed to initialize stepper device on %s , fd [%i].",
                       cnt->track.port, cnt->track.dev);
            return 0;
        }

        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: succeed , device started %s , fd [%i]",
                    cnt->track.port, cnt->track.dev);
    }

    /* x-axis */

    if (cent->x < imgs->width / 2) {
        command = STEPPER_COMMAND_LEFT_N;
        data = imgs->width / 2 - cent->x;
    }

    if (cent->x > imgs->width / 2) {
        command = STEPPER_COMMAND_RIGHT_N;
        data = cent->x - imgs->width / 2;
    }

    data = data * cnt->track.stepsize / imgs->width;

    if (data) stepper_command(cnt, cnt->track.motorx, command, data);

    /* y-axis */

    if (cent->y < imgs->height / 2) {
        command = STEPPER_COMMAND_UP_N;
        data = imgs->height / 2 - cent->y;
    }

    if (cent->y > imgs->height / 2) {
        command = STEPPER_COMMAND_DOWN_N;
        data = cent->y - imgs->height / 2;
    }

    data = data * cnt->track.stepsize / imgs->height;

    if (data)
        stepper_command(cnt, cnt->track.motory, command, data);


    return cnt->track.move_wait;
}

/******************************************************************************
 *   Servo motor on serial port
 *   http://www.lavrsen.dk/twiki/bin/view/Motion/MotionTracking
 *   http://www.lavrsen.dk/twiki/bin/view/Motion/MotionTrackerServoAPI
 ******************************************************************************/

static int servo_open(struct context *cnt)
{
    struct termios adtio;

    if ((cnt->track.dev = open(cnt->track.port, O_RDWR | O_NOCTTY)) < 0) {
        MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Unable to open serial device %s",
                   cnt->track.port);
        return 0;
    }

    bzero (&adtio, sizeof(adtio));
    adtio.c_cflag= SERVO_BAUDRATE | CS8 | CLOCAL | CREAD;
    adtio.c_iflag= IGNPAR;
    adtio.c_oflag= 0;
    adtio.c_lflag= 0;       /* non-canon, no echo */
    adtio.c_cc[VTIME] = 0;  /* timer unused */
    adtio.c_cc[VMIN] = 0;   /* blocking read until 1 char */
    tcflush (cnt->track.dev, TCIFLUSH);

    if (tcsetattr(cnt->track.dev, TCSANOW, &adtio) < 0) {
        MOTION_LOG(ERR, TYPE_TRACK, NO_ERRNO, "%s: Unable to initialize serial device %s",
                   cnt->track.port);
        cnt->track.dev = -1;
        return 0;
    }

    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Opened serial device %s and initialize, fd %i",
               cnt->track.port, cnt->track.dev);

    return 1;
}


static unsigned int servo_command(struct context *cnt, unsigned int motor,
                                  unsigned int command, unsigned int data)
{
    unsigned char buffer[3];
    time_t timeout = time(NULL);

    buffer[0] = motor;
    buffer[1] = command;
    buffer[2] = data;


    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: SENDS port %s dev fd %i, motor %hu command %hu data %hu",
               cnt->track.port, cnt->track.dev, buffer[0], buffer[1], buffer[2]);

    if (write(cnt->track.dev, buffer, 3) != 3) {
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: port %s dev fd %i, motor %hu command %hu data %hu",
                   cnt->track.port, cnt->track.dev, motor, command, data);
        return 0;
    }

    while (read(cnt->track.dev, buffer, 1) != 1 && time(NULL) < timeout + 1);

    if (time(NULL) >= timeout + 2) {
        MOTION_LOG(ERR, TYPE_TRACK, NO_ERRNO, "%s: Status byte timeout!");
        return 0;
    }

    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Command return %d", buffer[0]);


    return buffer[0];
}


static unsigned int servo_position(struct context *cnt, unsigned int motor)
{
    unsigned int ret = 0;

    ret = servo_command(cnt, motor, SERVO_COMMAND_POSITION, 0);

    return ret;
}


/**
 * servo_move
 *      Does relative movements to current position.
 *
 */
static unsigned int servo_move(struct context *cnt, struct coord *cent,
                                     struct images *imgs, unsigned int manual)
{
    unsigned int command = 0;
    unsigned int data = 0;
    unsigned int position;

    /* If device is not open yet , open and center */
    if (cnt->track.dev < 0) {
        if (!servo_center(cnt, 0, 0)) {
            MOTION_LOG(ERR, TYPE_TRACK, NO_ERRNO, "%s: Problem opening servo!");
            return 0;
        }
    }

    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cent->x %d, cent->y %d, reversex %d,"
               "reversey %d manual %d", cent->x , cent->y,
               cnt->track.motorx_reverse, cnt->track.motory_reverse, manual);

    if (manual) {
        int offset;

        if (cent->x) {
            position = servo_position(cnt, cnt->track.motorx);
            offset = cent->x * cnt->track.stepsize;


            if ((cnt->track.motorx_reverse && (offset > 0)) ||
                (!cnt->track.motorx_reverse && (offset < 0)))
                command = SERVO_COMMAND_LEFT_N;
            else
                command = SERVO_COMMAND_RIGHT_N;

            data = abs(offset);

            if ((data + position > (unsigned)cnt->track.maxx) ||
                (position - offset < (unsigned)cnt->track.minx)) {
                MOTION_LOG(ERR, TYPE_TRACK, NO_ERRNO, "%s: x %d value out of range! (%d - %d)",
                           data, cnt->track.minx, cnt->track.maxx);
                return 0;
            }

            /* Set Speed , TODO : it should be done only when speed changes */
            servo_command(cnt, cnt->track.motorx, SERVO_COMMAND_SPEED, cnt->track.speed);
            servo_command(cnt, cnt->track.motorx, command, data);
        }


        if (cent->y) {
            position = servo_position(cnt, cnt->track.motory);
            offset = cent->y * cnt->track.stepsize;

            if ((cnt->track.motory_reverse && (offset > 0)) ||
                (!cnt->track.motory_reverse && (offset < 0)))
                command = SERVO_COMMAND_UP_N;
            else
                command = SERVO_COMMAND_DOWN_N;

            data = abs(offset);

            if ((data + position > (unsigned)cnt->track.maxy) ||
                (position - offset < (unsigned)cnt->track.miny)) {
                MOTION_LOG(ERR, TYPE_TRACK, NO_ERRNO, "%s: y %d value out of range! (%d - %d)",
                           data, cnt->track.miny, cnt->track.maxy);
                return 0;
            }

            /* Set Speed , TODO : it should be done only when speed changes */
            servo_command(cnt, cnt->track.motory, SERVO_COMMAND_SPEED, cnt->track.speed);
            servo_command(cnt, cnt->track.motory, command, data);
        }

    } else {
        /***** x-axis *****/

        /* Move left */
        if (cent->x < imgs->width / 2) {
            if (cnt->track.motorx_reverse)
                command = SERVO_COMMAND_RIGHT_N;
            else
                command = SERVO_COMMAND_LEFT_N;
            data = imgs->width / 2 - cent->x;
        }

        /* Move right */
        if (cent->x > imgs->width / 2) {
            if (cnt->track.motorx_reverse)
                command = SERVO_COMMAND_LEFT_N;
            else
                command = SERVO_COMMAND_RIGHT_N;
            data = cent->x - imgs->width / 2;
        }


        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: X offset %d", data);

        data = data * cnt->track.stepsize / imgs->width;

        if (data && command) {

            // TODO: need to get position to avoid overflow limits
            position = servo_position(cnt, cnt->track.motorx);

            if ((position + data > (unsigned)cnt->track.maxx) ||
                (position - data < (unsigned)cnt->track.minx)) {
                MOTION_LOG(ERR, TYPE_TRACK, NO_ERRNO, "%s: x %d value out of range! (%d - %d)",
                           data, cnt->track.minx, cnt->track.maxx);
                return 0;
            }

            /* Set Speed , TODO : it should be done only when speed changes */

            servo_command(cnt, cnt->track.motorx, SERVO_COMMAND_SPEED, cnt->track.speed);
            servo_command(cnt, cnt->track.motorx, command, data);

            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: X cent->x %d, cent->y %d, reversex %d,"
                       "reversey %d motorx %d data %d command %d",
                       cent->x, cent->y, cnt->track.motorx_reverse,
                       cnt->track.motory_reverse, cnt->track.motorx, data, command);
        }

        /***** y-axis *****/

        /* Move down */
        if (cent->y < imgs->height / 2) {
            if (cnt->track.motory_reverse)
                command = SERVO_COMMAND_UP_N;
            else
                command = SERVO_COMMAND_DOWN_N;
            data = imgs->height / 2 - cent->y;
        }

        /* Move up */
        if (cent->y > imgs->height / 2) {
            if (cnt->track.motory_reverse)
                command = SERVO_COMMAND_DOWN_N;
            else
                command = SERVO_COMMAND_UP_N;
            data = cent->y - imgs->height / 2;
        }

        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Y offset %d", data);

        data = data * cnt->track.stepsize / imgs->height;

        if (data && command) {

            // TODO: need to get position to avoid overflow limits
            position = servo_position(cnt, cnt->track.motory);

            if ((position + data > (unsigned)cnt->track.maxy) ||
                (position - data < (unsigned)cnt->track.miny)) {
                MOTION_LOG(ERR, TYPE_TRACK, NO_ERRNO, "%s: y %d value out of range! (%d - %d)",
                           data, cnt->track.miny, cnt->track.maxy);
                return 0;
            }

            /* Set Speed , TODO : it should be done only when speed changes */
            servo_command(cnt, cnt->track.motory, SERVO_COMMAND_SPEED, cnt->track.speed);
            servo_command(cnt, cnt->track.motory, command, data);

            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Y cent->x %d, cent->y %d, reversex %d,"
                       "reversey %d motory %d data %d command %d",
                        cent->x, cent->y, cnt->track.motorx_reverse,
                        cnt->track.motory_reverse, cnt->track.motory, command);
        }
    }

    return cnt->track.move_wait;
}

#if 0
static unsigned int servo_status(struct context *cnt, unsigned int motor)
{
    return servo_command(cnt, motor, SERVO_COMMAND_STATUS, 0);
}
#endif

/**
 * servo_center
 *      Moves servo to home position.
 *      Does absolute movements ( offsets relative to home position ).
 *
 *      Note : Using Clockwise as a convention for right , left , up , down
 *              so left minx , right maxx , down miny , up maxy
 *
 */

static unsigned int servo_center(struct context *cnt, int x_offset, int y_offset)
{
    int x_offset_abs;
    int y_offset_abs;

    /* If device is not open yet */
    if (cnt->track.dev < 0) {
        if (!servo_open(cnt)) {
            MOTION_LOG(ERR, TYPE_TRACK, NO_ERRNO, "%s: Problem opening servo!");
            return 0;
        }
    }

    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: X-offset %d, Y-offset %d, x-position %d. y-position %d,"
               "reversex %d, reversey %d , stepsize %d", x_offset, y_offset,
               cnt->track.homex + (x_offset * cnt->track.stepsize),
               cnt->track.homey + (y_offset * cnt->track.stepsize),
               cnt->track.motorx_reverse, cnt->track.motory_reverse,
               cnt->track.stepsize);

    /* x-axis */
    if (cnt->track.motorx_reverse)
        x_offset_abs = (128 - cnt->track.homex) - (x_offset * cnt->track.stepsize) + 128;
    else
        x_offset_abs = cnt->track.homex + (x_offset * cnt->track.stepsize);

    if (x_offset_abs <= cnt->track.maxx  && x_offset_abs >= cnt->track.minx) {
        /* Set Speed , TODO : it should be done only when speed changes */
        servo_command(cnt, cnt->track.motorx, SERVO_COMMAND_SPEED, cnt->track.speed);
        servo_command(cnt, cnt->track.motorx, SERVO_COMMAND_ABSOLUTE, x_offset_abs);
    }

    /* y-axis */
    if (cnt->track.motory_reverse)
        y_offset_abs = (128 - cnt->track.homey) - (y_offset * cnt->track.stepsize) + 128;
    else
        y_offset_abs = cnt->track.homey + (y_offset * cnt->track.stepsize);

    if (y_offset_abs <= cnt->track.maxy && y_offset_abs >= cnt->track.minx) {
        /* Set Speed , TODO : it should be done only when speed changes */
        servo_command(cnt, cnt->track.motory, SERVO_COMMAND_SPEED, cnt->track.speed);
        servo_command(cnt, cnt->track.motory, SERVO_COMMAND_ABSOLUTE, y_offset_abs);
    }

    return cnt->track.move_wait;
}


/******************************************************************************

    Iomojo Smilecam on serial port

******************************************************************************/

static char iomojo_command(struct context *cnt, char *command, int len, unsigned int ret)
{
    char buffer[1];
    time_t timeout = time(NULL);

    if (write(cnt->track.dev, command, len) != len)
        return 0;

    if (ret) {
        while (read(cnt->track.dev, buffer, 1) != 1 && time(NULL) < timeout + 2);

        if (time(NULL) >= timeout + 2) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Return byte timeout!");
            return 0;
        }
    }
    /* range values ? */
    return buffer[0];
}

static void iomojo_setspeed(struct context *cnt, unsigned int speed)
{
    char command[3];

    command[0] = IOMOJO_SETSPEED_CMD;
    command[1] = cnt->track.iomojo_id;
    command[2] = speed;

    if (iomojo_command(cnt, command, 3, 1) != IOMOJO_SETSPEED_RET)
        MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Unable to set camera speed");
}

static void iomojo_movehome(struct context *cnt)
{
    char command[2];

    command[0] = IOMOJO_MOVEHOME;
    command[1] = cnt->track.iomojo_id;

    iomojo_command(cnt, command, 2, 0);
}

static unsigned int iomojo_center(struct context *cnt, int x_offset, int y_offset)
{
    struct termios adtio;
    char command[5], direction = 0;

    if (cnt->track.dev < 0) {
        if ((cnt->track.dev = open(cnt->track.port, O_RDWR | O_NOCTTY)) < 0) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Unable to open serial device %s",
                       cnt->track.port);
            return 0;
        }

        bzero (&adtio, sizeof(adtio));
        adtio.c_cflag = IOMOJO_BAUDRATE | CS8 | CLOCAL | CREAD;
        adtio.c_iflag = IGNPAR;
        adtio.c_oflag = 0;
        adtio.c_lflag = 0;      /* non-canon, no echo */
        adtio.c_cc[VTIME] = 0;  /* timer unused */
        adtio.c_cc[VMIN] = 0;   /* blocking read until 1 char */
        tcflush(cnt->track.dev, TCIFLUSH);
        if (tcsetattr(cnt->track.dev, TCSANOW, &adtio) < 0) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Unable to initialize serial device %s",
                       cnt->track.port);
            return 0;
        }
    }

    iomojo_setspeed(cnt, 40);
    iomojo_movehome(cnt);

    if (x_offset || y_offset) {
        if (x_offset > 0) {
            direction |= IOMOJO_DIRECTION_RIGHT;
        } else {
            direction |= IOMOJO_DIRECTION_LEFT;
            x_offset *= -1;
        }

        if (y_offset > 0) {
            direction |= IOMOJO_DIRECTION_UP;
        } else {
            direction |= IOMOJO_DIRECTION_DOWN;
            y_offset *= -1;
        }

        if (x_offset > 180)
            x_offset = 180;

        if (y_offset > 60)
            y_offset = 60;

        command[0] = IOMOJO_MOVEOFFSET_CMD;
        command[1] = cnt->track.iomojo_id;
        command[2] = direction;
        command[3] = x_offset;
        command[4] = y_offset;
        iomojo_command(cnt, command, 5, 0);
    }

    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: succeed");

    return cnt->track.move_wait;
}

static unsigned int iomojo_move(struct context *cnt, int dev, struct coord *cent,
                                      struct images *imgs)
{
    char command[5];
    int direction = 0;
    int nx = 0, ny = 0;
    int i;

    if (dev < 0)
        if (!iomojo_center(cnt, 0, 0))
            return 0;

    if (cent->x < imgs->width / 2) {
        direction |= IOMOJO_DIRECTION_LEFT;
        nx = imgs->width / 2 - cent->x;
    }

    if (cent->x > imgs->width / 2) {
        direction |= IOMOJO_DIRECTION_RIGHT;
        nx = cent->x - imgs->width / 2;
    }

    if (cent->y < imgs->height / 2) {
        direction |= IOMOJO_DIRECTION_DOWN;
        ny = imgs->height / 2 - cent->y;
    }

    if (cent->y > imgs->height / 2) {
        direction |= IOMOJO_DIRECTION_UP;
        ny = cent->y - imgs->height / 2;
    }

    nx = nx * 72 / imgs->width;
    ny = ny * 72 / imgs->height;

    if (nx || ny) {
        if (nx > 180)
            nx = 180;

        if (ny > 60)
            ny = 60;

        command[0] = IOMOJO_MOVEOFFSET_CMD;
        command[1] = cnt->track.iomojo_id;
        command[2] = direction;
        command[3] = nx;
        command[4] = ny;
        iomojo_command(cnt, command, 5, 0);

        /* Number of frames to skip while moving */
        if (ny >= nx)
            i = 25 * ny / 90;
        else
            i = 25 * nx / 90;
        return i;
    }

    return 0;
}

/******************************************************************************

    Logitech QuickCam Orbit camera tracking code by folkert@vanheusden.com

******************************************************************************/
#ifdef HAVE_V4L2
static unsigned int lqos_center(struct context *cnt, int dev, int x_angle, int y_angle)
{
    int reset = 3;
    struct pwc_mpt_angles pma;
    struct pwc_mpt_range pmr;

    if (cnt->track.dev == -1) {

        if (ioctl(dev, VIDIOCPWCMPTRESET, &reset) == -1) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Failed to reset pwc camera to starting position! Reason");
            return 0;
        }

        SLEEP(6, 0);

        if (ioctl(dev, VIDIOCPWCMPTGRANGE, &pmr) == -1) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: failed VIDIOCPWCMPTGRANGE");
            return 0;
        }

        cnt->track.dev = dev;
        cnt->track.minmaxfound = 1;
        cnt->track.minx = pmr.pan_min;
        cnt->track.maxx = pmr.pan_max;
        cnt->track.miny = pmr.tilt_min;
        cnt->track.maxy = pmr.tilt_max;
    }

    if (ioctl(dev, VIDIOCPWCMPTGANGLE, &pma) == -1)
        MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: ioctl VIDIOCPWCMPTGANGLE");

    pma.absolute = 1;

    if (x_angle * 100 < cnt->track.maxx && x_angle * 100 > cnt->track.minx)
        pma.pan = x_angle * 100;

    if (y_angle * 100 < cnt->track.maxy && y_angle * 100 > cnt->track.miny)
        pma.tilt = y_angle * 100;

    if (ioctl(dev, VIDIOCPWCMPTSANGLE, &pma) == -1) {
        MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Failed to pan/tilt pwc camera! Reason");
        return 0;
    }

    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: succeed");

    return cnt->track.move_wait;
}

static unsigned int lqos_move(struct context *cnt, int dev, struct coord *cent,
                                    struct images *imgs, unsigned int manual)
{
    int delta_x = cent->x - (imgs->width / 2);
    int delta_y = cent->y - (imgs->height / 2);
    int move_x_degrees, move_y_degrees;
    struct pwc_mpt_angles pma;
    struct pwc_mpt_range pmr;

    /* If we are on auto track we calculate delta, otherwise we use user input in degrees times 100 */
    if (!manual) {
        if (delta_x > imgs->width * 3/8 && delta_x < imgs->width * 5/8)
            return 0;
        if (delta_y > imgs->height * 3/8 && delta_y < imgs->height * 5/8)
            return 0;

        move_x_degrees = delta_x * cnt->track.step_angle_x * 100 / (imgs->width / 2);
        move_y_degrees = -delta_y * cnt->track.step_angle_y * 100 / (imgs->height / 2);
    } else {
        move_x_degrees = cent->x * 100;
        move_y_degrees = cent->y * 100;
    }

    /* If we never checked for the min/max values for pan/tilt we do it now */
    if (cnt->track.minmaxfound == 0) {
        if (ioctl(dev, VIDIOCPWCMPTGRANGE, &pmr) == -1) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: failed VIDIOCPWCMPTGRANGE");
            return 0;
        }
        cnt->track.minmaxfound = 1;
        cnt->track.minx = pmr.pan_min;
        cnt->track.maxx = pmr.pan_max;
        cnt->track.miny = pmr.tilt_min;
        cnt->track.maxy = pmr.tilt_max;
    }

    /* Get current camera position */
    if (ioctl(dev, VIDIOCPWCMPTGANGLE, &pma) == -1)
        MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: ioctl VIDIOCPWCMPTGANGLE");


    /*
     * Check current position of camera and see if we need to adjust
     * values down to what is left to move
     */
    if (move_x_degrees < 0 && (cnt->track.minx - pma.pan) > move_x_degrees)
        move_x_degrees = (cnt->track.minx - pma.pan);

    if (move_x_degrees > 0 && (cnt->track.maxx - pma.pan) < move_x_degrees)
        move_x_degrees = (cnt->track.maxx - pma.pan);

    if (move_y_degrees < 0 && (cnt->track.miny - pma.tilt) > move_y_degrees)
        move_y_degrees = (cnt->track.miny - pma.tilt);

    if (move_y_degrees > 0 && (cnt->track.maxy - pma.tilt) < move_y_degrees)
        move_y_degrees = (cnt->track.maxy - pma.tilt);

    /* Move camera relative to current position */
    pma.absolute = 0;
    pma.pan = move_x_degrees;
    pma.tilt = move_y_degrees;

    if (ioctl(dev, VIDIOCPWCMPTSANGLE, &pma) == -1) {
        MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Failed to pan/tilt pwc camera! Reason");
        return 0;
    }

    return cnt->track.move_wait;
}

/******************************************************************************

    Logitech QuickCam Sphere camera tracking code by oBi

    Modify by Dirk Wesenberg(Munich) 30.03.07
    - for new API in uvcvideo
    - add Trace-steps for investigation
******************************************************************************/
static unsigned int uvc_center(struct context *cnt, int dev, int x_angle, int y_angle)
{
    /* CALC ABSOLUTE MOVING : Act.Position +/- delta to request X and Y */
    int move_x_degrees = 0, move_y_degrees = 0;

    union pantilt {
        struct {
            short pan;
            short tilt;
        } s16;
        int value;
    };
    union pantilt pan;

    if (cnt->track.dev == -1) {

        int reset = 3; //0-non reset, 1-reset pan, 2-reset tilt, 3-reset pan&tilt
        struct v4l2_control control_s;

        control_s.id = V4L2_CID_PAN_RESET;
        control_s.value = (unsigned char) reset;

        if (ioctl(dev, VIDIOC_S_CTRL, &control_s) < 0) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Failed to reset UVC camera to starting position! Reason");
            return 0;
        }

        control_s.id = V4L2_CID_TILT_RESET;
        control_s.value = (unsigned char) reset;

        if (ioctl(dev, VIDIOC_S_CTRL, &control_s) < 0) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Failed to reset UVC camera to starting position! Reason");
            return 0;
        }

        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Reseting UVC camera to starting position");

        SLEEP(8, 0);

        /* Get camera range */
        struct v4l2_queryctrl queryctrl;
        queryctrl.id = V4L2_CID_PAN_RELATIVE;

        if (ioctl(dev, VIDIOC_QUERYCTRL, &queryctrl) < 0) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: ioctl querycontrol");
            return 0;
        }

        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Getting camera range");

       /* DWe 30.03.07 The orig request failed :
        * must be VIDIOC_G_CTRL separate for pan and tilt or via VIDIOC_G_EXT_CTRLS - now for 1st manual
        * Range X = -70 to +70 degrees
        * Y = -30 to +30 degrees
        */

        //get mininum
        //pan.value = queryctrl.minimum;

        cnt->track.minx = -4480 / INCPANTILT;
        cnt->track.miny = -1920 / INCPANTILT;
        //get maximum
        cnt->track.maxx = 4480 / INCPANTILT;
        cnt->track.maxy = 1920 / INCPANTILT;
        //pan.value = queryctrl.maximum;

        cnt->track.dev = dev;
        cnt->track.pan_angle = 0;
        cnt->track.tilt_angle = 0;
        cnt->track.minmaxfound = 1;

    }

    struct v4l2_control control_s;

    MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, "%s: INPUT_PARAM_ABS pan_min %d,pan_max %d,tilt_min %d,tilt_max %d ",
               cnt->track.minx, cnt->track.maxx, cnt->track.miny, cnt->track.maxy);
    MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, "%s: INPUT_PARAM_ABS X_Angel %d, Y_Angel %d ",
               x_angle, y_angle);

    if (x_angle <= cnt->track.maxx && x_angle >= cnt->track.minx)
        move_x_degrees = x_angle - (cnt->track.pan_angle);

    if (y_angle <= cnt->track.maxy && y_angle >= cnt->track.miny)
        move_y_degrees = y_angle - (cnt->track.tilt_angle);


    /*
     * tilt up: - value
     * tilt down: + value
     * pan left: - value
     * pan right: + value
     */
    pan.s16.pan = -move_x_degrees * INCPANTILT;
    pan.s16.tilt = -move_y_degrees * INCPANTILT;

    MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, "%s: For_SET_ABS move_X %d,move_Y %d",
               move_x_degrees, move_y_degrees);

    /* DWe 30.03.07 Must be broken in diff calls, because
     * one call for both is not accept via VIDIOC_S_CTRL -> maybe via VIDIOC_S_EXT_CTRLS
     * The Webcam or uvcvideo does not like a call with a zero-move
     */

    if (move_x_degrees != 0) {
        control_s.id = V4L2_CID_PAN_RELATIVE;
        //control_s.value = pan.value;
        control_s.value = pan.s16.pan;

        if (ioctl(dev, VIDIOC_S_CTRL, &control_s) < 0) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Failed to move UVC camera!");
            return 0;
        }
    }

    /* DWe 30.03.07 We must wait a little,before we set the next CMD, otherwise PAN is mad ... */
    if ((move_x_degrees != 0) && (move_y_degrees != 0))
        SLEEP(1, 0);

    if (move_y_degrees != 0) {
        control_s.id = V4L2_CID_TILT_RELATIVE;
        //control_s.value = pan.value;
        control_s.value = pan.s16.tilt;

        if (ioctl(dev, VIDIOC_S_CTRL, &control_s) < 0) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Failed to move UVC camera!");
            return 0;
        }
    }

    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Found MINMAX = %d",
               cnt->track.minmaxfound);

    if (cnt->track.dev != -1) {
        MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, "%s: Before_ABS_Y_Angel : x= %d , Y= %d, ",
                   cnt->track.pan_angle, cnt->track.tilt_angle);

        if (move_x_degrees != -1) {
            cnt->track.pan_angle += move_x_degrees;
        }

        if (move_x_degrees != -1) {
            cnt->track.tilt_angle += move_y_degrees;
        }

        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: After_ABS_Y_Angel : x= %d , Y= %d",
                   cnt->track.pan_angle, cnt->track.tilt_angle);
    }

    return cnt->track.move_wait;
}

static unsigned int uvc_move(struct context *cnt, int dev, struct coord *cent,
                                   struct images *imgs, unsigned int manual)
{
    /* RELATIVE MOVING : Act.Position +/- X and Y */

    int delta_x = cent->x - (imgs->width / 2);
    int delta_y = cent->y - (imgs->height / 2);
    int move_x_degrees, move_y_degrees;

    /*
     *  DWe 30.03.07 Does the request of act.position from WebCam work ? luvcview shows at every position 180 :(
     *        Now we init the Web by call Reset, so we can sure, that we are at x/y = 0,0
     *        Don't worry, if the WebCam make a sound - over End at PAN  - hmmm, should it be normal ...?
     *        PAN Value 7777 in relative will init also a want reset for CAM - it will be "0" after that
     */
    if ((cnt->track.minmaxfound != 1) || (cent->x == 7777)) {
        unsigned int reset = 3; //0-non reset, 1-reset pan, 2-reset tilt, 3-reset pan&tilt
        struct v4l2_control control_s;

        control_s.id = V4L2_CID_PAN_RESET;
        control_s.value = (unsigned char) reset;

        if (ioctl(dev, VIDIOC_S_CTRL, &control_s) < 0) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Failed to reset UVC camera to starting position! Reason");
            return 0;
        }

        control_s.id = V4L2_CID_TILT_RESET;
        control_s.value = (unsigned char) reset;

        if (ioctl(dev, VIDIOC_S_CTRL, &control_s) < 0) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Failed to reset UVC camera to starting position! Reason");
            return 0;
        }

        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Reseting UVC camera to starting position");

        /* set the "helpvalue" back to null because after reset CAM should be in x=0 and not 70 */
        cent->x = 0;
        SLEEP(8, 0);

        /*
         * DWe 30.03.07 The orig request failed :
         * must be VIDIOC_G_CTRL separate for pan and tilt or via VIDIOC_G_EXT_CTRLS - now for 1st manual
         * Range X = -70 to +70 degrees
         *       Y = -30 to +30 degrees
         */

        cnt->track.minx = -4480 / INCPANTILT;
        cnt->track.miny = -1920 / INCPANTILT;
        cnt->track.maxx = 4480 / INCPANTILT;
        cnt->track.maxy = 1920 / INCPANTILT;
        cnt->track.dev = dev;
        cnt->track.pan_angle = 0;
        cnt->track.tilt_angle = 0;
        cnt->track.minmaxfound = 1;
    }


    /* If we are on auto track we calculate delta, otherwise we use user input in degrees */
    if (!manual) {
        if (delta_x > imgs->width * 3/8 && delta_x < imgs->width * 5/8)
            return 0;
        if (delta_y > imgs->height * 3/8 && delta_y < imgs->height * 5/8)
            return 0;

        move_x_degrees = delta_x * cnt->track.step_angle_x / (imgs->width / 2);
        move_y_degrees = -delta_y * cnt->track.step_angle_y / (imgs->height / 2);
    } else {
        move_x_degrees = cent->x;
        move_y_degrees = cent->y;
    }

    union pantilt {
        struct {
            short pan;
            short tilt;
        } s16;
        int value;
    };

    struct v4l2_control control_s;
    union pantilt pan;

    if (cnt->track.minmaxfound == 1) {
    /*
     * Check current position of camera and see if we need to adjust
     * values down to what is left to move
     */
        if (move_x_degrees < 0 && (cnt->track.minx - cnt->track.pan_angle) > move_x_degrees)
            move_x_degrees = cnt->track.minx - cnt->track.pan_angle;

        if (move_x_degrees > 0 && (cnt->track.maxx - cnt->track.pan_angle) < move_x_degrees)
            move_x_degrees = cnt->track.maxx - cnt->track.pan_angle;

        if (move_y_degrees < 0 && (cnt->track.miny - cnt->track.tilt_angle) > move_y_degrees)
            move_y_degrees = cnt->track.miny - cnt->track.tilt_angle;

        if (move_y_degrees > 0 && (cnt->track.maxy - cnt->track.tilt_angle) < move_y_degrees)
            move_y_degrees = cnt->track.maxy - cnt->track.tilt_angle;
    }

    MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, "For_SET_REL pan_min %d,pan_max %d,tilt_min %d,tilt_max %d",
               cnt->track.minx, cnt->track.maxx, cnt->track.miny, cnt->track.maxy);
    MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, "For_SET_REL track_pan_Angel %d, track_tilt_Angel %d",
               cnt->track.pan_angle, cnt->track.tilt_angle);
    MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, "For_SET_REL move_X %d,move_Y %d", move_x_degrees, move_y_degrees);

    /*
     * tilt up: - value
     * tilt down: + value
     * pan left: - value
     * pan right: + value
     */

    pan.s16.pan = -move_x_degrees * INCPANTILT;
    pan.s16.tilt = -move_y_degrees * INCPANTILT;

    /* DWe 30.03.07 Must be broken in diff calls, because
     * one call for both is not accept via VIDIOC_S_CTRL -> maybe via VIDIOC_S_EXT_CTRLS
     * The Webcam or uvcvideo does not like a call with a zero-move
     */

    if (move_x_degrees != 0) {

        control_s.id = V4L2_CID_PAN_RELATIVE;

        control_s.value = pan.s16.pan;
        MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, " dev %d, addr= %d, control_S= %d, Wert= %d",
                   dev, VIDIOC_S_CTRL, &control_s, pan.s16.pan);

        if (ioctl(dev, VIDIOC_S_CTRL, &control_s) < 0) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Failed to move UVC camera!");
            return 0;
        }
    }

    /* DWe 30.03.07 We must wait a little,before we set the next CMD, otherwise PAN is mad ... */
    if ((move_x_degrees != 0) && (move_y_degrees != 0))
        SLEEP (1, 0);


    if (move_y_degrees != 0) {

        control_s.id = V4L2_CID_TILT_RELATIVE;

        control_s.value = pan.s16.tilt;
        MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, " dev %d,addr= %d, control_S= %d, Wert= %d",
                   dev, VIDIOC_S_CTRL, &control_s, pan.s16.tilt);

        if (ioctl(dev, VIDIOC_S_CTRL, &control_s) < 0) {
            MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Failed to move UVC camera!");
            return 0;
        }
    }

    MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, "%s: Found MINMAX = %d",
                cnt->track.minmaxfound);

    if (cnt->track.minmaxfound == 1) {
        MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, "Before_REL_Y_Angel : x= %d , Y= %d",
                   cnt->track.pan_angle, cnt->track.tilt_angle);

        if (move_x_degrees != 0)
            cnt->track.pan_angle += -pan.s16.pan / INCPANTILT;

        if (move_y_degrees != 0)
            cnt->track.tilt_angle += -pan.s16.tilt / INCPANTILT;

        MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, "After_REL_Y_Angel : x= %d , Y= %d",
                   cnt->track.pan_angle, cnt->track.tilt_angle);
    }

    return cnt->track.move_wait;
}
#endif /* HAVE_V4L2 */

/******************************************************************************
 *   Foscam FOSCAM_HD network camera tracking code by Inzebaba
 *   http://www.lavrsen.dk/twiki/bin/view/Motion/MotionTracking
 *   Experimental 2d approach with external wget - changed to use curl
******************************************************************************/
/**
 *      FOSCAM_HD_command
 *      Execute 'command' with 'arg' as its argument.
 *      if !arg command is started with no arguments
 *      Before we call execl we need to close all the file handles
 *      that the fork inherited from the parent in order not to pass
 *      the open handles on to the shell
 *
 *    	Replaced all this external stuff with curl commands!
 */
 
 
static int FOSCAM_HD_command(char *curl_cmd)
{
        // Just send a command to the camera
		
		CURLcode ret;
        CURL *hnd; 
		
		hnd = curl_easy_init();

		if(hnd) {
		curl_easy_setopt(hnd, CURLOPT_URL, curl_cmd);
		//curl_easy_setopt(hnd, CURLOPT_HEADER, FALSE);
		//curl_easy_setopt(hnd, CURLOPT_POSTREDIR, FALSE);
		//curl_easy_setopt(hnd, CURLOPT_CONNECT_ONLY, TRUE);
		curl_easy_setopt(hnd, CURLOPT_DNS_USE_GLOBAL_CACHE, FALSE);
		curl_easy_setopt(hnd, CURLOPT_FAILONERROR, FALSE);
		//curl_easy_setopt(hnd, CURLOPT_NOBODY, TRUE);
		//curl_easy_setopt(hnd, CURLOPT_TIMEOUT, 1);
		
        ret = curl_easy_perform(hnd);
			if(ret != 0) {
				MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: curl error %d. ABORTING!", ret);
				curl_easy_cleanup(hnd);	
				return(-1);
			}
    
		MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Command is running CURL %s", curl_cmd);
		curl_easy_cleanup(hnd);
		}
	return(0);
}

static void FOSCAM_HD_move_driver(struct context *cnt, char* direction,int nb_pixel)
{
		double diff;								// time check stuff
		struct timespec sleep, tv1, tv2, temp;		// time check stuff
        char command[PATH_MAX];
        const char *cptr;
		const long NSECS_PER_SEC = 1000000000;
		long CAM_LATENCY = 0;						// variable calculated as time it takes for cam to respond to HTTP commands
													// Depends on a lot of factors, like network response - so guess at 0.2s (in ns)
													
		CAM_LATENCY=((2*NSECS_PER_SEC)/10);			// 2 and 10 are 20% of 1 ns = 0.2s (keep below 1s code isn't set up for more)
		
        cnt->track.isCamMoving=TRUE;
        
		MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Camera %s moving in %s direction, stepsize is %d", cnt->netcam->connect_host, direction, nb_pixel);
				
        sprintf(command,"http://%s:%d/cgi-bin/CGIProxy.fcgi?cmd=%s",cnt->netcam->connect_host,cnt->netcam->connect_port,direction);
        if (cnt->conf.netcam_userpass != NULL) {
            if ((cptr = strchr(cnt->conf.netcam_userpass, ':')) != NULL) {
                sprintf(command,"http://%s:%d/cgi-bin/CGIProxy.fcgi?cmd=%s&usr=%.*s&pwd=%s",
                cnt->netcam->connect_host,cnt->netcam->connect_port,direction,(int)(cptr - cnt->conf.netcam_userpass),cnt->conf.netcam_userpass,cptr+1);
            }
        }
		
		sleep.tv_sec=(int)(nb_pixel/cnt->track.speed);
		sleep.tv_nsec=((nb_pixel%cnt->track.speed) * (NSECS_PER_SEC / cnt->track.speed));
		
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO,
            "%s: cam %s track_speed: %d px/seconds - calculated move delay: %d.%d seconds, Cam Latency 0.%d",cnt->netcam->connect_host, cnt->track.speed, sleep.tv_sec, (sleep.tv_nsec/10000000), (CAM_LATENCY/10000000));

		clock_gettime(CLOCK_MONOTONIC, &tv1);
		
		if ((sleep.tv_nsec-CAM_LATENCY) < 0) {
			sleep.tv_sec--;
			if (sleep.tv_sec < 0) {
				sleep.tv_sec=0;
				sleep.tv_nsec=0;
			}
			sleep.tv_nsec=NSECS_PER_SEC+sleep.tv_nsec-CAM_LATENCY;
		}
		else
			sleep.tv_nsec=sleep.tv_nsec-CAM_LATENCY;
			
		FOSCAM_HD_command(command);	 
					
        SLEEP(sleep.tv_sec, sleep.tv_nsec); //nanosleep function
	//nanosleep(&sleep, (struct timespec *)NULL);
		
        sprintf(command,"http://%s:%d/cgi-bin/CGIProxy.fcgi?cmd=%s",cnt->netcam->connect_host,cnt->netcam->connect_port,FOSCAM_HD_STOP);
        if (cnt->conf.netcam_userpass != NULL) {
            if ((cptr = strchr(cnt->conf.netcam_userpass, ':')) != NULL) {
                sprintf(command,"http://%s:%d/cgi-bin/CGIProxy.fcgi?cmd=%s&usr=%.*s&pwd=%s",
                cnt->netcam->connect_host,cnt->netcam->connect_port,FOSCAM_HD_STOP,(int)(cptr - cnt->conf.netcam_userpass),cnt->conf.netcam_userpass,cptr+1);
            }
        }
        FOSCAM_HD_command(command);
		
		clock_gettime(CLOCK_MONOTONIC, &tv2);
		if ((tv2.tv_nsec-tv1.tv_nsec)<0) {
			temp.tv_sec = tv2.tv_sec-tv1.tv_sec-1;
			temp.tv_nsec = NSECS_PER_SEC+tv2.tv_nsec-tv1.tv_nsec;
		} else {
			temp.tv_sec = tv2.tv_sec-tv1.tv_sec;
			temp.tv_nsec = tv2.tv_nsec-tv1.tv_nsec;
		}
		diff = (double)(temp.tv_sec + ((double)temp.tv_nsec/NSECS_PER_SEC));	//How long did we really sleep?

		MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO,
            "%s: cam %s Sleep time: set %d.%d seconds, Cam Latency 0.%d - actual move delay: %.6f seconds", cnt->netcam->connect_host, sleep.tv_sec, (sleep.tv_nsec/10000000), (CAM_LATENCY/10000000), (float)diff);
        
        cnt->track.isCamMoving=FALSE;
}

// Goto to predefined position number 0 (or named in config) rather than center which is unusefull.
static unsigned int FOSCAM_HD_center(struct context *cnt, int x_offset, int y_offset)
{
    //this is an absolute move
    char command [PATH_MAX];
    const char *cptr;
	
	//if x_offset = y_offset = 0, then we are supposed to center the PTZ, instead, move to defined home position (if defined)
    if(x_offset == 0 && y_offset == 0 && strlen(cnt->track.track_home_pos_name) != 0) {
        cnt->moved=cnt->lastrate * 8;	//stop motion detection while we move home (8 seconds)
        cnt->track.active = 3;  //set mode to "centering" (for text display)
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Resetting FOSCAM_HD camera %s to home position position: %s", cnt->netcam->connect_host, cnt->track.track_home_pos_name );
        cnt->track.isCamMoving=TRUE;
        sprintf(command,"http://%s:%d/cgi-bin/CGIProxy.fcgi?cmd=%s%s",cnt->netcam->connect_host,cnt->netcam->connect_port,FOSCAM_HD_MOVEHOME,cnt->track.track_home_pos_name);
        if (cnt->conf.netcam_userpass != NULL) {
            if ((cptr = strchr(cnt->conf.netcam_userpass, ':')) != NULL) {
                sprintf(command,"http://%s:%d/cgi-bin/CGIProxy.fcgi?cmd=%s%s&usr=%.*s&pwd=%s",
                cnt->netcam->connect_host,cnt->netcam->connect_port,FOSCAM_HD_MOVEHOME,cnt->track.track_home_pos_name,(int)(cptr-cnt->conf.netcam_userpass),cnt->conf.netcam_userpass,cptr+1);
            }
        }
        FOSCAM_HD_command(command);

        cnt->track.abs_x=0;
        cnt->track.abs_y=0;
        cnt->track.direction=4;
        
        cnt->track.isCamMoving=FALSE;
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO,
           "%s: Camera %s Moved to home position: %s, abs position x=%d y=%d", cnt->netcam->connect_host, cnt->track.track_home_pos_name, cnt->track.abs_x, cnt->track.abs_y);
        if(cnt->track.active > 0 && cnt->track.thread_id == 0 )  //if tracking not disabled, and tracking thread not started (otherwise tracking thread will reset this)
            cnt->track.active = 1;  //set mode to "active" (for text display)
        return (cnt->lastrate * cnt->track.speed)/20; //means ~8 seconds times frame rate which should be long enough to move home.
    }
    else {  //manual entry?
        if(x_offset == 10000 || y_offset == 10000) {    //very bogus number to indicate calibrating
            cnt->track.active = 4;  //set mode to "calibrating" (for text display), and to trigger calibration routine in track_thread
            return cnt->track.move_wait;
        }
        if (x_offset == 1000) x_offset = cnt->netcam->width/2;  //1000 is just a bogus value used by the manual entry system to indicate no value
        if (y_offset == 1000) y_offset = cnt->netcam->height/2;
        struct coord cent;
        cent.x = x_offset;
        cent.y = y_offset;
        cent.width = cnt->netcam->width;
        cent.height = cnt->netcam->height;
        struct images *imgs = &cnt->imgs;
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO,
           "%s: Received Command to move Camera %s to abs position x=%d y=%d", cnt->netcam->connect_host, x_offset, y_offset);
           
        //MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO,
        //   "%s: Camera %s values: netcam width:%d height:%d, imgs width:%d height:%d", cnt->netcam->connect_host, cnt->netcam->width, cnt->netcam->height, imgs->width, imgs->height);
        return FOSCAM_HD_move(cnt, &cent, imgs);
    }
}

void *FOSCAM_HD_move_thread ( void *ptr )
{
    struct context *cnt = ptr;	//get pointer to context passed
    char* mv_direction[9]={FOSCAM_HD_LEFT_UP,FOSCAM_HD_UP,FOSCAM_HD_RIGHT_UP,
        FOSCAM_HD_LEFT,FOSCAM_HD_STOP,FOSCAM_HD_RIGHT,FOSCAM_HD_LEFT_DOWN,
        FOSCAM_HD_DOWN,FOSCAM_HD_RIGHT_DOWN};
    char direction_st[PATH_MAX];
    //char old_direction[PATH_MAX];
    int direction=0;
    int old_direction=0;
    int step_size=0;
    int new_step_size=0;
    int old_step_size=0;
    int curr_x=0, curr_y=0; //current abs x and y pos
    int pos_delta=0;    //change in abs_x and y for diagonal moves
    int check_limits=0;
    int recal_interval=0;    //Number of motions before a recal is needed - guess at 100 start/stops (recal_count)
    int recal_count = cnt->track.motorx; //100; //was 100, hijack motorx for recal count (number of moves before recalibration)
    time_t last_motion_t, now_t;  //timing of "home" event
    int home_timeout = cnt->track.iomojo_id;    //hijack iomojo_id for home time period in seconds
    if(recal_count < 100) recal_count = 100;    //minimmum number of moves is 100
    

    time(&last_motion_t); //capture start time

    /* Store the corresponding motion thread number in TLS also for this
    * thread (necessary for 'MOTION_LOG' to function properly).
    */
    pthread_setspecific(tls_key_threadnr, (void *)((unsigned long)cnt->threadnr));

	if ((cnt->track.maxx!=0) || (cnt->track.maxy!=0) || (cnt->track.minx!=0) || (cnt->track.miny!=0)) {	// if any of these values have been set in .conf
		check_limits=1;
		MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Camera %s Limits check enabled - Left %d, Right %d, Down %d, Up %d",
			cnt->netcam->connect_host, cnt->track.minx, cnt->track.maxx, cnt->track.miny, cnt->track.maxy);
	}
    
    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s PTZ Started, home timeout %ds", cnt->netcam->connect_host, home_timeout/1000);
        
    /*
    * The logic of our loop is very simple.  We move compare old direction and step size with new,
    * if it changes we set isCamMoving then move the camera and wait for it to finish. Unset isCamMoving.
    * The loop continues until cnt->finish is set.
    */	
    while (!cnt->finish) {
        //pthread_mutex_lock(&global_lock);
        direction=cnt->track.direction;
        //strcpy(direction_st, mv_direction[cnt->track.direction]);
        step_size=cnt->track.step_size;
        //pthread_mutex_unlock(&global_lock);

        if ((cnt->track.direction != old_direction) || (step_size != old_step_size)) {
            MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, "%s: cam %s PTZ Running, got direction: %d, step: %d", cnt->netcam->connect_host, direction, step_size);
            //MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, "%s: Thread %d running (cam %s), direction: %d, step: %d old direction: %s, old step: %d, old abs pos is x=%d, y=%d",
            //    cnt->track.thread_id, cnt->netcam->connect_host, direction, step_size, old_direction, old_step_size, cnt->track.abs_x, cnt->track.abs_y ); //causes crash...
            //cnt->track.isCamMoving=TRUE;
            if(cnt->track.active == 1)  //if we are in normal tracking mode
                cnt->track.active = 2;  //set mode to "tracking" (for text display)
            strcpy(direction_st, mv_direction[direction]);
            //step_size=cnt->track.step_size;
            old_direction=direction;
            old_step_size=step_size;
            curr_x = cnt->track.abs_x;  //save old values
            curr_y = cnt->track.abs_y;
            
            if (step_size <= 0) 
                continue;   //skip everything as no motion!
            
            pos_delta = (int)sqrt((step_size*step_size)/2); //x and y change for diagonal moves
            
            time(&last_motion_t); //capture start time of motion
            
            //work out new absolute position
            //remember if image is 640X480, then limits on step_size are (left/right) +/-320(x) (up/down) +/-240(y) and so on for different matrix sizes
            
            switch (old_direction) {
                case 0:
                    cnt->track.abs_y=cnt->track.abs_y+pos_delta;    //up so add
                    cnt->track.abs_x=cnt->track.abs_x-pos_delta;    //left so subtract
                    if (check_limits) {
                        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Checking cam %s Limits", cnt->netcam->connect_host);
                        if (cnt->track.abs_x < cnt->track.minx) {
                            int delta = abs(cnt->track.abs_x-cnt->track.minx);    //reduction in x/y step size
                            cnt->track.abs_y -= delta;
                            cnt->track.abs_x += delta;
                            //cnt->track.abs_x=cnt->track.minx;
                            pos_delta -= delta;
                            step_size=(int)sqrt((pos_delta*pos_delta)*2);
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s LEFT UP motion beyond x limit %d, step reduced by %d", cnt->netcam->connect_host, cnt->track.minx, old_step_size - step_size);
                            if(step_size <= 0) {    //we can't move left any more, but can still move up
                                cnt->track.abs_y += delta;  //up so add
                                cnt->track.direction = 1;   //up
                                new_step_size = delta;
                                //step_size = delta;
                                MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Can't move left anymore, so moving UP to new motion target %d/%d, step size %d", cnt->netcam->connect_host, cnt->track.abs_x,cnt->track.abs_y, new_step_size);
                                break;
                            }
                        }
                    if (cnt->track.abs_y > cnt->track.maxy) {
                            int delta = abs(cnt->track.abs_y-cnt->track.maxy);    //reduction in x/y step size
                            cnt->track.abs_x += delta;
                            cnt->track.abs_y -= delta;
                            //cnt->track.abs_y=cnt->track.maxy;
                            pos_delta -= delta;
                            step_size=(int)sqrt((pos_delta*pos_delta)*2);
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: %s cam LEFT UP motion beyond y limit %d, step reduced by %d", cnt->netcam->connect_host, cnt->track.maxy, old_step_size - step_size);
                            if(step_size <= 0) {    //we can't move up any more, but can still move left
                                cnt->track.abs_x -= delta;  //left so subtract
                                cnt->track.direction = 3;   //left
                                new_step_size = delta;
                                //step_size = delta;
                                MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Can't move up anymore, so moving LEFT to new motion target %d/%d, step size %d", cnt->netcam->connect_host, cnt->track.abs_x,cnt->track.abs_y, new_step_size);
                                break;
                            }
                        }
                    }
                    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s LEFT_UP %d, new abs pos x=%d, y=%d", cnt->netcam->connect_host, step_size, cnt->track.abs_x, cnt->track.abs_y);
                    break;
                case 1:
                    cnt->track.abs_y=cnt->track.abs_y+step_size;
                    if (check_limits) {
                        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Checking cam %s Limits", cnt->netcam->connect_host);
                        if (cnt->track.abs_y > cnt->track.maxy) {
                            step_size=abs(step_size-abs(cnt->track.abs_y-cnt->track.maxy));
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s UP motion beyond limit %d, step reduced by %d to %d", cnt->netcam->connect_host, cnt->track.maxy, old_step_size - step_size, step_size);
                            cnt->track.abs_y=cnt->track.maxy;
                        }
                    }
                    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s UP %d, new abs pos x=%d, y=%d", cnt->netcam->connect_host, step_size, cnt->track.abs_x, cnt->track.abs_y);
                    break;
                case 2:
                    cnt->track.abs_y=cnt->track.abs_y+pos_delta;    //up so add
                    cnt->track.abs_x=cnt->track.abs_x+pos_delta;    //right so add
                    if (check_limits) {
                        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Checking cam %s Limits", cnt->netcam->connect_host);
                        if (cnt->track.abs_x > cnt->track.maxx) {
                            int delta = abs(cnt->track.abs_x-cnt->track.maxx);    //reduction in x/y step size
                            cnt->track.abs_y -= delta;
                            cnt->track.abs_x -= delta;
                            //cnt->track.abs_x=cnt->track.maxx;
                            pos_delta -= delta;
                            step_size=(int)sqrt((pos_delta*pos_delta)*2);
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s RIGHT UP motion beyond x limit %d, step reduced by %d", cnt->netcam->connect_host, cnt->track.maxx, old_step_size - step_size);
                            if(step_size <= 0) {    //we can't move right any more, but can still move up
                                cnt->track.direction = 1;   //up
                                new_step_size = delta;
                                //step_size = delta;
                                MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Can't move right anymore, so moving UP to new motion target %d/%d, step size %d", cnt->netcam->connect_host, cnt->track.abs_x,cnt->track.abs_y, new_step_size);
                                break;
                            }
                        }
                    if (cnt->track.abs_y > cnt->track.maxy) {
                            int delta = abs(cnt->track.abs_y-cnt->track.maxy);    //reduction in x/y step size
                            cnt->track.abs_x -= delta;
                            cnt->track.abs_y -= delta;
                            //cnt->track.abs_y=cnt->track.maxy;
                            pos_delta -= delta;
                            step_size=(int)sqrt((pos_delta*pos_delta)*2);
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s RIGHT UP motion beyond y limit %d, step reduced by %d", cnt->netcam->connect_host, cnt->track.maxy, old_step_size - step_size);
                            if(step_size <= 0) {    //we can't move up any more, but can still move right
                                cnt->track.abs_x += delta;  //right so add
                                cnt->track.direction = 5;   //right
                                new_step_size = delta;
                                //step_size = delta;
                                MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Can't move up anymore, so moving RIGHT to new motion target %d/%d, step size %d", cnt->netcam->connect_host, cnt->track.abs_x,cnt->track.abs_y, new_step_size);
                                break;
                            }
                        }
                    }
                    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s RIGHT_UP %d, new abs pos x=%d, y=%d", cnt->netcam->connect_host, step_size, cnt->track.abs_x, cnt->track.abs_y);
                    break;
                case 3:
                    cnt->track.abs_x=cnt->track.abs_x-step_size;
                    if (check_limits) {
                        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Checking cam %s Limits", cnt->netcam->connect_host);
                        if (cnt->track.abs_x < cnt->track.minx) {
                            step_size=abs(step_size-abs(cnt->track.abs_x-cnt->track.minx));
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s LEFT motion beyond limit %d, step reduced by %d to %d", cnt->netcam->connect_host, cnt->track.minx, old_step_size - step_size, step_size);
                            cnt->track.abs_x=cnt->track.minx;
                        }
                    }
                    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s LEFT %d, new abs pos x=%d, y=%d", cnt->netcam->connect_host, step_size, cnt->track.abs_x, cnt->track.abs_y);
                    break;
                case 4:
                    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Cam %s is CENTERED", cnt->netcam->connect_host);
                    step_size=0;
                    break;
                case 5:
                    cnt->track.abs_x=cnt->track.abs_x+step_size;
                    if (check_limits) {
                        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Checking cam %s Limits", cnt->netcam->connect_host);
                        if (cnt->track.abs_x > cnt->track.maxx) {
                            step_size=abs(step_size-abs(cnt->track.abs_x-cnt->track.maxx));
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s RIGHT motion (to %d) is beyond limit %d, step reduced by %d to %d", cnt->netcam->connect_host, cnt->track.abs_x, cnt->track.maxx, old_step_size - step_size, step_size);
                            cnt->track.abs_x=cnt->track.maxx;
                        }
                    }
                    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s RIGHT %d, new abs pos x=%d, y=%d", cnt->netcam->connect_host, step_size, cnt->track.abs_x, cnt->track.abs_y);
                    break;
                case 6:
                    cnt->track.abs_y=cnt->track.abs_y-pos_delta;    //down so subtract
                    cnt->track.abs_x=cnt->track.abs_x-pos_delta;    //left so subtract
                    if (check_limits) {
                        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Checking Limits");
                        if (cnt->track.abs_x < cnt->track.minx) {
                            int delta = abs(cnt->track.abs_x-cnt->track.minx);    //reduction in x/y step size
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s LEFT DOWN x motion target %d, %d beyond minx", cnt->netcam->connect_host, cnt->track.abs_x, delta);
                            cnt->track.abs_y += delta;
                            cnt->track.abs_x += delta;
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s LEFT DOWN new x/y motion target %d/%d", cnt->netcam->connect_host, cnt->track.abs_x,cnt->track.abs_y, delta);
                            //cnt->track.abs_x=cnt->track.minx;
                            pos_delta -= delta;
                            step_size=(int)sqrt((pos_delta*pos_delta)*2);
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s LEFT DOWN motion beyond x limit %d, step reduced by %d tp %d", cnt->netcam->connect_host, cnt->track.minx, old_step_size - step_size, new_step_size);
                            if(step_size <= 0) {    //we can't move left any more, but can still move down
                                cnt->track.abs_y -= delta;  //down so subtract
                                cnt->track.direction = 7;   //down
                                new_step_size = delta;
                                //step_size = delta;
                                MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Can't move left anymore, so moving DOWN to new motion target %d/%d, step size %d", cnt->netcam->connect_host, cnt->track.abs_x,cnt->track.abs_y, new_step_size);
                                break;
                            }
                        }
                    if (cnt->track.abs_y < cnt->track.miny) {
                            int delta = abs(cnt->track.abs_y-cnt->track.miny);    //reduction in x/y step size
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s LEFT DOWN y motion target %d, %d beyond miny", cnt->netcam->connect_host, cnt->track.abs_y, delta);
                            cnt->track.abs_x += delta;
                            cnt->track.abs_y += delta;
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s LEFT DOWN new x/y motion target %d/%d", cnt->netcam->connect_host, cnt->track.abs_x,cnt->track.abs_y, delta);
                            //cnt->track.abs_y=cnt->track.miny;
                            pos_delta -= delta;
                            step_size=(int)sqrt((pos_delta*pos_delta)*2);
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s LEFT DOWN motion beyond y limit %d, step reduced by %d to %d", cnt->netcam->connect_host, cnt->track.miny, old_step_size - step_size, step_size);
                            if(step_size <= 0) {    //we can't move down any more, but can still move left
                                cnt->track.abs_x -= delta;  //left so subtract
                                cnt->track.direction = 3;   //left
                                new_step_size = delta;
                                //step_size = delta;
                                MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Can't move down anymore, so moving LEFT to new motion target %d/%d, step size %d", cnt->netcam->connect_host, cnt->track.abs_x,cnt->track.abs_y, new_step_size);
                                break;
                            }
                        }
                    }
                    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s LEFT_DOWN %d, new abs pos x=%d, y=%d", cnt->netcam->connect_host, step_size, cnt->track.abs_x, cnt->track.abs_y);
                    break;
                case 7:
                    cnt->track.abs_y=cnt->track.abs_y-step_size;
                    if (check_limits) {
                        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Checking cam %s Limits", cnt->netcam->connect_host);
                        if (cnt->track.abs_y < cnt->track.miny) {
                            step_size=abs(step_size-abs(cnt->track.abs_y-cnt->track.miny));
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s DOWN motion beyond limit %d, step reduced by %d to %d", cnt->netcam->connect_host, cnt->track.miny, old_step_size - step_size, step_size);
                            cnt->track.abs_y=cnt->track.miny;
                        }
                    }
                    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s DOWN %d, new abs pos x=%d, y=%d", cnt->netcam->connect_host, step_size, cnt->track.abs_x, cnt->track.abs_y);
                    break;
                case 8:
                    cnt->track.abs_y=cnt->track.abs_y-pos_delta;    //down so subtract
                    cnt->track.abs_x=cnt->track.abs_x+pos_delta;    //right so add
                    if (check_limits) {
                        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Checking cam %s Limits", cnt->netcam->connect_host);
                        if (cnt->track.abs_x > cnt->track.maxx) {
                            int delta = abs(cnt->track.abs_x-cnt->track.maxx);    //reduction in x/y step size
                            //MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s RIGHT DOWN x motion target %d, %d beyond maxx", cnt->netcam->connect_host, cnt->track.abs_x, delta);
                            cnt->track.abs_y += delta;
                            cnt->track.abs_x -= delta;
                            //MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s RIGHT DOWN new x/y motion target %d/%d", cnt->netcam->connect_host, cnt->track.abs_x,cnt->track.abs_y, delta);
                            //cnt->track.abs_x=cnt->track.maxx;
                            pos_delta -= delta;
                            step_size=(int)sqrt((pos_delta*pos_delta)*2);
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s RIGHT DOWN motion beyond x limit %d, step reduced by %d to %d", cnt->netcam->connect_host, cnt->track.maxx, old_step_size - step_size, step_size);
                            if(step_size <= 0) {    //we can't move right any more, but can still move down
                                cnt->track.abs_y -= delta;  //down so subtract
                                cnt->track.direction = 7;   //down
                                new_step_size = delta;
                                //step_size = delta;
                                MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Can't move right anymore, so moving DOWN to new motion target %d/%d, step size %d", cnt->netcam->connect_host, cnt->track.abs_x,cnt->track.abs_y, new_step_size);
                                break;
                            }
                        }
                        if (cnt->track.abs_y < cnt->track.miny) {
                            int delta = abs(cnt->track.abs_y-cnt->track.miny);    //reduction in x/y step size
                            //MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s RIGHT DOWN y motion target %d, %d beyond miny", cnt->netcam->connect_host, cnt->track.abs_y, delta);
                            cnt->track.abs_x -= delta;
                            cnt->track.abs_y += delta;
                            //MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s RIGHT DOWN new x/y motion target %d/%d", cnt->netcam->connect_host, cnt->track.abs_x,cnt->track.abs_y, delta);
                            //cnt->track.abs_y=cnt->track.miny;
                            pos_delta -= delta;
                            step_size=(int)sqrt((pos_delta*pos_delta)*2);
                            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s RIGHT DOWN motion beyond y limit %d, step reduced by %d to %d", cnt->netcam->connect_host, cnt->track.miny, old_step_size - step_size, step_size);
                            if(step_size <= 0) {    //we can't move down any more, but can still move right
                                cnt->track.abs_x += delta;  //right so add
                                cnt->track.direction = 5;   //right
                                new_step_size = delta;
                                step_size = delta;
                                MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Can't move down anymore, so moving RIGHT to new motion target %d/%d, step size %d", cnt->netcam->connect_host, cnt->track.abs_x,cnt->track.abs_y, new_step_size);
                                break;
                            }
                        }
                    }
                    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s RIGHT_DOWN %d, new abs pos x=%d, y=%d", cnt->netcam->connect_host, step_size, cnt->track.abs_x, cnt->track.abs_y);
                    break;
                default:
                    MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: cam %s WRONG AREA Detection. Not Moving!", cnt->netcam->connect_host);
                    step_size=0;
                    //break;
            }   //of switch
            
            if (step_size > cnt->track.speed/4) {	// about 0.25 seconds of motion is the best control we have)
                recal_interval++;
                FOSCAM_HD_move_driver(cnt, direction_st, step_size); //send motion command
                MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s PTZ Motion complete (calibrate in: %d): New abs position is x=%d y=%d", cnt->netcam->connect_host, recal_count-recal_interval, cnt->track.abs_x, cnt->track.abs_y );
            }
            else {  //we didn't move so restore old values
                if((cnt->track.abs_x != curr_x) || (cnt->track.abs_y != curr_y)) {
                    cnt->track.abs_x = curr_x;
                    cnt->track.abs_y = curr_y;
                    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s PTZ Motion NOT complete (calibrate in: %d): motion requested (%d) too small. abs position is x=%d y=%d", cnt->netcam->connect_host, recal_count-recal_interval, step_size, cnt->track.abs_x, cnt->track.abs_y );
                }
            }
            
            if(new_step_size != 0) {    //if we have switched direction, set new step_size for next loop
                cnt->track.step_size = new_step_size;
                new_step_size = 0;
            }
        }   //of if
                
        //SLEEP (0, 300000000);
        if ( recal_interval > recal_count || cnt->track.active == 4) {	//recalibrate PTZ - move to limits and the centre
            MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s PTZ Motion calibration Starting - 90 secs to completion.", cnt->netcam->connect_host);
            //cnt->moved = 100000; //stop motion detection for a looong time
            cnt->pause = TRUE; //stop motion detection
            cnt->track.active = 4;  //set mode to "calibrating" (for text display)
            FOSCAM_HD_move_driver(cnt, "ptzMoveUp", 10*cnt->track.speed);
            FOSCAM_HD_move_driver(cnt, "ptzMoveDown", 15*cnt->track.speed);
            FOSCAM_HD_move_driver(cnt, "ptzMoveLeft", 15*cnt->track.speed);
            FOSCAM_HD_move_driver(cnt, "ptzMoveRight", 30*cnt->track.speed);
            //FOSCAM_HD_move_driver(cnt, "ptzMoveLeft", 20*cnt->track.speed);
            cnt->moved = FOSCAM_HD_center(cnt, 0, 0);  //will reset cnt->track.active
            //cnt->moved = 40*cnt->track.speed;
            cnt->pause = FALSE; //start motion detection (when moved expires)
            recal_interval=0;
            }
                
        //home camera if no motion for more than home_timeout seconds
        time(&now_t); //capture time now 
        if (((int)difftime(now_t, last_motion_t) >= home_timeout) && (home_timeout!=0)) {
            if(cnt->track.abs_x != 0 || cnt->track.abs_y != 0) {    //if we are not already at 0,0
                MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Timeout (%ds) - HOMING Camera,", cnt->netcam->connect_host, home_timeout);
                FOSCAM_HD_center(cnt, 0, 0);
            }
            time(&last_motion_t); //reset timer
        }

        //cnt->track.isCamMoving=FALSE;
        if(cnt->track.active > 0 && cnt->moved == 0)  //if tracking not disabled
            cnt->track.active = 1;  //set mode to "active" (for text display)

        SLEEP (0, 10000000);    //allow other threads to run 0.01s (10 ms) sleep
        //loop forever (or until cnt->finishes)
    }   //end of while

    /* Our thread is finished - decrement motion's thread count. */
    pthread_mutex_lock(&global_lock);
    threads_running--;
    pthread_mutex_unlock(&global_lock);
    cnt->track.thread_id=0;

    /* Log out a termination message. */
    MOTION_LOG(ALR, TYPE_NETCAM, NO_ERRNO, "%s: cam %s PTZ camera handler: finish set, exiting", cnt->netcam->connect_host);

    /* Goodbye..... */
    pthread_exit(NULL);
}


static unsigned int FOSCAM_HD_move(struct context *cnt, struct coord *cent, struct images *imgs)
{
    //this is a relative move
    int area_minx[9], area_miny[9], area_maxx[9], area_maxy[9];
    int i;
    int delta_x = abs(cent->x - (imgs->width / 2));
    int delta_y = abs(cent->y - (imgs->height / 2));
    int step_size;

	pthread_attr_t handler_attribute; /* Attributes of our handler thread. */
	
	if (cnt->track.isCamMoving) {
		MOTION_LOG(INF, TYPE_TRACK, NO_ERRNO, "%s: Camera %s is moving, waiting for it to stop", cnt->netcam->connect_host);
		return cnt->track.move_wait;
	}
	
	//MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Camera %s PTZ ready, move command received, motion location x=%d, y=%d", cnt->netcam->connect_host, cent->x, cent->y );

    /*
    Areas and algorithm explanation:
    Detect motion in predefined areas (1 - 9).
    Areas are numbered like that:  1 2 3
                                   4 5 6
                                   7 8 9
                                     
    Area 10 is out of range

    Matrix is set up as (for 640 x 480):
    area_minx:          0   240 400
                        0   240 400
                        0   240 400
    
    area_miny:          0   0   0 
                        120 120 120
                        360 360 360
                        
    area_maxx:          180 300 480
                        180 300 480
                        180 300 480
                        
    area_maxy:          120 120 120
                        360 360 360
                        480 480 480
                        
    if motion is detected at (420, 200), that would be Area: 6 as: 420>area_minx[6] & 420<area_maxx[6] & 200>area_miny[6] & 200<area_maxy[6]
    */
	
    /* Initialize area detection */
    area_minx[0] = area_minx[3] = area_minx[6] = 0;
    area_miny[0] = area_miny[1] = area_miny[2] = 0;

    area_minx[1] = area_minx[4] = area_minx[7] = imgs->width / 8 * 3;
    area_maxx[0] = area_maxx[3] = area_maxx[6] = imgs->width / 8 * 3;

    area_minx[2] = area_minx[5] = area_minx[8] = imgs->width / 8 * 5;
    area_maxx[1] = area_maxx[4] = area_maxx[7] = imgs->width / 8 * 5;

    area_miny[3] = area_miny[4] = area_miny[5] = imgs->height / 4;
    area_maxy[0] = area_maxy[1] = area_maxy[2] = imgs->height / 4;

    area_miny[6] = area_miny[7] = area_miny[8] = imgs->height / 4 * 3;
    area_maxy[3] = area_maxy[4] = area_maxy[5] = imgs->height / 4 * 3;

    area_maxx[2] = area_maxx[5] = area_maxx[8] = imgs->width;
    area_maxy[6] = area_maxy[7] = area_maxy[8] = imgs->height;

    // TO DO : optimisation should not waste time with center area
    for (i = 0; i < 9; i++) {
      if (cent->x >= area_minx[i] && cent->x <= area_maxx[i] &&
          cent->y >= area_miny[i] && cent->y <= area_maxy[i]) {
          break;
         }
    }
    
    if(i!=4) {   //if motion is not in the center (ie no camera movement)
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: Camera %s PTZ ready, move command received, motion location x=%d, y=%d", cnt->netcam->connect_host, cent->x, cent->y );
    }

//    MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Motion in area %d detected.", cnt->netcam->connect_host, i + 1);
    switch (i) {
    case 0:
        step_size = (int)sqrt((delta_y*delta_y)+(delta_x*delta_x)); //replaces above
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Motion in area %d detected. LEFT_UP %d", cnt->netcam->connect_host, i+1, step_size);
        break;
    case 1:
        step_size = delta_y;
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Motion in area %d detected. UP %d", cnt->netcam->connect_host, i+1, step_size);
        break;
    case 2:
        step_size = (int)sqrt((delta_y*delta_y)+(delta_x*delta_x)); //replaces above
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Motion in area %d detected. RIGHT_UP %d", cnt->netcam->connect_host, i+1, step_size);
        break;
    case 3:
        step_size = delta_x;
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Motion in area %d detected. LEFT %d", cnt->netcam->connect_host, i+1, step_size);
        break;
    case 4:
        MOTION_LOG(DBG, TYPE_TRACK, NO_ERRNO, "%s: Cam %s Motion in CENTRAL area - not Moving", cnt->netcam->connect_host);
        return 0;
    case 5:
        step_size = delta_x;
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Motion in area %d detected. RIGHT %d", cnt->netcam->connect_host, i+1, step_size);
        break;
    case 6:
        step_size = (int)sqrt((delta_y*delta_y)+(delta_x*delta_x)); //replaces above
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Motion in area %d detected. LEFT_DOWN %d", cnt->netcam->connect_host, i+1, step_size);
        break;
    case 7:
        step_size = delta_y;
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Motion in area %d detected. DOWN %d", cnt->netcam->connect_host, i+1, step_size);
        break;
    case 8:
        step_size = (int)sqrt((delta_y*delta_y)+(delta_x*delta_x)); //replaces above
        MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Motion in area %d detected. RIGHT_DOWN %d", cnt->netcam->connect_host, i+1, step_size);
        break;
    default:
        MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: cam %s Motion in area %d detected motion location x=%d, y:%d - WRONG AREA Detection.", cnt->netcam->connect_host, i+1, cent->x, cent->y);
        return 0;
    }
	
	/* Assign new direction and step size */
    //pthread_mutex_lock(&global_lock);
	cnt->track.direction=i;
	cnt->track.step_size=step_size;
    //pthread_mutex_unlock(&global_lock);
  
	if (cnt->track.thread_id == 0 ) {	
						
		pthread_attr_init(&handler_attribute);
		pthread_attr_setdetachstate(&handler_attribute, PTHREAD_CREATE_DETACHED);
		pthread_mutex_lock(&global_lock);
		cnt->track.thread_id = ++threads_running;
		pthread_mutex_unlock(&global_lock);
		/* create thread*/  
		MOTION_LOG(INF, TYPE_TRACK, NO_ERRNO, "%s: Starting cam %s PTZ thread: %d", cnt->netcam->connect_host, cnt->track.thread_id );
			if(pthread_create (&cnt->track.thread_id, &handler_attribute, &FOSCAM_HD_move_thread, cnt) < 0) {
				MOTION_LOG(ERR, TYPE_TRACK, SHOW_ERRNO, "%s: Unable to create cam %s Pthread, [%d]", cnt->netcam->connect_host, cnt->track.thread_id);
				return (0);
			}
    }

    /* 
    Number of frames to skip while moving :
    The number of frames to skip depends on cam fps and driver speed depends 
    on FOSCAM_HD PTz external set speed. Thus using cnt->lastrate, we only have to assumes that cnt->track.speed
    is set to speed in nb pixels move per seconds the cam is actually doing.
    */
    int no_motion_detect_frames = (1 + ((cnt->lastrate * step_size) / cnt->track.speed));
	MOTION_LOG(NTC, TYPE_TRACK, NO_ERRNO, "%s: cam %s Skipping detection on %d frame(s) while camera is moving as fps is %d",
		cnt->netcam->connect_host, no_motion_detect_frames + cnt->track.move_wait, cnt->lastrate);
	return no_motion_detect_frames + cnt->track.move_wait;	
}
