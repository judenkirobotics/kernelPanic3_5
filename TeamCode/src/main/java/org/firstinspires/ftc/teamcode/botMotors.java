package org.firstinspires.ftc.teamcode;

/**
 * Created by howard on 12/17/17.
 */

public class botMotors
{
    // botMotors is a structure used to pass motor command information from
    // a gyro turn or a fwd method back to the calling routine.
    // It is set up to support four independent motors.

    float leftFront;
    float rightFront;
    float leftRear;
    float rightRear;
    int status;

    // status of 0 means the "turn" or "fwd" method is proceeding OK
    // status of -1 should be used to indicate the arrival has been reached
    // status of -2 should be used to indicate the activity has timed out.
}
