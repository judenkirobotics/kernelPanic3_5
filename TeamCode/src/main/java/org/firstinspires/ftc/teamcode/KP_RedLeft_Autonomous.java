package org.firstinspires.ftc.teamcode;

/* *
 * Created by ftcrobotics on 11/19/17.
 * Concept by Howard
 * First Coding by Jeffrey and Alexis
 */


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@Autonomous(name="Generic Autonomous", group="Pushbot")
@SuppressWarnings("WeakerAccess")
public class genericAutonomous extends LinearOpMode {
    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    static final double LEFTCLAMPED = 45;
    static final double LEFTUNCLAMPED = -5;
    static final double RIGHTCLAMPED = 5;
    static final double RIGHTUNCLAMPED = -45;
    static final int CLAMPED = 1;
    static final int UNCLAMPED = 2;

    final long SENSORPERIOD = 50;
    final long ENCODERPERIOD = 50;
    final long SERVOPERIOD = 50;
    final long NAVPERIOD = 50;
    final long MOTORPERIOD = 50;
    final long TELEMETRYPERIOD = 1000;

    // states for the NAV switch statement
    final int CLM = 0;
    final int TRN = 1;
    final int STR = 2;
    final int LFT = 3;
    final int WAIT = 4;

    int CurrentAutoState = 0;
    int rightMotorPos;
    int lefMotorPos;
    int riserMotorPos;

    public boolean detectItem() {
        // presuming there will be a detect item here ... populate this code when we know that
        return true;
    }
    public boolean moveLever() {
        // presuming we will move a lever somehow.  Populate this method when that is known.
        return true;
    }

    @Override

    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        int[] thisCase =   {CLM,  LFT, TRN, STR,  CLM, WAIT};
        int[] clampArray=    {1,   0,   0,  -1,    0,    0,  0};
        double[] stateDur =  {500, 0,   0,   0,    0,  500,  0};
        int[] TurnArray =    {0,   0,  45,   0,    2,    0,  0};
        int[] TurnPower =    {0,   0,  40,   0,  -30,    0,  0};
        float[] StraightPwr= {25,  0,   0,  30,    0,    0,  0};
        int[] StraightDist=  {10,  0,   0,  50,    0,    0,  0};

        long CurrentTime = System.currentTimeMillis();

        long LastSensor = CurrentTime;
        long LastEncoderRead = CurrentTime + 5;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastController = CurrentTime + 7;
        long LastTelemetry = CurrentTime + 17;

        long liftDuration = 0;
        long liftOffDuration = 0;

        double leftClamp_Cmd = LEFTUNCLAMPED;
        double rightClamp_Cmd = RIGHTUNCLAMPED;
        double stageTimer=0;

        float startTime = 0;
        int startHeading = 0;
        double startPos = 0;
        float leftDriveCmd = 0;
        float rightDriveCmd = 0;
        float riserCmd = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();
        //A Timing System By Katherine Jeffrey,and Alexis
        // long currentThreadTimeMillis (0);
        //
        //int riserZero = robot.pulleyDrive.getCurrentPosition();

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();
  /* ***********************************************************************************************
   *****************************                CODE          ************************************
   ************************************************************************************************/


        /* ************************************************************
         *            Everything below here  \\ press START           *
         **************************************************************/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            CurrentTime = System.currentTimeMillis();

            //Loop For Timing System
            /* ***************************************************
             *                SENSORS
             *        INPUTS: Raw Sensor Values
             *       OUTPUTS: parameters containing sensor values*
             ****************************************************/
            if (CurrentTime - LastSensor > SENSORPERIOD) {
                LastSensor = CurrentTime;
                // no sensors at this time.  If we add some, change this comment.
            }


            /* ***************************************************
             *                ENCODERS                          *
             ****************************************************/
            if (CurrentTime - LastEncoderRead > ENCODERPERIOD) {
                LastEncoderRead = CurrentTime;
                // We want to READ the Encoders here
                //    ONLY set the motors in motion in ONE place.
                rightMotorPos = robot.rightDrive.getCurrentPosition();
                lefMotorPos = robot.leftDrive.getCurrentPosition();
                riserMotorPos = robot.pulleyDrive.getCurrentPosition();
            }
            /* **************************************************
             *                Controller INPUT                  *
             *  INPUTS: raw controller values                   *
             *  OUTPUTS:                                        *
             *         NO CONTROLLER INPUT FOR AUTONOMOUS       *
        /*  ***********************************************************************
         ^^^^^^^^^^^^^ ALL OF THE STUFF ABOVE HERE IS READING INPUTS ^^^^^^^^^^^^^^^
         ***************************************************************************/
        /* vvvvvvvvvvvvvvvvvv  THIS SECTION IS MAPPING INPUTS TO OUTPUTS vvvvvvvvvvvvvvvvv*/
            /* **************************************************
             *                NAV
             *      Inputs:  Gamepad positions
             *               Sensor Values (as needed)
             *      Outputs: Servo and Motor position commands
             ****************************************************/
            if (CurrentTime - LastNav > NAVPERIOD) {
                LastNav = CurrentTime;
                boolean stageComplete = false;
                // init drive min and max to default values.  We'll reset them to other numbers
                // if conditions demand it.
                float driveMax = 1;
                float driveMin = -1;
                float riserMax = 1;
                float riserMin = -1;
                double riserTarget = 0;

                int    clampMaxTime = 500;
                int    liftMaxTime = 600;
                int    driveMaxTime = 2000;   //Crude two seconds, eventually use encoders
                int    driveBackMaxTime = 200;
                int    driveForwardLittleTime = 1000;
                int    driveBackLittleTime = 250;
                stageTimer += NAVPERIOD;

                switch ( thisCase[CurrentAutoState] ) {
                    case CLM:  //Close clamp on cube
                        leftClamp_Cmd = LEFTUNCLAMPED;
                        rightClamp_Cmd = RIGHTUNCLAMPED;
                        if (clampArray[CurrentAutoState] == CLAMPED) {
                            leftClamp_Cmd = LEFTCLAMPED;
                            rightClamp_Cmd = RIGHTCLAMPED;
                        }
                        if (stageTimer > stateDur[CurrentAutoState]) {
                            stageComplete = true;
                        }
                        break;
                    case LFT:    // Raise or lower the device
                        riserCmd = riserMax;
                        if (stageTimer > liftMaxTime) {
                            riserCmd = 0;
                            stageComplete = true;
                        }
                        stageTimer += NAVPERIOD;
                        break;
                    case STR:   // Drive forward
                        leftDriveCmd = driveMax;
                        rightDriveCmd = driveMax;
                        if (stageTimer > driveMaxTime) {
                            leftDriveCmd=0;
                            rightDriveCmd=0;
                            stageComplete = true;
                        }
                        break;
                    case TRN:  // turn  --- need to add this.
                        stageComplete = true;
                        break;
                    case WAIT:
                        if (stageTimer > stateDur[CurrentAutoState])
                        {
                            stageComplete = true;
                            // set all motors to stop?
                        }
                        break;
                    default:
                        if (CurrentAutoState < 65535)
                        {
                            CurrentAutoState++;
                            stageComplete = true;
                            // set all motors to stop.
                        }
                }
                if (stageComplete) {
                    startPos = currentPos;
                    startHeading = currentHeading;
                    startTime = CurrentTime;
                    stageTimer= 0;
                    CurrentAutoState++;
                }
                // mapping inputs to servo command

                // The ONLY place we set the motor power request. Set them here, and
                // we will never have to worry about which set is clobbering the other.

                // Servo commands: Clipped and Clamped.

                // motor commands: Clipped & clamped.
                leftDriveCmd  = Range.clip((float)leftDriveCmd,         driveMin, driveMax);
                rightDriveCmd = Range.clip((float)rightDriveCmd,         driveMin, driveMax);
                riserCmd      = Range.clip((float)riserCmd, riserMin, riserMax);
            }
            // END NAVIGATION
        /*   ^^^^^^^^^^^^^^^^  THIS SECTION IS MAPPING INPUTS TO OUTPUTS   ^^^^^^^^^^^^^^^
         *  ------------------------------------------------------------------------------
         *           ALL OF THE STUFF BELOW HERE IS WRITING OUTPUTS
         * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV*/


            /* **************************************************
             *                SERVO OUTPUT
             *                Inputs: leftClamp position command
             *                        rightClamp position command *
             *                Outputs: Physical write to servo interface.
             ****************************************************/
            if (CurrentTime - LastServo > SERVOPERIOD) {
                LastServo = CurrentTime;

                // Move both servos to new position.
                robot.leftClamp.setPosition(leftClamp_Cmd);
                robot.rightClamp.setPosition(rightClamp_Cmd);
            }


            /* ***************************************************
             *                MOTOR OUTPUT
             *       Inputs:  Motor power commands
             *       Outputs: Physical interface to the motors
             ****************************************************/
            if (CurrentTime - LastMotor > MOTORPERIOD) {
                LastMotor = CurrentTime;
                // Yes, we'll set the power each time, even if it's zero.
                // this way we don't accidentally leave it somewhere.  Just simpler this way.
                /*  Left Drive Motor Power  */
                robot.leftDrive.setPower(leftDriveCmd);

                /*  Right Drive Motor Power */
                robot.rightDrive.setPower(rightDriveCmd);

                /* Lifter Motor Power   */
                robot.pulleyDrive.setPower(riserCmd);
            }


            /* ***************************************************
             *                TELEMETRY
             *       Inputs:  telemetry structure
             *       Outputs: command telemetry output to phone
             ****************************************************/

            if (CurrentTime - LastTelemetry > TELEMETRYPERIOD) {
                LastTelemetry = CurrentTime;
                telemetry.addData("Switch State ", CurrentAutoState);
                telemetry.addData("Switch Timer ", stageTimer );
                telemetry.update();
            }
        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
