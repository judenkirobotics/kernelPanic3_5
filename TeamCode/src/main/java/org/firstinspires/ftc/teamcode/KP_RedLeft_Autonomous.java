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


@Autonomous(name="Generic Autonomous", group="Pushbot")
@SuppressWarnings("WeakerAccess")
public class KP_RedLeft_Autonomous extends LinearOpMode {
    /* Declare OpMode members. */
    //Drive myDrive = new Drive(DCMotor mLeft,DcMotor mRight);

    hardwarePushBotKP robot   = new hardwarePushBotKP();   // Use a Pushbot's hardware
    static final double LEFTCLAMPED = 45;
    static final double LEFTUNCLAMPED = -5;
    static final double RIGHTCLAMPED = 5;
    static final double RIGHTUNCLAMPED = -45;
    static final int CL = 1;
    static final int UC = -1;
    static final float driveMax = 1;
    static final float driveMin = -1;
    static final float riserMax = 1;
    static final float riserMin = -1;

    static final int  liftMaxTime = 600;
    static final long MINOR_FRAME = 50;
    static final long TELEMETRYPERIOD = 1000;

    // states for the NAV switch statement
    final int CLM = 0;
    final int TRN = 1;
    final int STR = 2;
    final int LFT = 3;
    final int WAIT = 4;

    int CurrentAutoState = 0;
    //int rightMotorPos;
    //int lefMotorPos;
    //int riserMotorPos;
/*
    public boolean detectItem() {
        // presuming there will be a detect item here ... populate this code when we know that
        return true;
    }
    public boolean moveLever() {
        // presuming we will move a lever somehow.  Populate this method when that is known.
        return true;
    }
*/
    @Override

    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        Drive2 myDrive = new Drive2();
        botMotors dPwr;

        long CurrentTime = System.currentTimeMillis();

        long LastSensor = CurrentTime;
        long LastEncoderRead = CurrentTime + 5;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastTelemetry = CurrentTime + 17;


        double leftClamp_Cmd = LEFTUNCLAMPED;
        double rightClamp_Cmd = RIGHTUNCLAMPED;
        long stageTimer=0;


        //int startHeading = 0;
        //int currentHeading = 0;

        float leftDriveCmd = 0;
        float rightDriveCmd = 0;
        float riserCmd = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();
        //A Timing System By Katherine Jeffrey,and Alexis
       // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        int[] thisCase =   {  CLM,  LFT,  TRN,  STR,  CLM,  TRN, WAIT};
        int[] clampArray=    { CL,    0,    0,    0,   UC,    0,    0};
        double[] stateDur =  {500, 2000, 3000, 1000, 1000, 2000,  500};
        //int[] TurnArray =    {  0,    0,   45,    0,    2,    0,    0};
        //int[] TurnPower =    {  0,    0,   40,    0,  -30,    0,    0};
        float[] StraightPwr= { 25,    0,    0,   30,    0,    0,    0};
        long[] StraightTime=  { 10,    0,    0,   50,    0,    0,    0};

  /* ***********************************************************************************************
   *****************************       OpMode    CODE          ************************************
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
            if (CurrentTime - LastSensor > MINOR_FRAME) {
                LastSensor = CurrentTime;
                //currentHeading = robot.gyro.getHeading();
                // no sensors at this time.  If we add some, change this comment.
            }


            /* ***************************************************
             *                ENCODERS                          *
             ****************************************************/
            if (CurrentTime - LastEncoderRead > MINOR_FRAME) {
                LastEncoderRead = CurrentTime;
                // We want to READ the Encoders here
                //    ONLY set the motors in motion in ONE place.
                //rightMotorPos = robot.rightDrive.getCurrentPosition();
                //lefMotorPos = robot.leftDrive.getCurrentPosition();
                //riserMotorPos = robot.liftMotor.getCurrentPosition();
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
            if (CurrentTime - LastNav > MINOR_FRAME) {
                LastNav = CurrentTime;
                boolean stageComplete = false;
                // init drive min and max to default values.  We'll reset them to other numbers
                // if conditions demand it.
                //double maxT = stateDur[CurrentAutoState];

                stageTimer += MINOR_FRAME;

                switch ( thisCase[CurrentAutoState] ) {
                    case CLM:  //Close clamp on cube
                        leftClamp_Cmd = LEFTUNCLAMPED;
                        rightClamp_Cmd = RIGHTUNCLAMPED;
                        if (clampArray[CurrentAutoState] == CL) {
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
                        break;
                    case STR:   // Drive forward
                        long goal = StraightTime[CurrentAutoState];
                        float pwr  = StraightPwr[CurrentAutoState];
                        dPwr = myDrive.fwd6(pwr, goal, stageTimer);
                        leftDriveCmd = dPwr.leftFront;
                        rightDriveCmd = dPwr.rightFront;
                        if (dPwr.status < 0) {
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
                    //startPos = currentPos;
                    //startHeading = currentHeading;
                    stageTimer= 0;
                    CurrentAutoState++;

                }
                // mapping inputs to servo command

                // The ONLY place we set the motor power request. Set them here, and
                // we will never have to worry about which set is clobbering the other.

                // Servo commands: Clipped and Clamped.

                // motor commands: Clipped & clamped.
                leftDriveCmd  = Range.clip(leftDriveCmd,         driveMin, driveMax);
                rightDriveCmd = Range.clip(rightDriveCmd,         driveMin, driveMax);
                riserCmd      = Range.clip(riserCmd, riserMin, riserMax);
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
            if (CurrentTime - LastServo > MINOR_FRAME) {
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
            if (CurrentTime - LastMotor > MINOR_FRAME) {
                LastMotor = CurrentTime;
                // Yes, we'll set the power each time, even if it's zero.
                // this way we don't accidentally leave it somewhere.  Just simpler this way.
                /*  Left Drive Motor Power  */
                robot.leftDrive.setPower(leftDriveCmd);

                /*  Right Drive Motor Power */
                robot.rightDrive.setPower(rightDriveCmd);

                /* Lifter Motor Power   */
                robot.liftMotor.setPower(riserCmd);
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
        //SAFE EXIT OF RUN OPMODE, stop motors, leave servos????
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        robot.liftMotor.setPower(0);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
