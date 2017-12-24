package org.firstinspires.ftc.teamcode;
/* *
 * Created by ftcrobotics on 11/19/17.
 * Concept by Howard
 * First Coding by Jeffrey and Alexis
 **/
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
@SuppressWarnings("WeakerAccess")
@TeleOp(name = "KP Driver Cmd Mecanum", group = "k9Bot")

public class KP_DriverCmd_Mecanum extends LinearOpMode {
    /* Declare OpMode members. */
    hardwarePushBotKP robot   = new hardwarePushBotKP();   // Use a Pushbot's hardware
    Gamepad g1 = new Gamepad();
    Gamepad g2 = new Gamepad();
    final long TELEPERIOD = 50;
    final long TELEMETRYPERIOD = 1000;


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

        long CurrentTime = System.currentTimeMillis();

        long LastSensor = CurrentTime;
        long LastEncoderRead = CurrentTime + 5;
        long LastServo = CurrentTime + 10;
        long LastNav = CurrentTime + 15;
        long LastMotor = CurrentTime + 20;
        long LastController = CurrentTime + 7;
        long LastTelemetry = CurrentTime + 17;

        // variables for controller inputs.
        g1 = gamepad1;
        g2 = gamepad2;


        int g2_A_Counts = 0;
        int g2_DU_Counts = 0;

        double leftClamp_Cmd  = robot.LEFTUNCLAMPED;
        double rightClamp_Cmd = robot.RIGHTUNCLAMPED;

        float leftDriveCmd = 0;
        float rightDriveCmd = 0;
        float riserCmd = 0;

        float turtleScaler = 1;  //Initially full power
        float turtleSpeed  = 4;  // Divider

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();


        //A Timing System By Katherine Jeffrey,and Alexis
        // long currentThreadTimeMillis (0);
        //
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
            if (CurrentTime - LastSensor > TELEPERIOD) {
                LastSensor = CurrentTime;

                // no sensors at this time.  If we add some, change this comment.
            }


        /* ***************************************************
         *                ENCODERS                          *
         ****************************************************/
            if (CurrentTime - LastEncoderRead > TELEPERIOD) {
                LastEncoderRead = CurrentTime;
                // We want to READ the Encoders here  Not currently using data, invalid
                //    ONLY set the motors in motion in ONE place.
                // NO ENCODERS AT THIS TIME
              //  rightMotorPos = robot.rightDrive.getCurrentPosition();
                //lefMotorPos   = robot.leftDrive.getCurrentPosition();
                //riserMotorPos = robot.pulleyDrive.getCurrentPosition();

            }
        /* **************************************************
         *                Controller INPUT                  *
         *  INPUTS: raw controller values                   *
         *  OUTPUTS:
         *         g1_LeftX    g1_RightX
         *         g1_LeftY    g1_RightY
         *         g1_a (gamepad A)
         *         g1_b (gamepad B)
         ****************************************************/
            if (CurrentTime - LastController > TELEPERIOD) {
                LastController = CurrentTime;
                g1 = gamepad1;
                g2 = gamepad2;


                // do a little debounce on gamepad1.a so we don't drop the block accidentally
                // 6 counts at 30 milliseconds will delay things by 180 ms, and that allows
                // a flaky controller or a jittery operator. Splitting out the sensor "reads"
                // from the rest of the logic let's us do this here, rather than muddling up
                // the main logic of a more abstract, more complicated piece of code.
                g2_A_Counts = Range.clip((gamepad2.a)? g2_A_Counts + 1 : g2_A_Counts - 1, 0,12);
                g2.a = (g2_A_Counts >= 6);

                g2_DU_Counts = Range.clip((gamepad2.dpad_up)? g2_DU_Counts + 1 : g2_DU_Counts - 1, 0,12);
                g2.dpad_up = (g2_DU_Counts >= 6);



    /*  ***********************************************************************
     ^^^^^^^^^^^^^ ALL OF THE STUFF ABOVE HERE IS READING INPUTS ^^^^^^^^^^^^^^^
     ***************************************************************************/



    /* ********************************************************************************/
    /* vvvvvvvvvvvvvvvvvv  THIS SECTION IS MAPPING INPUTS TO OUTPUTS vvvvvvvvvvvvvvvvv*/

        /* **************************************************
         *                NAV
         *      Inputs:  Gamepad positions
         *               Sensor Values (as needed)
         *      Outputs: Servo and Motor position commands
         *                         motor
         ****************************************************/
                if (CurrentTime - LastNav > TELEPERIOD) {
                    LastNav = CurrentTime;

                    // init drive min and max to default values.  We'll reset them to other numbers
                    // if conditions demand it.
                    float driveMax = 1;
                    float driveMin = -1;
                    float riserMax = 1;
                    float riserMin = -1;
                    double riserTarget = 0;

              //Simple mapping of controller to test KP bot

                    //Control riser motor
                    riserCmd = 0;
                    if (g2.right_trigger > 0) {    //UP
                        riserCmd = riserMax;
                    }
                    if (g2.left_trigger > 0) {    //DOWN
                        riserCmd = riserMin;
                    }

                    //Control Servos for Clamp
                    if (g2.b ){
                        leftClamp_Cmd = robot.LEFTCLAMPED;
                        rightClamp_Cmd = robot.RIGHTCLAMPED;
                    }
                    if (g2.a) {
                        leftClamp_Cmd = robot.LEFTUNCLAMPED;
                        rightClamp_Cmd = robot.RIGHTUNCLAMPED;
                    }
                    if (g2.x) {  // Bias to left
                        leftClamp_Cmd = leftClamp_Cmd   -robot.SERVO_TWEAK;
                        rightClamp_Cmd = rightClamp_Cmd +robot.SERVO_TWEAK;
                    }
                    if (g2.y) {  // Bias to right
                        leftClamp_Cmd = leftClamp_Cmd   +robot.SERVO_TWEAK;
                        rightClamp_Cmd = rightClamp_Cmd -robot.SERVO_TWEAK;
                    }

                    if (g2.dpad_up) { // Mostly Clamped
                        leftClamp_Cmd = robot.LEFTMOSTLYCLAMPED;
                        rightClamp_Cmd = robot.RIGHTMOSTLYCLAMPED;
                    }

                    if (g2.dpad_down) { // Extra Clamped
                        leftClamp_Cmd = robot.LEFTTIGHTCLAMPED;
                        rightClamp_Cmd = robot.RIGHTTIGHTCLAMPED;
                    }

//
                    //Turtle Mode toggle
                    if ((g1.right_bumper) || (g1.right_trigger > 0)) {  //Turtle
                        turtleScaler = turtleSpeed;
                    }
                    if ((g1.left_bumper) || (g1.left_trigger > 0)) {  // Exit Turtle
                        turtleScaler = driveMax;
                    }

                    // mapping inputs to motor commands - cube them to desensetize them around
                    // the 0,0 point.  Switching to single stick operation ought to be pretty
                    // straightforward, if that's desired.  Using 2 sticks was simpler to
                    // code up in a hurry.
                    g1.left_stick_y  = (g1.left_stick_y  * g1.left_stick_y  * g1.left_stick_y) / turtleScaler;
                    g1.right_stick_y = (g1.right_stick_y * g1.right_stick_y * g1.right_stick_y) / turtleScaler;


                    // The ONLY place we set the motor power variables. Set them here, and
                    // we will never have to worry about which set is clobbering the other.
                    // I aligned them this way to make it REALLY clear what's going on.
                    // Should probably think through the logic of doing the same with the servos
                    // Ideally we'd calculate a desired motor or servo action then apply any
                    // necessary clamps or limits (or overrides) right before shipping it out.

                    // Servo commands: Clipped and Clamped.

                    // motor commands: Clipped & clamped.
                    leftDriveCmd  = Range.clip(g1.left_stick_y,  driveMin, driveMax);
                    rightDriveCmd = Range.clip(g1.right_stick_y, driveMin, driveMax);
                    riserCmd      = Range.clip((float)riserCmd,  riserMin, riserMax);
                }                    // END NAVIGATION


    /*   ^^^^^^^^^^^^^^^^  THIS SECTION IS MAPPING INPUTS TO OUTPUTS   ^^^^^^^^^^^^^^^*/
    /* ********************************************************************************/



    /*  ***********************************************************************
     * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV
     *           ALL OF THE STUFF BELOW HERE IS WRITING OUTPUTS
     * VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV*/


        /* **************************************************
         *                SERVO OUTPUT
         *                Inputs: leftClamp position command
         *                        rightClamp position command *
         *                Outputs: Physical write to servo interface.
         ****************************************************/
                if (CurrentTime - LastServo > TELEPERIOD) {
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
                if (CurrentTime - LastMotor > TELEPERIOD) {
                    LastMotor = CurrentTime;
                    // Yes, we'll set the power each time, even if it's zero.
                    // this way we don't accidentally leave it somewhere.  Just simpler this way.
                /*  Left Drive Motor Power  */
                    //robot.leftDrive.setPower(leftDriveCmd);
                    robot.leftDrive.setPower(-1*rightDriveCmd);

                /*  Right Drive Motor Power */
                    // robot.rightDrive.setPower(rightDriveCmd);
                    robot.rightDrive.setPower(-1*leftDriveCmd);

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
                    telemetry.update();
                }
            }

            telemetry.addData("Left Motor Power     ", leftDriveCmd);
            telemetry.addData("Right Motor Power    ", rightDriveCmd);
            telemetry.addData("Riser Motor Power    ", riserCmd);
            telemetry.addData("Left Clamp Command   ", leftClamp_Cmd);
            telemetry.addData("Right Clamp Command  ", rightClamp_Cmd);
            telemetry.update();
        }


        //SAFE EXIT OF RUN OPMODE, stop motors, leave servos????
        robot.liftMotor.setPower(0);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
}
