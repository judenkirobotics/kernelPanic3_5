package org.firstinspires.ftc.teamcode;
/* *
 * Created by ftcrobotics on 11/19/17.
 * Concept by Howard
 * First Coding by Jeffrey and Alexis
 **/
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@SuppressWarnings("WeakerAccess")
@TeleOp(name = "KP Driver Cmd Mecanum", group = "k9Bot")

public class KP_DriverCmd_Mecanum extends LinearOpMode {
    /* Declare OpMode members. */
    hardwarePushBotKP robot = new hardwarePushBotKP();
    Drive2 myDrive = new Drive2();
    //hardwarePushBotKP robot   = new hardwarePushBotKP();   // Use a Pushbot's hardware
    Gamepad g1 = new Gamepad();
    Gamepad g2 = new Gamepad();
    final float DRIVEMAX = 1;
    final float DRIVEMIN = -1;
    final float LIFTMAX = 1;
    final float LIFTMIN = -1;
    final long MINORFRAME = 50;
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
        botMotors mPwr = new botMotors();

        double leftClamp_Cmd = robot.LEFTUNCLAMPED;
        double rightClamp_Cmd = robot.RIGHTUNCLAMPED;

        float leftDriveCmd = 0;
        float rightDriveCmd = 0;
        float leftRearCmd = 0;
        float rightRearCmd = 0;
        float riserCmd = 0;

        float turtleScaler = 1;  //Initially full power
        float turtleSpeed = 4;  // Divider

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime runtime = new ElapsedTime();

        //A Timing System By Katherine, Jeffrey, and Alexis
        // long currentThreadTimeMillis (0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
    /* ******************************************************************************************
     *****************************              CODE            *********************************
     *                           Everything below here  \\ press START                          *
     ********************************************************************************************/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            CurrentTime = System.currentTimeMillis();
            //Loop For Timing System
            /* ***************************************************
             *                SENSORS
             *        INPUTS: Raw Sensor Values
             *       OUTPUTS: parameters containing sensor values*
             ****************************************************/
            if (CurrentTime - LastSensor > MINORFRAME) {
                LastSensor = CurrentTime;

                // no sensors at this time.  If we add some, change this comment.
            }

            /* ***************************************************
             *                ENCODERS                          *
             ****************************************************/
            if (CurrentTime - LastEncoderRead > MINORFRAME) {
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
             ****************************************************/
            if (CurrentTime - LastController > MINORFRAME) {
                LastController = CurrentTime;
                g1 = gamepad1;
                g2 = gamepad2;

                // do a little debounce on gamepad1.a so we don't drop the block accidentally
                // 6 counts at 30 milliseconds will delay things by 180 ms, and that allows
                // a flaky controller or a jittery operator. Splitting out the sensor "reads"
                // from the rest of the logic let's us do this here, rather than muddling up
                // the main logic of a more abstract, more complicated piece of code.
                g2_A_Counts = Range.clip((gamepad2.a) ? g2_A_Counts + 1 : g2_A_Counts - 1, 0, 12);
                g2.a = (g2_A_Counts >= 6);

                g2_DU_Counts =
                        Range.clip((gamepad2.dpad_up) ? g2_DU_Counts + 1 : g2_DU_Counts - 1, 0, 12);
                g2.dpad_up = (g2_DU_Counts >= 6);
            }

            /* **************************************************
             *                NAV
             *      Inputs:  Gamepad positions
             *               Sensor Values (as needed)
             *      Outputs: Servo and Motor position commands
             *                         motor
             ****************************************************/
            if (CurrentTime - LastNav > MINORFRAME) {
                LastNav = CurrentTime;

                // init drive min and max to default values.  We'll reset them to other numbers
                // if conditions demand it.

                //double riserTarget = 0;

                //Control riser motor
                riserCmd = 0;
                if (g2.right_trigger > 0) {    //UP
                    riserCmd = LIFTMAX;
                }
                if (g2.left_trigger > 0) {    //DOWN
                    riserCmd = LIFTMIN;
                }

                //Control Servos for Clamp
                if (g2.b) {
                    leftClamp_Cmd = robot.LEFTCLAMPED;
                    rightClamp_Cmd = robot.RIGHTCLAMPED;
                }
                if (g2.a) {
                    leftClamp_Cmd = robot.LEFTUNCLAMPED;
                    rightClamp_Cmd = robot.RIGHTUNCLAMPED;
                }
                if (g2.x) {  // Bias to left
                    leftClamp_Cmd = leftClamp_Cmd - robot.SERVO_TWEAK;
                    rightClamp_Cmd = rightClamp_Cmd + robot.SERVO_TWEAK;
                }
                if (g2.y) {  // Bias to right
                    leftClamp_Cmd = leftClamp_Cmd + robot.SERVO_TWEAK;
                    rightClamp_Cmd = rightClamp_Cmd - robot.SERVO_TWEAK;
                }
                if (g2.dpad_up) { // Mostly Clamped
                    leftClamp_Cmd = robot.LEFTMOSTLYCLAMPED;
                    rightClamp_Cmd = robot.RIGHTMOSTLYCLAMPED;
                }
                if (g2.dpad_down) { // Extra Clamped
                    leftClamp_Cmd = robot.LEFTTIGHTCLAMPED;
                    rightClamp_Cmd = robot.RIGHTTIGHTCLAMPED;
                }

                //Turtle Mode toggle
                if ((g1.right_bumper) || (g1.right_trigger > 0)) {  //Turtle
                    turtleScaler = turtleSpeed;
                }
                if ((g1.left_bumper) || (g1.left_trigger > 0)) {  // Exit Turtle
                    turtleScaler = DRIVEMAX;
                }

                // mapping inputs to motor commands - cube them to desensetize them around
                // the 0,0 point.
                g1.left_stick_y = (float) Math.pow((double) g1.left_stick_y, (double) 3);
                g1.left_stick_y = g1.left_stick_y / turtleScaler;

                g1.right_stick_y = (float) Math.pow((double) g1.left_stick_y, (double) 3);
                g1.right_stick_y = g1.right_stick_y / turtleScaler;

                if (g1.x)
                {
                    mPwr.leftFront = g1.left_stick_y;
                    mPwr.rightFront = g1.right_stick_y;
                    mPwr.leftRear = mPwr.leftFront;
                    mPwr.rightRear = mPwr.rightFront;
                }
                else
                {
                    mPwr = myDrive.crab(g1);
                }
                // The ONLY place we set the motor power variables. Set them here, and
                // we will never have to worry about which set is clobbering the other.

                // motor commands: Clipped & clamped.
                leftDriveCmd = Range.clip(mPwr.leftFront, DRIVEMIN, DRIVEMAX);
                rightDriveCmd = Range.clip(mPwr.rightFront, DRIVEMIN, DRIVEMAX);
                leftRearCmd = Range.clip(mPwr.leftRear, DRIVEMIN, DRIVEMAX);
                rightRearCmd = Range.clip(mPwr.rightRear, DRIVEMIN, DRIVEMAX);
                riserCmd = Range.clip(riserCmd, DRIVEMIN, DRIVEMAX);
            }                    // END NAVIGATION

            /* **************************************************
             *                SERVO OUTPUT
             *                Inputs: leftClamp position command
             *                        rightClamp position command *
             *                Outputs: Physical write to servo interface.
             ****************************************************/
            if (CurrentTime - LastServo > MINORFRAME) {
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
            if (CurrentTime - LastMotor > MINORFRAME) {
                LastMotor = CurrentTime;
                // Yes, we'll set the power each time, even if it's zero.
                // this way we don't accidentally leave it somewhere.  Just simpler this way.
                robot.leftDrive.setPower(-1 * rightDriveCmd);
                robot.rightDrive.setPower(-1 * leftDriveCmd);
                robot.leftRear.setPower(-1*rightRearCmd);
                robot.rightRear.setPower(-1*leftRearCmd);
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
            telemetry.addData("Left Motor Power     ", leftDriveCmd);
            telemetry.addData("Right Motor Power    ", rightDriveCmd);
            telemetry.addData("Riser Motor Power    ", riserCmd);
            telemetry.addData("Left Clamp Command   ", leftClamp_Cmd);
            telemetry.addData("Right Clamp Command  ", rightClamp_Cmd);
            telemetry.update();
        }

        //SAFE EXIT OF RUN OPMODE, stop motors, leave servos????
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightRear.setPower(0);
        robot.liftMotor.setPower(0);
    }
}
