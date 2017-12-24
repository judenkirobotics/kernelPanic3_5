package org.firstinspires.ftc.teamcode;

/**
 * Created by howard on 12/23/17.
 */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;

public class hardwarePushBotKP {
    final double LEFTCLAMPED = 45;
    final double LEFTUNCLAMPED = -5;
    final double RIGHTCLAMPED = 5;
    final double RIGHTUNCLAMPED = -45;
    final double SERVO_TWEAK    = 2;
    final double LEFTMOSTLYCLAMPED = 42;
    final double RIGHTMOSTLYCLAMPED= -42;
    final double LEFTTIGHTCLAMPED = 48;
    final double RIGHTTIGHTCLAMPED= -48;
    //static final double CLAMP_MOTION_TIME = 250;
    //rampMotor  = hwMap.get(DcMotor.class, "beltmotor");
    //loaderMotor= hwMap.get(DcMotor.class, "loadermotor");

    // Define and Initialize Servos
    //leftClamp = ahwMap.get(Servo.class, "left_clamp");
    public GyroSensor  gyro           = null;
    public TouchSensor extensionTouch = null;
    public Servo       leftClamp      = null;
    public Servo       rightClamp     = null;
    public DcMotor     leftDrive      = null;
    public DcMotor     rightDrive     = null;
    public DcMotor     liftMotor      = null;
    public DcMotor     leftRear       = null;
    public DcMotor     rightRear      = null;

    /* local OpMode members. */
    //private ElapsedTime period  = new ElapsedTime();

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        //HardwareMap hwMap          = ahwMap;
        gyro = ahwMap.get(GyroSensor.class, "gyro");
        rightClamp = ahwMap.get(Servo.class, "right_clamp");
        leftClamp = ahwMap.get(Servo.class, "left_clamp");

        //Define and Initialize Sensors
        extensionTouch = ahwMap.get(TouchSensor.class, "ext_touch");

        // Define and Initialize Motors
        DcMotor leftDrive  = ahwMap.get(DcMotor.class, "left_drive");
        DcMotor rightDrive = ahwMap.get(DcMotor.class, "right_drive");
        DcMotor leftRear  = ahwMap.get(DcMotor.class, "left_rear");
        DcMotor rightRear = ahwMap.get(DcMotor.class, "right_rear");
        DcMotor liftMotor = ahwMap.get(DcMotor.class, "liftmotor");

        leftDrive.setDirection(DcMotor.Direction.FORWARD); //Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);//Set to FORWARD if using AndyMark motors
        leftRear.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        //rampMotor.setDirection(DcMotor.Direction.FORWARD);
        //loaderMotor.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to run without encoders.

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //loaderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        //rampMotor.setPower(0);
        //loaderMotor.setPower(0);
        liftMotor.setPower(0);
    }
}