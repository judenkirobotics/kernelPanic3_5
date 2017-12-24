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
    /* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */




/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */

    static final double LEFTCLAMPED = 45;
    static final double LEFTUNCLAMPED = -5;
    static final double RIGHTCLAMPED = 5;
    static final double RIGHTUNCLAMPED = -45;
    static final double SERVO_TWEAK    = 2;
    static final double LEFTMOSTLYCLAMPED = 42;
    static final double RIGHTMOSTLYCLAMPED= -42;
    static final double LEFTTIGHTCLAMPED = 48;
    static final double RIGHTTIGHTCLAMPED= -48;
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


        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
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