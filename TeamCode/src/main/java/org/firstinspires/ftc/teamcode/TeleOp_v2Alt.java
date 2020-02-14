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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp version 2.0 Double", group="Main")

public class TeleOp_v2Alt extends OpMode
{
    // Declare OpMode members.
//    private DriveTrain driveTrain;

    private Robot robot;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    private Servo grabbyLeft;
    private Servo grabbyRight;

    private Servo bigGrabby;
    private DcMotor lifty;

    private BNO055IMU imu;

    private double x;
    private double y;
    private double z;

    double leftPos = 0.0;
    double rightPos = 0.0;
    double bigPos = 0.0;


    private int encoder;
    private int encoderMin;
    private int encoderMax;

    private boolean bigGrab;
    private boolean pressed1;
    private boolean lilGrab;
    private boolean pressed2;

    private boolean slowMode;
    private boolean pressed3;

    private boolean done = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
//        leftFront = hardwareMap.dcMotor.get("leftFront");
//        rightFront = hardwareMap.dcMotor.get("rightFront");
//        leftRear = hardwareMap.dcMotor.get("leftRear");
//        rightRear = hardwareMap.dcMotor.get("rightRear");
//
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        grabbyLeft = hardwareMap.servo.get("leftGrabber");
        grabbyRight = hardwareMap.servo.get("rightGrabber");

        bigGrabby = hardwareMap.servo.get("mainGrabber");
        lifty = hardwareMap.dcMotor.get("lift");

//        imu = hardwareMap.get(BNO055IMU.class, "imu");


//        driveTrain = new DriveTrain(leftFront, rightFront, leftRear, rightRear, imu, telemetry);
        robot = new Robot(hardwareMap, "leftFront", "rightFront", "leftRear", "rightRear", "imu", telemetry);

        lifty.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lifty.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifty.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        encoderMin = 0;
        encoderMax = encoderMin - 4700;

        grabbyLeft.setPosition(1);
        grabbyRight.setPosition(0);
        bigGrabby.setPosition(0.95);

        bigGrab = false;
        pressed1 = false;
        lilGrab = false;
        pressed2 = false;

        slowMode = false;
        pressed3 = false;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

//        driveTrain.resetEncoders();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        if (gamepad1.x && !done) {
            done = robot.goTo(0,0,0.25, DriveTrain.Direction.N);
        }
        else if(!gamepad1.x){
            done = false;
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            z = gamepad1.right_stick_x;

            encoder = lifty.getCurrentPosition();


//        if((gamepad2.left_stick_y > 0 && encoder <= encoderMax) || (gamepad2.left_stick_y < 0 && encoder >= encoderMin))
//        {
//            lifty.setPower(-gamepad2.left_stick_y);
//        } else {
//            lifty.setPower(0.0);
//        }

            if(gamepad2.dpad_up && encoder >= encoderMax)
            {
                lifty.setPower(-.5);
            } else if (gamepad2.dpad_down && encoder <= encoderMin)
            {
                lifty.setPower(.5);
            } else {
                lifty.setPower(0);
            }

            if(gamepad2.a && !pressed1)
            {
                bigGrab = !bigGrab;
                pressed1 = true;
            } else if (!gamepad2.a && pressed1)
            {
                pressed1 = false;
            }

            if(gamepad2.x && !pressed2)
            {
                lilGrab = !lilGrab;
                pressed2 = true;
            } else if (!gamepad2.x && pressed2)
            {
                pressed2 = false;
            }

            if(gamepad1.a && !pressed3)
            {
                slowMode = !slowMode;
                pressed3 = true;
            } else if (!gamepad1.a && pressed3)
            {
                pressed3 = false;
            }
            if(lilGrab)
            {
                grabbyLeft.setPosition(0.0);
                grabbyRight.setPosition(1.0);
            }
            else
            {
                grabbyLeft.setPosition(0.45);
                grabbyRight.setPosition(0.45);
            }

            if(bigGrab)
            {
                bigGrabby.setPosition(0.05);
            }
            else
            {
                bigGrabby.setPosition(0.80);
            }

            if(slowMode)
            {
                robot.driveTrain.setMotorPower(x/2, y/2, z/2);
            } else {
                robot.driveTrain.setMotorPower(x,y,z);
            }
        }

//        if(gamepad1.dpad_left){
//            bigPos -= 0.01;
//        } else if(gamepad1.dpad_right)
//        {
//            bigPos += 0.01;
//        } bigPos = Range.clip(bigPos, 0.0, 1.0);
//
//        if(gamepad1.a){
//            leftPos -= 0.01;
//        } else if(gamepad1.b){
//            leftPos += 0.01;
//        } leftPos = Range.clip(leftPos, 0.0, 1.0);
//
//        if(gamepad1.x){
//            rightPos -= 0.01;
//        } else if(gamepad1.y){
//            rightPos += 0.01;
//        } rightPos = Range.clip(rightPos, 0.0, 1.0);



        telemetry.addLine()
                .addData("X: ", robot.odometry.getXPos())
                .addData(" Y: ", robot.odometry.getYPos())
                .addData(" Theta: ", robot.odometry.getOrientation());
        telemetry.addData("Lift encoder: ", lifty.getCurrentPosition());


        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
