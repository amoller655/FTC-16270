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

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;

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

@Autonomous(name= "Park-Close-Right", group="Right")

public class Short_Park_Right extends OpMode
{
    // Declare OpMode members.
    private DriveTrain driveTrain;

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

    private int state;
    private boolean isFinished;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        grabbyLeft = hardwareMap.servo.get("leftGrabber");
        grabbyRight = hardwareMap.servo.get("rightGrabber");

        bigGrabby = hardwareMap.servo.get("mainGrabber");
        lifty = hardwareMap.dcMotor.get("lift");

        imu = hardwareMap.get(BNO055IMU.class, "imu");


        driveTrain = new DriveTrain(leftFront, rightFront, leftRear, rightRear, imu, telemetry);

        lifty.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifty.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        encoderMin = lifty.getCurrentPosition();
        encoderMax = encoderMin - 4700;

        grabbyLeft.setPosition(0.45);
        grabbyRight.setPosition(0.45);
        bigGrabby.setPosition(0.95);

        bigGrab = false;
        pressed1 = false;
        lilGrab = false;
        pressed2 = false;

        state = 1;
        isFinished = false;
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
        bigGrabby.setPosition(.2);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch(state){
            case 1:
                if(driveTrain.encoderDrive(DriveTrain.Direction.N, 6, 0.25))
                {
                    state = 2;
                }
                break;
            case 2:
                if(driveTrain.gyroTurn(DriveTrain.Direction.TURNLEFT, 0.25, 90))
                {
                    state = 3;
                }
                break;
            case 3:
                if(driveTrain.encoderDrive(DriveTrain.Direction.N, 6, 0.25))
                {
                    state = 4;
                }
                break;
            case 4:
                stop();
                telemetry.addLine("Done! ");
                break;
        }
        telemetry.addData("State: ", state);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
