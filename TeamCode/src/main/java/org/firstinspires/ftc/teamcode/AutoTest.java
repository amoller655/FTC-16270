
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Test", group="Linear Opmode")

public class AutoTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    private BNO055IMU imu;

    DriveTrain driveTrain;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        driveTrain = new DriveTrain(leftFront, rightFront, leftRear, rightRear, imu, telemetry);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        while(runtime.seconds() < 1.0) driveTrain.drive(DriveTrain.Direction.N, 0.5);
        runtime.reset();
        while(runtime.seconds() < 0.5)driveTrain.stop();
        runtime.reset();
        while(runtime.seconds() < 1.0) driveTrain.drive(DriveTrain.Direction.S, 0.5);
        runtime.reset();
        while(runtime.seconds() < 0.5)driveTrain.stop();
        runtime.reset();
        while(runtime.seconds() < 1.0) driveTrain.drive(DriveTrain.Direction.E, 0.5);
        runtime.reset();
        while(runtime.seconds() < 0.5)driveTrain.stop();
        runtime.reset();
        while(runtime.seconds() < 1.0) driveTrain.drive(DriveTrain.Direction.W, 0.5);
        runtime.reset();
        while(runtime.seconds() < 0.5)driveTrain.stop();
        runtime.reset();
        while(runtime.seconds() < 1.0) driveTrain.drive(DriveTrain.Direction.TURNLEFT, 0.5);
        runtime.reset();
        while(runtime.seconds() < 0.5)driveTrain.stop();
        runtime.reset();
        while(runtime.seconds() < 1.0) driveTrain.drive(DriveTrain.Direction.TURNRIGHT, 0.5);
        runtime.reset();
        while(runtime.seconds() < 2.5)driveTrain.stop();
        runtime.reset();
        double x = 1.0;
        double y = 0.0;
        int quad = 1;
        while(quad == 1){
            driveTrain.setMotorPower(x, y, 0);
            x -= 0.1;
            y += 0.1;
            if(y == 1.0) quad++;
        }
        while(quad == 2){
            driveTrain.setMotorPower(x, y, 0);
            x -= 0.1;
            y -= 0.1;
            if(y == 0) quad++;
        }
        while(quad == 3){
            driveTrain.setMotorPower(x, y, 0);
            x += 0.1;
            y -= 0.1;
            if(y == -1.0) quad++;
        }
        while(quad == 4){
            driveTrain.setMotorPower(x, y, 0);
            x += 0.1;
            y += 0.1;
            if(y == 0) quad++;
        }
        driveTrain.stop();
    }
}
