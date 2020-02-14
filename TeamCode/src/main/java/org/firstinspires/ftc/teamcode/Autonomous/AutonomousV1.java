package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

@Autonomous(name = "Autonomouse Ver 1.0.0", group = "Main")
public class AutonomousV1 extends LinearOpMode {
    private Robot robot;
    Selector selector;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, "leftFront", "rightFront", "leftRear", "rightRear", "imu", telemetry);

        selector = new Selector(gamepad1, telemetry);
        while(!isStarted())
        {
            selector.selectionLoop();
            if(gamepad1.dpad_right)
            {
                telemetry.addLine("RIGHT");
                telemetry.update();

            }
            sleep(250);
        }
        waitForStart();
        boolean hasFinished = false;
        while(!hasFinished && opModeIsActive())
        {
            hasFinished = robot.goTo(0, 18, .25, DriveTrain.Direction.N);
        }
    }
}
