
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class DriveTrain{

//    MOTORS
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

//    IMU
    private BNO055IMU imu;
    private BNO055IMU.Parameters parameters;
    private BNO055IMU.AccelerationIntegrator accel;

//    TELEMETRY
    private Telemetry telemetry;

//    CONSTRUCTOR
    public DriveTrain(DcMotor leftFront, DcMotor rightFront, DcMotor leftRear, DcMotor rightRear, BNO055IMU imu, Telemetry telemetry){
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;

        this.imu = imu;

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "Hub1Alone.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(DistanceUnit.INCH, 0, 0, 0, 0), new Velocity(DistanceUnit.INCH, 0, 0, 0, 0), 10);

        telemetry.addData("DriveTrain.java Startup ", "Initiating");
        telemetry.update();

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);

        telemetry.addData("DriveTrain.java Startup ", "Completed");
    }

    public double getXPos(){
        return imu.getPosition().x;
    }

    public double getYPos(){
        return imu.getPosition().y;
    }

    public double getZPos(){
        return imu.getPosition().z;
    }

    public double getXVel(){
        return imu.getVelocity().xVeloc;
    }

    public double getYVel(){
        return imu.getVelocity().yVeloc;
    }

    public double getZVel(){
        return imu.getVelocity().zVeloc;
    }

    public double getXAcc(){
        return imu.getAcceleration()
    }
    public void setMotorPower(double x, double y, double z){
        /*
        Guide to motor Powers:
        Left Front: y + x + z
        Right Front: y - x - z
        Left Rear: y + x - z
        Right Rear: y - x + z
         */
        leftFront.setPower(Range.clip((y + x + z), -1.0, 1.0));
        rightFront.setPower(Range.clip((y - x - z), -1.0, 1.0));
        leftRear.setPower(Range.clip((y + x - z), -1.0, 1.0));
        rightRear.setPower(Range.clip((y - x + z), -1.0, 1.0));
    }

    public void drive(Direction direction, double power){
        switch(direction){
            case N:
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftRear.setPower(power);
                rightRear.setPower(power);
                break;
            case S:
                leftFront.setPower(-power);
                rightFront.setPower(-power);
                leftRear.setPower(-power);
                rightRear.setPower(-power);
                break;
            case W:
                leftFront.setPower(-power);
                leftRear.setPower(power);
                rightFront.setPower(power);
                rightRear.setPower(-power);
                break;
            case E:
                leftFront.setPower(power);
                leftRear.setPower(-power);
                rightFront.setPower(-power);
                rightRear.setPower(power);
                break;
            case TURNLEFT:
                leftFront.setPower(-power);
                leftRear.setPower(-power);
                rightFront.setPower(power);
                rightRear.setPower(power);
                break;
            case TURNRIGHT:
                leftFront.setPower(power);
                leftRear.setPower(power);
                rightFront.setPower(-power);
                rightRear.setPower(-power);
                break;
            case NW:
                leftFront.setPower(power);
                leftRear.setPower(0.0);
                rightFront.setPower(0.0);
                rightRear.setPower(power);
                break;
            case NE:
                leftFront.setPower(0.0);
                leftRear.setPower(power);
                rightFront.setPower(power);
                rightRear.setPower(0.0);
                break;
            case SW:
                leftFront.setPower(0.0);
                leftRear.setPower(-power);
                rightFront.setPower(-power);
                rightRear.setPower(0.0);
                break;
            case SE:
                leftFront.setPower(-power);
                leftRear.setPower(0.0);
                rightFront.setPower(0.0);
                rightRear.setPower(-power);
                break;
            case ENE:
                leftFront.setPower(power);
                leftRear.setPower(-power/2);
                rightFront.setPower(-power/2);
                rightRear.setPower(power);
                break;
            case NNE:
                leftFront.setPower(power);
                rightFront.setPower(power/2);
                leftRear.setPower(power/2);
                rightRear.setPower(power);
                break;
            case NNW:
                leftFront.setPower(power/2);
                rightFront.setPower(power);
                leftRear.setPower(power);
                rightRear.setPower(power/2);
                break;
            case WNW:
                leftFront.setPower(-power/2);
                rightFront.setPower(power);
                leftRear.setPower(power);
                rightRear.setPower(-power);
                break;
            case WSW:
                leftFront.setPower(-power);
                rightFront.setPower(power/2);
                leftRear.setPower(power/2);
                rightRear.setPower(-power/2);
                break;
        }
    }

    public void stop(){
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
    }

    public enum Direction{
        N,
        S,
        E,
        W,
        TURNRIGHT,
        TURNLEFT,
        NE,
        SE,
        NW,
        SW,
        ENE,
        NNE,
        NNW,
        WNW,
        WSW,
        SSW,
        SSE,
        ESE
    }
}