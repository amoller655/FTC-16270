package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    public DriveTrain driveTrain;

    public Odometry odometry;

    private DcMotor leftFront, rightFront, leftRear, rightRear;

    private DcMotor leftEncoder, rightEncoder, normalEncoder;

    private String leftFrontName, rightFrontName, leftRearName, rightRearName;
//    CHECK PORT NAMES
    private String leftEncoderName = leftFrontName, rightEncoderName = rightFrontName, normalEncoderName = rightRearName;

//    OBSOLETE: phase out
    private HardwareMap hardwareMap;
    private BNO055IMU imu;
    private String imuName;

//    CONSTANTS FOR goto()
    private final double driveSlow1 = 6.0;
    private final double driveSlow2 = 2.5;
    private final double turnSlow1 = 22.5;
    private final double turnSlow2 = 5.0;
    private final double

    public Robot(HardwareMap hardwareMap, String leftFrontName, String rightFrontName, String leftRearName, String rightRearName, String imuName, Telemetry telemetry) {
        this.leftFrontName = leftFrontName;
        this.rightFrontName = rightFrontName;
        this.leftRearName = leftRearName;
        this.rightRearName = rightRearName;
        this.imuName = imuName;
        this.hardwareMap = hardwareMap;

        initHardwareMap(this.leftFrontName, this.rightFrontName, this.leftRearName, this.rightRearName, this.leftEncoderName, this.rightEncoderName, this.normalEncoderName, this.imuName);

        driveTrain = new DriveTrain(leftFront, rightFront, leftRear, rightRear, imu, telemetry);

        odometry = new Odometry(leftEncoder, rightEncoder, normalEncoder, 0.0, 50);
    }

    private void initHardwareMap(String leftFrontName, String rightFrontName, String leftRearName, String rightRearName, String leftEncoderName, String rightEncoderName, String normalEncoderName, String imuName)
    {
        leftFront = hardwareMap.dcMotor.get(leftFrontName);
        rightFront = hardwareMap.dcMotor.get(rightFrontName);
        leftRear = hardwareMap.dcMotor.get(leftRearName);
        rightRear = hardwareMap.dcMotor.get(rightRearName);

        leftEncoder = hardwareMap.dcMotor.get(leftEncoderName);
        rightEncoder = hardwareMap.dcMotor.get(rightEncoderName);
        normalEncoder = hardwareMap.dcMotor.get(normalEncoderName);

        imu = hardwareMap.get(BNO055IMU.class, imuName);

    }

    public boolean goTo(double xF, double yF, double power, DriveTrain.Direction direction)
    {
        double x = odometry.getXPos();
        double y = odometry.getYPos();
        double theta = odometry.getOrientation();

        double dx = xF - x;
        double dy = yF - y;

        double pathAngle = Math.toDegrees(calculatePathAngle(dx, dy));
        double facingOffset = 0.0;
        switch(direction)
        {
            case N:
                facingOffset = 0;
                break;
            case NE:
                facingOffset = 315;
                break;
            case E:
                facingOffset = 270;
                break;
            case SE:
                facingOffset = 225;
                break;
            case S:
                facingOffset = 180;
                break;
            case SW:
                facingOffset = 135;
                break;
            case W:
                facingOffset = 90;
                break;
            case NW:
                facingOffset = 45;
                break;
            default:
                facingOffset = 0;
                break;
        }
        DriveTrain.Direction turn = DriveTrain.Direction.TURNRIGHT;
        double turnPower = power;
        double remainingTurn = 0;
        double goalAngle = wrapAngle(pathAngle - facingOffset);
        if(theta != goalAngle){
            if(theta <= goalAngle){
                if((goalAngle - theta) <= 180){
                    turn = DriveTrain.Direction.TURNLEFT;
                    remainingTurn = goalAngle - theta;
                }
                else {
                    turn = DriveTrain.Direction.TURNRIGHT;
                    remainingTurn = 360 - (goalAngle - theta);
                }

            } else {
                if((theta - goalAngle) <= 180){
                    turn = DriveTrain.Direction.TURNRIGHT;
                    remainingTurn = theta - goalAngle;
                }
                else {
                    turn = DriveTrain.Direction.TURNLEFT;
                    remainingTurn = 360 - (theta - goalAngle);
                }
            }
            if(remainingTurn <= 5)
                turnPower = power * 1 / 3;
            if(remainingTurn <= 22.5)
                turnPower = power * 2 / 3;
            driveTrain.drive(turn, turnPower);
            return false;
        }
        else if((x != xF) && (y != yF)) {
            double dist =  dist(dx, dy);
            double drivePower = power;
            if(dist  <= 2.5)
                drivePower = power/3;
            else if(dist <= 6.0)
                drivePower = power * 2/3;
            driveTrain.drive(direction, drivePower);
            return false;
        }
        return true;
    }

    private double calculatePathAngle(double dx, double dy)
    {
        if(dy == 0)
        {
            if(dx > 0) return 0;
            else if(dx < 0) return 180;
        }
        else if(dx == 0)
        {
            if(dy > 0) return 90;
            else if(dy < 0) return 270;
        }
        else if (dx > 0) return (360 + Math.atan2(dx, dy)) % 360;
        else if (dx < 0) return (180 + Math.atan2(dx, dy));
        return 0;
    }

    private double wrapAngle(double angle) {
        if(angle < 0) return angle + 360;
        else if(angle >= 360) return angle - 360;
        else return angle;
    }

    private double dist(double dx, double dy)
    {
        return (Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2)));
    }
}
