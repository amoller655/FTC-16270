package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    public DriveTrain driveTrain;

    public Odometry odometry;

    public Thread posThread;


    private DcMotor leftFront, rightFront, leftRear, rightRear;

    private DcMotor leftEncoder, rightEncoder, normalEncoder;

    private String leftFrontName, rightFrontName, leftRearName, rightRearName;
//    CHECK PORT NAMES
    private String leftEncoderName = leftFrontName, rightEncoderName = rightFrontName, normalEncoderName = rightRearName;

//    OBSOLETE: phase out
    private HardwareMap hardwareMap;
    private BNO055IMU imu;
    private String imuName;

    public Telemetry telemetry;
//    CONSTANTS FOR goto()
    private final double driveSlow1 = 6.0;
    private final double driveSlow2 = 2.5;
    private final double turnSlow1 = 45;
    private final double turnSlow2 = 22.5;

    public Robot(HardwareMap hardwareMap, String leftFrontName, String rightFrontName, String leftRearName, String rightRearName, String imuName, Telemetry telemetry) {
        this.leftFrontName = leftFrontName;
        this.rightFrontName = rightFrontName;
        this.leftRearName = leftRearName;
        this.rightRearName = rightRearName;
        this.imuName = imuName;
        this.hardwareMap = hardwareMap;

        this.leftEncoderName = leftFrontName;
        this.rightEncoderName = rightFrontName;
        this.normalEncoderName = rightRearName;

        this.telemetry = telemetry;

        initHardwareMap(this.leftFrontName, this.rightFrontName, this.leftRearName, this.rightRearName, this.leftEncoderName, this.rightEncoderName, this.normalEncoderName, this.imuName);

        driveTrain = new DriveTrain(leftFront, rightFront, leftRear, rightRear, imu, telemetry);

        odometry = new Odometry(leftEncoder, rightEncoder, normalEncoder, 288.8665556, 50);
//        odometry.reverseRight();
        odometry.reverseNormal();

        posThread = new Thread(odometry);
        posThread.start();
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

        telemetry.addLine()
                .addData("dX: ", dx)
                .addData("dY", dy);

        double pathAngle = calculatePathAngle(dx, dy);
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
        DriveTrain.Direction turn;
        double turnPower = power;
        double remainingTurn = 0;
        double goalAngle = wrapAngle(pathAngle - facingOffset);
        telemetry.addLine().addData("PATH: ", pathAngle);
        telemetry.addLine().addData("GOAL: ", goalAngle);
        if(theta > wrapAngle(goalAngle + 10) || theta < wrapAngle(goalAngle - 10)){
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
            telemetry.addLine().addData("Remaining Turn: ", remainingTurn);
            if(remainingTurn <= turnSlow2)
                driveTrain.drive(turn, (power/3));
            if(remainingTurn <= turnSlow1)
                driveTrain.drive(turn, (power*2/3));
            else
                driveTrain.drive(turn, turnPower);
            return false;
        }

        else if((x > xF + .5 || x < xF - .5) || (y > yF + .5 || y < yF - .5)) {
            double dist =  dist(dx, dy);
            double drivePower = power;
            if(dist  <= driveSlow2)
                drivePower = power/3;
            else if(dist <= driveSlow1)
                drivePower = power * 2/3;
            driveTrain.drive(direction, drivePower);
            if(theta > goalAngle)
            {
                double correction = (theta - goalAngle) / 10;
                driveTrain.adjust(DriveTrain.Direction.TURNLEFT, correction);
            } else if (theta < goalAngle)
            {
                double correction = (goalAngle - theta) / 10;
                driveTrain.adjust(DriveTrain.Direction.TURNRIGHT, correction);
            }
            return false;
        } 
        else {
            driveTrain.stop();
            return true;
        }

    }

    private double calculatePathAngle(double dx, double dy)
    {
        double angle = Math.toDegrees(Math.atan2(dy, dx));
        if(angle < 0)
            angle += 360;
        return angle;
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
