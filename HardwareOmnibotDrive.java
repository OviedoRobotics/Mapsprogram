package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

/**
 *Created by STEM Punk and MAPS
 */
public class HardwareOmnibotDrive
{
    /* Public OpMode members. */
    public final static double MIN_SPIN_RATE = 0.05;
    public final static double MIN_DRIVE_RATE = 0.05;

    // Robot Controller Config Strings
    public final static String IMU = "imu";
    public final static String FRONT_LEFT_MOTOR = "l1";
    public final static String FRONT_RIGHT_MOTOR = "r1";
    public final static String REAR_LEFT_MOTOR = "l2";
    public final static String REAR_RIGHT_MOTOR = "r2";
    public final static String SENSOR_RANGE_1 = "range1";
    public final static String SENSOR_RANGE_2 = "range2";

    // Hardware objects
    protected DcMotor frontLeft = null;
    protected DcMotor frontRight = null;
    protected DcMotor rearLeft = null;
    protected DcMotor rearRight = null;
    protected BNO055IMU imu = null;
    public DistanceSensor sensorRange; // Sense distance from stone
    public DistanceSensor sensorRange2; // Confirm distance from stone

    // Tracking variables
    private static final int encoderClicksPerSecond = 2800;
    protected double frontLeftMotorPower = 0.0;
    protected double rearLeftMotorPower = 0.0;
    protected double frontRightMotorPower = 0.0;
    protected double rearRightMotorPower = 0.0;
    private boolean inputShaping = true;
    protected boolean imuRead = false;
    protected double imuValue = 0.0;

    /* local OpMode members. */
    protected HardwareMap hwMap  =  null;

    /* Constructor */
    public HardwareOmnibotDrive(){
    }

    public void setInputShaping(boolean inputShapingEnabled) {
        inputShaping = inputShapingEnabled;
    }

    public void initIMU()
    {
        // Init IMU code
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, IMU);
        imu.initialize(parameters);
    }

    public void resetReads() {
        imuRead = false;
    }

    public double readIMU()
    {
        if(!imuRead) {
            // Read IMU Code
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            imuValue = (double)angles.firstAngle;
            imuRead = true;
        }

        return imuValue;
    }

    public double readBackLeftTo() {
        return sensorRange.getDistance(DistanceUnit.CM);
    }

    public double readBackRightTo(){
        return sensorRange2.getDistance(DistanceUnit.CM);
    }

    public void setFrontLeftMotorPower(double power)
    {
        if(power != frontLeftMotorPower)
        {
            frontLeftMotorPower = power;
            frontLeft.setPower(power);
        }
    }

    public void setRearLeftMotorPower(double power)
    {
        if(power != rearLeftMotorPower)
        {
            rearLeftMotorPower = power;
            rearLeft.setPower(power);
        }
    }

    public void setFrontRightMotorPower(double power)
    {
        if(power != frontRightMotorPower)
        {
            frontRightMotorPower = power;
            frontRight.setPower(power);
        }
    }

    public void setRearRightMotorPower(double power)
    {
        if(power != rearRightMotorPower)
        {
            rearRightMotorPower = power;
            rearRight.setPower(power);
        }
    }

    public void setAllDrive(double power) {
        setFrontLeftMotorPower(power);
        setFrontRightMotorPower(power);
        setRearRightMotorPower(power);
        setRearLeftMotorPower(power);
    }

    public void setAllDriveZero()
    {
        setAllDrive(0.0);
    }

    /**
     *
     * @param xPower - -1.0 to 1.0 power in the X axis
     * @param yPower - -1.0 to 1.0 power in the Y axis
     * @param spin - -1.0 to 1.0 power to rotate the robot
     * @param angleOffset - The offset from the gyro to run at, such as drive compensation
     */
    public void drive(double xPower, double yPower, double spin, double angleOffset) {
        double gyroAngle = readIMU() + angleOffset;
        double leftFrontAngle = toRadians(135.0 + gyroAngle); // previous value 45
        double rightFrontAngle = toRadians(45.0 + gyroAngle); // previous value -45
        double leftRearAngle = toRadians(225.0 + gyroAngle); // previous value 135
        double rightRearAngle = toRadians(-45.0 + gyroAngle); // previous value -135
        double joystickMagnitude = sqrt(xPower*xPower + yPower*yPower);
        double joystickAngle = atan2(yPower, xPower);
        double newPower = driverInputShaping(joystickMagnitude);
        double newSpin = driverInputSpinShaping(spin);
        double newXPower = newPower * cos(joystickAngle) ;
        double newYPower = newPower * sin(joystickAngle) ;

        double LFpower = newXPower * cos(leftFrontAngle) + newYPower * sin(leftFrontAngle) + newSpin;
        double LRpower = newXPower * cos(leftRearAngle) + newYPower * sin(leftRearAngle) + newSpin;
        double RFpower = newXPower * cos(rightFrontAngle) + newYPower * sin(rightFrontAngle) + newSpin;
        double RRpower = newXPower * cos(rightRearAngle) + newYPower * sin(rightRearAngle) + newSpin;

        double maxPower = max(1.0, max(max(abs(LFpower), abs(LRpower)),
                max(abs(RFpower), abs(RRpower))));

        if(maxPower > 1.0) {
            LFpower /= maxPower;
            RFpower /= maxPower;
            RFpower /= maxPower;
            RRpower /= maxPower;
        }

        setFrontLeftMotorPower(LFpower);
        setFrontRightMotorPower(RFpower);
        setRearRightMotorPower(RRpower);
        setRearLeftMotorPower(LRpower);
    }

    protected double driverInputShaping( double valueIn) {
        double valueOut = 0.0;

        if(Math.abs(valueIn) < MIN_DRIVE_RATE) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                if (valueIn > 0) {
                    valueOut = MIN_DRIVE_RATE + (1.0 - MIN_DRIVE_RATE) * valueIn;
                } else {
                    valueOut = -MIN_DRIVE_RATE + (1.0 - MIN_DRIVE_RATE) * valueIn;
                }
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }

    protected double driverInputSpinShaping( double valueIn) {
        double valueOut = 0.0;

        if(Math.abs(valueIn) < MIN_SPIN_RATE) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                if (valueIn > 0) {
                    valueOut = (1.0 + MIN_SPIN_RATE) * valueIn - MIN_SPIN_RATE;
                } else {
                    valueOut = (1.0 + MIN_SPIN_RATE) * valueIn + MIN_SPIN_RATE;
                }
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }

    public void disableDriveEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetDriveEncoders()
    {
        int sleepTime = 0;
        int encoderCount = frontLeft.getCurrentPosition();

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while((encoderCount != 0) && (sleepTime < 1000)) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) { break; }
            sleepTime += 10;
            encoderCount = frontLeft.getCurrentPosition();
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set the stop mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeft = hwMap.dcMotor.get(FRONT_LEFT_MOTOR);
        frontRight  = hwMap.dcMotor.get(FRONT_RIGHT_MOTOR);
        rearLeft = hwMap.dcMotor.get(REAR_LEFT_MOTOR);
        rearRight = hwMap.dcMotor.get(REAR_RIGHT_MOTOR);
        sensorRange = hwMap.get(DistanceSensor.class, SENSOR_RANGE_1);
        sensorRange2 = hwMap.get(DistanceSensor.class, SENSOR_RANGE_2);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        setAllDriveZero();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //resetDriveEncoders(); //Had to exclude for drive to run properly

        initIMU();
    }
    public boolean gotoRearTarget(double driveSpeed, double spinSpeed){
        //These are the values we see when it is against the platform
        double backLeftTargetDistance = 2.0;
        double backRightTargetDistance = 4.5;

        double leftToDistance = readBackLeftTo();
        double rightToDistance = readBackRightTo();

        double drivePower = 0.0;
        double spinPower = 0.0;

        double leftError = backLeftTargetDistance- leftToDistance;
        double rightError = backRightTargetDistance - rightToDistance;
        boolean touching = false;

        //Slow down for 10cm
        double slowDownStart = 20.0;
        double slowDownRange = 10.0;
        if((leftError < 0) || (rightError < 0)){
            //Have to drive backwards towards the foundation
            drivePower = driveSpeed;
            //if one of them is within range, drive speed should be minimum, all
            // rotation
            if(!((rightError < 0) && (leftError < 0))){
                drivePower = MIN_DRIVE_RATE;
                spinPower = Math.copySign(spinPower, MIN_SPIN_RATE);

            } else {
                // We want the one closer to the foundation minError should be negative.
                double minError = Math.max(rightError, leftError);

                // Need to set drive power based on range.
                // Go at slowest speed.
                double scaleFactor = 0.95 * (slowDownRange - (slowDownStart + minError))/ slowDownRange + MIN_DRIVE_RATE;
                scaleFactor = Math.min(1.0, scaleFactor);
                drivePower = driveSpeed * scaleFactor;
            }
            // make sure we don't go below minimum spin power;
            if(spinPower < MIN_SPIN_RATE) {
                spinPower = Math.copySign(spinPower, MIN_SPIN_RATE);
            }
            // make sure we don't go below minimum drive power.
            if(drivePower < MIN_DRIVE_RATE){
                drivePower = MIN_DRIVE_RATE;
            }
            // Scale the power based on how far
            drive( 0, -drivePower, -spinPower, -readIMU());
        }else {
            drive(0, -MIN_DRIVE_RATE, 0, -readIMU());
            touching = true;
        }
        return touching;
    }
}
