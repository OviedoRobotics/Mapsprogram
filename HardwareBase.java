package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

public class HardwareBase
{
    /* Public OpMode members. */

    // Wheel objects
    public DcMotor left_drive = null; // left drive front
    public DcMotor right_drive = null; // left drive front
    public DcMotor right_drive1 = null; // right drive back
    public DcMotor left_drive1 = null; // right drive back

    // Foundation objects
    public Servo hook1 = null; // Hook left
    public Servo hook2 = null; // Hook right

    // Aquisition objects
    public DcMotor acq1 = null; // Torret
    public DcMotor acq2 = null; // Torret
    public Servo arm = null; // Capture and Release
    public DistanceSensor sensorRange;
    public DistanceSensor sensorRange2;
    protected BNO055IMU imu = null;
    private boolean inputShaping = true;
    protected boolean imuRead = false;
    protected double imuValue = 0.0;

    protected double frontLeftMotorPower = 0.0;
    protected double rearLeftMotorPower = 0.0;
    protected double frontRightMotorPower = 0.0;
    protected double rearRightMotorPower = 0.0;
    public final static double MIN_SPIN_RATE = 0.05;
    public final static double MIN_DRIVE_RATE = 0.05;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBase(){

    }

    public void setInputShaping(boolean inputShapingEnabled) {
        inputShaping = inputShapingEnabled;
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

    public void initIMU(){
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile = "BNO055IMUCalibration.json";
    parameters.loggingEnabled = true;
    parameters.loggingTag = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        left_drive  = hwMap.get(DcMotor.class, "l1");
        right_drive = hwMap.get(DcMotor.class, "r1");
        left_drive1 = hwMap.get(DcMotor.class, "l2");
        right_drive1 = hwMap.get(DcMotor.class, "r2");
        acq1 = hwMap.get(DcMotor.class, "a1");
        acq2 = hwMap.get(DcMotor.class, "a2");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        left_drive.setDirection(DcMotor.Direction.FORWARD);
        right_drive1.setDirection(DcMotor.Direction.FORWARD);
        left_drive1.setDirection(DcMotor.Direction.REVERSE);
        acq1.setDirection(DcMotor.Direction.FORWARD);
        acq2.setDirection(DcMotor.Direction.FORWARD);
        sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");
        sensorRange2 = hwMap.get(DistanceSensor.class, "range2");

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_drive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        acq1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        acq2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Define and initialize ALL installed servos.
        hook1 = hwMap.get(Servo.class, "h1");
        hook2 = hwMap.get(Servo.class, "h2");
        arm = hwMap.get(Servo.class, "arm");
    }

    public void setFrontLeftMotorPower(double power)
    {
        if(power != frontLeftMotorPower)
        {
            frontLeftMotorPower = power;
            left_drive.setPower(power);
        }
    }

    public void setRearLeftMotorPower(double power)
    {
        if(power != rearLeftMotorPower)
        {
            rearLeftMotorPower = power;
            left_drive1.setPower(power);
        }
    }

    public void setFrontRightMotorPower(double power)
    {
        if(power != frontRightMotorPower)
        {
            frontRightMotorPower = power;
            right_drive.setPower(power);
        }
    }

    public void setRearRightMotorPower(double power)
    {
        if(power != rearRightMotorPower)
        {
            rearRightMotorPower = power;
            right_drive1.setPower(power);
        }
    }

    public void setAllDrive(double power) {
        setFrontLeftMotorPower(power);
        setFrontRightMotorPower(power);
        setRearRightMotorPower(power);
        setRearLeftMotorPower(power);
    }

    public void drive(double xPower, double yPower, double spin, double angleOffset) {
        double gyroAngle = readIMU() + angleOffset;
        double leftFrontAngle = toRadians(45.0 + gyroAngle);
        double rightFrontAngle = toRadians(-45.0 + gyroAngle);
        double leftRearAngle = toRadians(135.0 + gyroAngle);
        double rightRearAngle = toRadians(-135.0 + gyroAngle);
        double joystickMagnitude = sqrt(xPower*xPower + yPower*yPower);
        double joystickAngle = atan2(yPower, xPower);
        double newPower = driverInputShaping(joystickMagnitude);
        double newSpin = driverInputSpinShaping(spin);
        double newXPower = newPower * cos(joystickAngle);
        double newYPower = newPower * sin(joystickAngle);

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
}

