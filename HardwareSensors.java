package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareSensors
{
    /* Public OpMode members. */

    // Strings for config
    public final static String HOOK_1 = "h1";
    public final static String HOOK_2 = "h2";
    public final static String REAR_LEFT_MOTOR = "l2";
    public final static String REAR_RIGHT_MOTOR = "r2";
    public final static String ACQU1 = "a1";
    public final static String ACQU2 = "a2";
    public final static String ARM = "arm";
    public final static String SENSOR_RANGE_1 = "range1";
    public final static String SENSOR_RANGE_2 = "range2";

       // Foundation objects
    public Servo hook1; // Hook left
    public Servo hook2; // Hook right

    // Aquisition objects
    public DcMotor acq1; // Turret
    public DcMotor acq2;  // Turret
    public Servo arm; // Capture and Release
    public DistanceSensor sensorRange; // Sense distance from stone
    public DistanceSensor sensorRange2; // Confirm distance from stone

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSensors(){

    }



    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Hardware
        acq1 = hwMap.get(DcMotor.class, ACQU1);
        acq2 = hwMap.get(DcMotor.class, ACQU2);
        sensorRange = hwMap.get(DistanceSensor.class, SENSOR_RANGE_1);
        sensorRange2 = hwMap.get(DistanceSensor.class, SENSOR_RANGE_2);
        hook1 = hwMap.get(Servo.class, HOOK_1);
        hook2 = hwMap.get(Servo.class, HOOK_2);
        arm = hwMap.get(Servo.class, ARM);

        acq1.setDirection(DcMotor.Direction.FORWARD);
        acq2.setDirection(DcMotor.Direction.FORWARD);
    }


}
