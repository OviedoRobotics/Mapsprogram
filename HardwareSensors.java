package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public final static String ACQU1 = "a1";
    public final static String ACQU2 = "a2";
    public final static String ARM = "arm";
    public final static String FINE_MOVEMENT = "adjust";
    public final static String SENSOR_RANGE_1 = "range1";
    public final static String SENSOR_RANGE_2 = "range2";

       // Foundation objects
    public Servo hook1; // Hook left
    public Servo hook2; // Hook right

    // Aquisition objects
    public DcMotor acq1; // Turret
    public DcMotor acq2;  // Extension
    public Servo fineMovemnt; // Minor adjustments (rarely used)
    public Servo arm; // Capture and Release
    public DistanceSensor sensorRange; // Sense distance from stone
    public DistanceSensor sensorRange2; // Confirm distance from stone

    // Variables
    private final int DENEST = 1400; // Subject to change
    private final int LIFT_TO_DENEST = 850; // Subject to change
    private final int LEVEL_INCREMENT = 1000;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareSensors(){

    }

    public int extension(int level) {
        return level*LEVEL_INCREMENT;
    }


    public void withEncoders(){
        acq1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        acq2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoders() {
        acq1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        acq2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        acq1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        acq2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void denest() throws InterruptedException {
        acq2.setTargetPosition(LIFT_TO_DENEST);
        acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        acq2.setPower(1);
        sleep
        acq1.setTargetPosition(DENEST);
        acq1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        acq1.setPower(1);

        acq2.setTargetPosition(0);
        acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        acq2.setPower(0.5);

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
        fineMovemnt = hwMap.get(Servo.class, FINE_MOVEMENT);
        arm = hwMap.get(Servo.class, ARM);

        acq1.setDirection(DcMotor.Direction.REVERSE);
        acq2.setDirection(DcMotor.Direction.REVERSE);
        withEncoders();
        resetEncoders();
    }


}

