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
    public enum DENEST_ACTIVITY {
        IDLE,
        LIFTING_TO_ROTATE,
        ROTATING,
        LOWERING_TO_POSITION
    }

    public enum ACQUIRE_ACTIVITY {
        IDLE,
        LOWER,
        ACQUIRE,
        RAISE
    }

    // Strings for config
    public final static String HOOK_1 = "h1";
    public final static String HOOK_2 = "h2";
    public final static String ACQU1 = "a1";
    public final static String ACQU2 = "a2";
    public final static String ARM = "arm";
    public final static String FINE_MOVEMENT = "adjust";

    // Foundation objects
    public Servo hook1; // Hook left
    public Servo hook2; // Hook right

    // Aquisition objects
    public DcMotor acq1; // Turret
    public DcMotor acq2;  // Extension
    public Servo fineMovemnt; // Minor adjustments (rarely used)
    public Servo arm; // Capture and Release
    public DENEST_ACTIVITY denestState = DENEST_ACTIVITY.IDLE;
    public ACQUIRE_ACTIVITY acquireState = ACQUIRE_ACTIVITY.IDLE;

    // Variables
    private final int DENEST = 1350; // Subject to change
    private final int LIFT_TO_DENEST = 2800; // Subject to change
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

    public void denest(){
        acq2.setTargetPosition(LIFT_TO_DENEST);
        acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        acq2.setPower(1);

        acq1.setTargetPosition(DENEST);
        acq1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        acq1.setPower(1);
    }

    public boolean startDenesting() {
        // This is more of a design pattern to show if the start happened or not.
        boolean startingDenest;

        // We don't want to start denesting if it is in the middle of denesting.
        if(denestState == DENEST_ACTIVITY.IDLE) {
            startingDenest = true;
            acq2.setTargetPosition(LIFT_TO_DENEST);
            acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            acq2.setPower(1);
            fineMovemnt.setPosition(0);

            // This activates the perform Denesting function to start executing.
            denestState = DENEST_ACTIVITY.LIFTING_TO_ROTATE;
        }
        else{
            startingDenest = false;
        }

        return startingDenest;
    }
    public void performDenesting() {
        switch(denestState) {
            case LIFTING_TO_ROTATE:
                // When acq2 is not busy, that means it finished lifting.
                if(acq2.getCurrentPosition() > 2500) {
                    // Time to start rotating.
                    acq1.setTargetPosition(DENEST);
                    acq1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    acq1.setPower(1);
                    // Set our state to the next step.  Next time through the OpMode loop
                    // it will get to the next step.
                    denestState = DENEST_ACTIVITY.ROTATING;
                }
                break;
            case ROTATING:
                // When acq1 is not busy, that means it finished rotating.
                if(acq1.getCurrentPosition() > 1250) {
                    acq2.setTargetPosition(0);
                    acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    acq2.setPower(0.5);
                    fineMovemnt.setPosition(1);
                    // Set our state to the next step.  Next time through the OpMode loop
                    // it will get to the next step.
                    denestState = DENEST_ACTIVITY.LOWERING_TO_POSITION;
                }
                break;
            case LOWERING_TO_POSITION:
                if(!acq1.isBusy()) {
                    // We are done, go back to IDLE.
                    denestState = DENEST_ACTIVITY.IDLE;
                }
                break;
            case IDLE:
                // Do nothing.
                break;
        }
    }
    public boolean startAcquiring(){
        // This is more of a design pattern to show if the start happened or not.
        boolean startingAcquire;

        // We don't want to start acquiring if it is in the middle of acquiring.
        if(acquireState == ACQUIRE_ACTIVITY.IDLE) {
            startingAcquire = true;
            fineMovemnt.setPosition(0.90);

            // This activates the perform starting to  function to start executing.
            acquireState = ACQUIRE_ACTIVITY.LOWER;
        }
        else{
            startingAcquire = false;
        }
        return startingAcquire;
    }
    public void performAcquire(){
        switch(acquireState) {
            case LOWER:
                // When finemovement reaches this position, that means it finished lifting.
                if(fineMovemnt.getPosition() == 0.90) {
                    // Time to start securing the block
                    arm.setPosition(0);
                    // Set our state to the next step.  Next time through the OpMode loop
                    // it will get to the next step.
                    acquireState = ACQUIRE_ACTIVITY.ACQUIRE;
                }
                break;
            case ACQUIRE:
                // When arm reaches this position, that means it finished rotating.
                if(arm.getPosition() == 0){
                    // Time to start going up
                    fineMovemnt.setPosition(0);
                    // Set our state to the next step.  Next time through the OpMode loop
                    // it will get to the next step.
                    acquireState = ACQUIRE_ACTIVITY.RAISE;
                }
                break;
            case RAISE:
                if(fineMovemnt.getPosition() == 0) {
                    // We are done, go back to IDLE.
                    acquireState = ACQUIRE_ACTIVITY.IDLE;
                }
                break;
            case IDLE:
                // Do nothing.
                break;
        }
    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Hardware
        acq1 = hwMap.get(DcMotor.class, ACQU1);
        acq2 = hwMap.get(DcMotor.class, ACQU2);
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

