package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;
/*
 * Created by Jazmyn James on 11/27/2019
 */
@TeleOp
public class AcquisitionSystem extends OpMode {

    /* Declare OpMode members. */
    HardwareSensors onbot = new HardwareSensors();
    private ElapsedTime runtime = new ElapsedTime();
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double OPEN    =  0.8;     // Maximum rotational position
    static final double CLOSE   =  0.6;     // Minimum rotational position
    static final double UNHOOK    =  0.0, UNHOOK1    =  0.5;     // Maximum rotational position for hooks
    static final double HOOK   =  0.0, HOOK1 = 0.65;     // Minimum rotational position
    static final double UP = 0, DOWN = 1.0;
    static  int countLevel = 0;
    boolean x2Pressed = false;
    boolean x2Held = false;
    boolean nest = true;
    boolean y2Pressed = false;
    boolean y2Held = false;
    double  position2; // Start at halfway position
    double position3;
    double  position = (OPEN + CLOSE) / 2; // Start at halfway position


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        onbot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        onbot.denest();

    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop(){

        ///////////////////////////////////////////////////////////////////
        ////                                                           ////
        ////              Acquisition System program                   ////
        ////                                                           ////
        ///////////////////////////////////////////////////////////////////


        x2Pressed = gamepad2.x;
        // If x2 was pressed, but not held
        if(x2Pressed && !x2Held && nest) {
            x2Held = true;
            // So doing stuff with acq2 and acq1 in here might conflict with the activity in onbot.
            // So for instance it might make the isBusy no longer work.  So might have to integrate
            // stuff like this into the activities below in onbot.  For now checking the activity state
            // to make sure it doesn't interfere.
            if(onbot.denestState == HardwareSensors.DENEST_ACTIVITY.IDLE) {
                countLevel = 1;
                int up = onbot.extension(countLevel);

                onbot.acq2.setTargetPosition(up);
                onbot.acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                onbot.acq2.setPower(1);
            }
        } else if(!x2Pressed) {
            // This happens if the button is not being pressed at all, which resets that
            // we are holding it so the next time it is pressed it will trigger the action
            // again.
            x2Held = false;
        }

        y2Pressed = gamepad2.y;

        if(y2Pressed && !y2Held) {
            int newHeight = onbot.acq2.getCurrentPosition();
            if( newHeight > 300 ) {

                onbot.acq2.setTargetPosition(0);
                onbot.acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                onbot.acq2.setPower(0.75);


            }
            else if(newHeight <= 300) {
                onbot.acq2.setTargetPosition(0);
                onbot.acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                onbot.acq2.setPower(0.75);
            }
        } else if(!y2Pressed) {
            y2Held = false;

        }

        double position1 = 0;

        //down
        if(gamepad2.left_stick_y > 0.02  ) {
            // Keep stepping up until we hit the max value.
            position1 += INCREMENT ;
            if (position1 >= UP ) {
                position1 = UP;
            }
            onbot.fineMovemnt.setPosition(position1);
                    }
        //up
        else if(gamepad2.left_stick_y < 0.02 ){
            // Keep stepping down until we hit the min value.
            position1 -= INCREMENT ;
            if (position1 <= DOWN ) {
                position1 = DOWN;

            }
            onbot.fineMovemnt.setPosition(position1);
        }



        telemetry.addData("Current Position of Extension", + onbot.acq2.getCurrentPosition() );
        telemetry.addData("Current Position of Turn", + onbot.acq1.getCurrentPosition() );
        telemetry.update();

        // Open the acquisition system
        if(gamepad2.left_bumper  ) {
            position = OPEN;
        }
        // secure the block
        else if(gamepad2.right_bumper ){
            position = CLOSE;
        }

        onbot.arm.setPosition(position);

        // So doing stuff with acq2 and acq1 in here might conflict with the activity in onbot.
        // So for instance it might make the isBusy no longer work.  So might have to integrate
        // stuff like this into the activities below in onbot.  For now checking the activity state
        // to make sure it doesn't interfere.
        if(onbot.denestState == HardwareSensors.DENEST_ACTIVITY.IDLE) {
            if (position == OPEN && onbot.acq2.getCurrentPosition() > 0) {
                onbot.acq2.setTargetPosition(0);
                onbot.acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                onbot.acq2.setPower(0.5);
            }
        }

        // ///////////////////////////////////////////////////////////////////
        // ////                                                           ////
        // ////                Foundation Hooks program                   ////
        // ////                                                           ////
        // ///////////////////////////////////////////////////////////////////

        // Open the hook system
        if (gamepad2.b) {
            position2 = UNHOOK;
            position3 = UNHOOK1;
        }
        // secure the foundation
        else if(gamepad2.a ){
            position2 = HOOK1;
            position3 = HOOK;
        }

        onbot.hook1.setPosition(position2);
        onbot.hook2.setPosition(position3);

        // We have a block of all of our "perform" functions at the end of our loop.
        // If the activity isn't doing anything, it should just return.
        //onbot.performDenesting();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}

