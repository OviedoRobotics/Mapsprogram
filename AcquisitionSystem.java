package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp
public class AcquisitionSystem extends OpMode{

    /* Declare OpMode members. */
    HardwareSensors onbot = new HardwareSensors();
    private ElapsedTime runtime = new ElapsedTime();
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    DENEST = 3200;
    static final int    HEIGHT_INCREMENT = 3100;
    static final int    SPIN = 200;
    static final double LIFT_SPEED = 1.0;
    static final double OPEN    =  0.8;     // Maximum rotational position
    static final double CLOSE   =  0.6;     // Minimum rotational position
    static final double UNHOOK    =  1.0;     // Maximum rotational position
    static final double HOOK   =  0.0;     // Minimum rotational position
    double  position2 = (UNHOOK - HOOK) / 2; // Start at halfway position
    double position3 = (UNHOOK - HOOK) / 2;
    double  position = (OPEN + CLOSE) / 2; // Start at halfway position
    boolean x2Pressed = false;
    boolean x2Held = false;
    boolean nest = true;
    boolean y2Pressed = false;
    boolean y2Held = false;

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
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        ///////////////////////////////////////////////////////////////////
        ////                                                           ////
        ////              Acquisition System program                   ////
        ////                                                           ////
        ///////////////////////////////////////////////////////////////////


        x2Pressed = gamepad2.x;
        // If x2 was pressed, but not held
        if(x2Pressed && !x2Held && nest) {
            x2Held = true;
            nest = false;

            onbot.acq2.setTargetPosition(DENEST);
            onbot.acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            onbot.acq2.setPower(LIFT_SPEED);

            onbot.acq1.setTargetPosition(SPIN);
            onbot.acq1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            onbot.acq1.setPower(LIFT_SPEED);


        } else if(!x2Pressed) {
            // This happens if the button is not being pressed at all, which resets that
            // we are holding it so the next time it is pressed it will trigger the action
            // again.
            x2Held = false;
        }


        y2Pressed = gamepad2.y;

        if(y2Pressed && !y2Held && !nest ) {
            y2Held = true;
            nest = false;

            onbot.acq2.setTargetPosition(0);
            onbot.acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            onbot.acq2.setPower(LIFT_SPEED);

            onbot.acq1.setTargetPosition(0);
            onbot.acq1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            onbot.acq1.setPower(LIFT_SPEED);

        } else if(!y2Pressed) {
            y2Held = false;
        }

        if( !nest ) {
         while( onbot.acq2.getCurrentPosition() > 0) {
             onbot.acq2.setPower(gamepad2.right_stick_y);
         }
            onbot.acq2.setPower(0);
        }

        telemetry.addData("Current Position of Rotation", + onbot.acq2.getCurrentPosition());
        boolean turn = false;

        onbot.acq1.setPower(0.25);


        telemetry.addData("Current Position of Extension", + onbot.acq1.getCurrentPosition() );
        telemetry.update();

        // Open the acquistion system
        if(gamepad2.left_bumper  ) {
            // Keep stepping up until we hit the max value.
            position += INCREMENT ;
            if (position >= OPEN ) {
                position = OPEN;
            }
        }
        // secure the block
        else if(gamepad2.right_bumper ){
            // Keep stepping down until we hit the min value.
            position -= INCREMENT ;
            if (position <= CLOSE ) {
                position = CLOSE;

            }
        }

        onbot.arm.setPosition(position);

        // ///////////////////////////////////////////////////////////////////
        // ////                                                           ////
        // ////                Foundation Hooks program                   ////
        // ////                                                           ////
        // ///////////////////////////////////////////////////////////////////

        // Open the acquistion system
        if (gamepad2.b) {
            position2 = UNHOOK;
            position3 = HOOK;
        }
        // secure the block
        else if(gamepad2.a ){
            position2 = 0.5;
            position3 = 0.5;
        }

        onbot.hook1.setPosition(position2);
        onbot.hook2.setPosition(position3);
    }
    public double scale(double n)
    {
        return n;
    }
    public void limitforExtension( double power )
    {

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}

