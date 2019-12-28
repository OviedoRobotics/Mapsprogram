package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
        //onbot.denest();

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
        //onbot.denest();

        x2Pressed = gamepad2.x;
        // If x2 was pressed, but not held
        if(x2Pressed && !x2Held && nest) {
            x2Held = true;
            countLevel++;
            int up = onbot.extension(countLevel);

            onbot.acq2.setTargetPosition(up);
            onbot.acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            onbot.acq2.setPower(1);

        } else if(!x2Pressed) {
            // This happens if the button is not being pressed at all, which resets that
            // we are holding it so the next time it is pressed it will trigger the action
            // again.
            x2Held = false;
        }

        double upPower = -gamepad1.left_stick_y;
        
        onbot.acq2.setPower(upPower);



        telemetry.addData("Current Position of Extension", + onbot.acq2.getCurrentPosition() );
        telemetry.addData("Current Position of Turn", + onbot.acq1.getCurrentPosition() );
        telemetry.update();

        // Open the acquistion system
        if(gamepad2.left_bumper  ) {
            position = OPEN;
        }
        // secure the block
        else if(gamepad2.right_bumper ){
            position = CLOSE;
        }

        onbot.arm.setPosition(position);

        if( position == OPEN && onbot.acq2.getCurrentPosition() > )

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

