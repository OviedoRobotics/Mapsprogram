package org.firstinspires.ftc.teamcode.Mapsprogram;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.toDegrees;

/**
 * Created by 12090 STEM Punk
 */

@TeleOp(name="Omni: TeleOpDrive", group ="TeleOp")
public class OmniTeleOpDrive extends OpMode {
    public HardwareOmnibotDrive robot = new HardwareOmnibotDrive();
    private HardwareSensors onbot = new HardwareSensors();
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    HEIGHT_INCREMENT = 900;
    static final int    LOWER = 2800;
    static final int    SPIN_INCREMENT = 1385;
    static final double LIFT_SPEED = 1.0;
    static final double OPEN    =  1.0;     // Maximum rotational position
    static final double CLOSE   =  0.6;     // Minimum rotational position
    static final double UNHOOK    =  1.0;     // Maximum rotational position
    static final double HOOK   =  0.0;     // Minimum rotational position
    double  position2 = (UNHOOK - HOOK) / 2; // Start at halfway position
    double position3 = (UNHOOK - HOOK) / 2;
    double  position = (OPEN + CLOSE) / 2; // Start at halfway position
    boolean x2Pressed = false;
    boolean x2Held = false;
    boolean y2Pressed = false;
    boolean y2Held = false;
    boolean togglePressed = false;
    boolean toggleHeld = false;

    @Override
    public void init() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        robot.init(hardwareMap);
        onbot.init(hardwareMap);
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
    }


    private double driverAngle = 0.0;
    private final double MAX_SPEED = 1.0;
    private final double MAX_SPIN = 1.0;
    private double yPower;
    private double xPower;
    private double spin;
    private double gyroAngle;

    // Set these if you want a slow mode for precision work
    private double speedMultiplier = MAX_SPEED;
    private double spinMultiplier = MAX_SPIN;
    private double lowSpeed = 0.3;
    private double lowSpin = 0.3;
    private boolean bumpPressed = false;
    private boolean bumpHeld = false;

    @Override
    public void start()
    {
    }

    @Override
    public void loop() {
        // In order to use sensor values throughout the program, we use the call
        // to read the sensor.  But since this is a costly call to make we cache
        // the value and return it until we reset reads to the sensors again.
        robot.resetReads();

        //left joystick is for moving
        //right joystick is for rotation
        yPower = -gamepad1.left_stick_y;
        xPower = gamepad1.left_stick_x;
        spin = gamepad1.right_stick_x;
        gyroAngle = robot.readIMU();


        if (gamepad1.x) {
            // The driver presses X, then uses the left joystick to say what angle the robot
            // is aiming.  This will calculate the values as long as X is pressed, and will
            // not drive the robot using the left stick.  Once X is released, it will use the
            // final calculated angle and drive with the left stick.  Button should be released
            // before stick.  The default behavior of atan2 is 0 to -180 on Y Axis CCW, and 0 to
            // 180 CW.  This code normalizes that to 0 to 360 CCW from the Y Axis.
            driverAngle = toDegrees(atan2(yPower, xPower)) - robot.readIMU();
            xPower = 0.0;
            yPower = 0.0;
            spin = 0.0;
        }

        // When bumper is pressed, the speed changes
        bumpPressed = gamepad1.right_bumper;

        if(bumpPressed && !bumpHeld) {
            bumpHeld = true;
            // Check what speed we currently are.
            if(speedMultiplier == MAX_SPEED) {
                speedMultiplier = lowSpeed;
                spinMultiplier = lowSpin;
            } else {
                speedMultiplier = MAX_SPEED;
                spinMultiplier = MAX_SPIN;
            }
        }
        else if( !bumpPressed ) {
            bumpHeld = false;
        }
        robot.drive(speedMultiplier * xPower, speedMultiplier * yPower, spinMultiplier * spin, driverAngle);        bumpPressed = gamepad1.right_bumper;


        telemetry.addData("Y Power: ", yPower);
        telemetry.addData("X Power: ", xPower);
        telemetry.addData("Spin: ", spin);
        telemetry.addData("Offset Angle: ", driverAngle);
        telemetry.addData("Gyro Angle: ", gyroAngle);
        updateTelemetry(telemetry);


        x2Pressed = gamepad2.x;
        // If x2 was pressed, but not held
        if(x2Pressed && !x2Held) {
            x2Held = true;
            int newHeight = onbot.acq2.getCurrentPosition();

            newHeight += HEIGHT_INCREMENT;

            onbot.acq2.setTargetPosition(newHeight);
            onbot.acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            onbot.acq2.setPower(LIFT_SPEED);


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
                newHeight -= (LOWER) ;

                onbot.acq2.setTargetPosition(newHeight);
                onbot.acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                onbot.acq2.setPower(LIFT_SPEED);
                onbot.acq2.setTargetPosition(newHeight);
                onbot.acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                onbot.acq2.setPower((LIFT_SPEED));

            }
            else if(newHeight <= 300) {
                onbot.acq2.setTargetPosition(newHeight);
                onbot.acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                onbot.acq2.setPower(0);
            }
        } else if(!y2Pressed) {
            y2Held = false;

        }

        telemetry.addData("Current Position of Rotation", + onbot.acq1.getCurrentPosition());
        boolean turn = false;

        togglePressed = gamepad2.left_stick_button;
        // if trigger
        if(togglePressed && !toggleHeld) {
            toggleHeld = true;
            int newSpin = onbot.acq1.getCurrentPosition();

            newSpin += SPIN_INCREMENT;

            onbot.acq1.setTargetPosition(newSpin);
            onbot.acq1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            onbot.acq1.setPower(LIFT_SPEED);


        } else if(!togglePressed) {
            // This happens if the button is not being pressed at all, which resets that
            // we are holding it so the next time it is pressed it will trigger the action
            // again.
            togglePressed = false;
        }

        if(onbot.acq2.getCurrentPosition() > 2000 ) {
            turn = !turn;
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

        // ///////////////////////////////////////////////////////////////////
        // ////                                                           ////
        // ////                Foundation Hooks program                   ////
        // ////                                                           ////
        // ///////////////////////////////////////////////////////////////////

        // Open the hook system
        if (gamepad2.b) {
            position2 = 0.0;
            position3 = 0.5;
        }
        // secure the foundation
        else if(gamepad2.a ){
            position2 = 0.65;
            position3 = 0;
        }

        onbot.hook1.setPosition(position2);
        onbot.hook2.setPosition(position3);
    }
}
