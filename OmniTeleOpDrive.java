package org.firstinspires.ftc.teamcode.Mapsprogram;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    @Override
    public void init() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        robot.init(hardwareMap);
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
    }
}
