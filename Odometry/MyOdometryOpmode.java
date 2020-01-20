package org.firstinspires.ftc.teamcode.Mapsprogram.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Mapsprogram.HardwareOmnibotDrive;
import org.firstinspires.ftc.teamcode.Mapsprogram.Odometry.OdometryGlobalCoordinatePosition;

/**
 * Modified by Jazmyn on 12/20/19
 */
@Autonomous(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {

    boolean ODOMETRY_DEBUG = false;

    //Drive motors
    HardwareOmnibotDrive robot = new HardwareOmnibotDrive();


    final double COUNTS_PER_INCH = 307.699557;

    /*
    For Mapping purposes the front of the robot's face is the of the "A" of the chassis. Ignore the confusing names they wil be changed later.
     */

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        // Show the status of initialization
        robot.init(hardwareMap);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.leftEncoder(), robot.rightEncoder(), robot.horizontalEncoder(), COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        unitTestOdometryDrive();

        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", robot.leftEncoder().getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.rightEncoder().getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.horizontalEncoder().getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }




        //Stop the thread
        globalPositionUpdate.stop();

    }
    /*--------------------------------------------------------------------------------------------*/
    // TEST CODE: Verify gyro-based motion functions against a tape measure
    private void unitTestOdometryDrive() {
        double fast_power=0.50, fast_xy_tol = 2.00, fast_ang_tol = 2.5;   // power, inches, degrees
        double slow_power=0.25, slow_xy_tol = 0.75, slow_ang_tol = 1.0;   // power, inches, degrees

        goToPosition( 20.0 * COUNTS_PER_INCH, 0.0 * COUNTS_PER_INCH, 0.5, 0, 1*COUNTS_PER_INCH  );

        robot.setAllDriveZero();
        sleep( 2000 );


    } // unitTestOdometryDrive

    /*--------------------------------------------------------------------------------------------*/
    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        // The current Distance to point
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
        double angleWorld = Math.toRadians(globalPositionUpdate.returnOrientation());
        double smallAngleSpeed = 0.25;
        //int timesLooped = 0;

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        while(opModeIsActive() && distance > allowableDistanceError){
            //timesLooped++;
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.atan2(distanceToYTarget, distanceToXTarget);
            angleWorld = Math.toRadians(globalPositionUpdate.returnOrientation());

            double relativeAngleToPoint = AngleWrap(robotMovementAngle - angleWorld);

            double relativeXToPoint = calculateX(relativeAngleToPoint, distance);
            double relativeYToPoint = calculateY(relativeAngleToPoint, distance);

            double movementXPower = relativeXToPoint/ (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint/ (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

            double movement_x = movementXPower * robotPower;
            double movement_y = movementYPower * robotPower;

            double relativeTurnAngle = AngleWrap(Math.toRadians(desiredRobotOrientation)- angleWorld);

            double movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * robotPower;

            double rf = movement_x - movement_y - movement_turn;
            double lf = movement_x + movement_y + movement_turn;
            double rb = movement_x + movement_y - movement_turn;
            double lb = movement_x - movement_y + movement_turn;

            double maxWheelPower = Math.max( Math.max( Math.abs(lb),  Math.abs(rb)  ),
                    Math.max( Math.abs(lf), Math.abs(rf) ) );
            if( ODOMETRY_DEBUG ) {
                //Display Global (x, y, theta) coordinates
                telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

                telemetry.addData("Vertical left encoder position", robot.leftEncoder().getCurrentPosition());
                telemetry.addData("Vertical right encoder position", robot.rightEncoder().getCurrentPosition());
                telemetry.addData("horizontal encoder position", robot.horizontalEncoder().getCurrentPosition());

                telemetry.update();
                sleep(3500);  // so we can read it
            }

            if( maxWheelPower > 1.0 )
            {
                lb   /= maxWheelPower;
                rb  /= maxWheelPower;
                lf  /= maxWheelPower;
                rf /= maxWheelPower;
            }
            robot.setFrontLeftMotorPower(lf);
            robot.setFrontRightMotorPower(rf);
            robot.setRearLeftMotorPower(lb);
            robot.setRearRightMotorPower(rb);

            telemetry.addData("right front power", rf);
            telemetry.addData("right back power", rb);
            telemetry.addData("left front power", lf);
            telemetry.addData("left back power", lb);
            telemetry.addData("Robot movement x", movementXPower);
            telemetry.addData("Robot movement y", movementYPower);

            telemetry.addData("Movement angle", robotMovementAngle);

            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate()/ COUNTS_PER_INCH);
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Distance", distance/ COUNTS_PER_INCH);
            //telemetry.addData("Times Looped", timesLooped);

            telemetry.update();
        }

        robot.setFrontLeftMotorPower(0);
        robot.setFrontRightMotorPower(0);
        robot.setRearLeftMotorPower(0);
        robot.setRearRightMotorPower(0);
    }


    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param distance robot's relative distsnce
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double distance) {
        return Math.cos(desiredAngle) * distance;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param distance robot's relative distance
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double distance) {
        return Math.sin(desiredAngle) * distance;
    }

    public static double AngleWrap(double angle){
        while (angle < -Math.PI){
            angle +=2*Math.PI;
        }
        while (angle > Math.PI){
            angle -=2*Math.PI;
        }
        return angle;

    }

}