/* FTC Team 7572 - Version 2.0 (01/08/2020)
 */
package org.firstinspires.ftc.teamcode.Mapsprogram.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mapsprogram.HardwareOmnibotDrive;
import org.firstinspires.ftc.teamcode.Mapsprogram.Odometry.OdometryGlobalCoordinatePosition;

/**
 * Autonomous
 */

@Autonomous(name="Odometry UnitTest", group="7592")
//@Disabled
public class OdometryTEST extends LinearOpMode {

    boolean ODOMETRY_DEBUG = false;
    int     ODOMETRY_UPDATE_DELAY = 75;  // msec
    final double COUNTS_PER_INCH = 307.699557;

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    HardwareOmnibotDrive robot = new HardwareOmnibotDrive();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("State", "Initializing (please wait)");
        telemetry.update();

        // Initialize robot hardware
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("State", "Ready");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition( robot.leftEncoder(), robot.rightEncoder(), robot.leftEncoder(), COUNTS_PER_INCH, ODOMETRY_UPDATE_DELAY );
        globalPositionUpdate.globalCoordinatePositionSet( (0.0 * COUNTS_PER_INCH), (0.0 * COUNTS_PER_INCH), 0.0 );

        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        unitTestOdometryDrive();

        telemetry.addData("Program", "Complete");
        telemetry.update();
    } /* runOpMode() */

    /*--------------------------------------------------------------------------------------------*/
    // TEST CODE: Verify gyro-based motion functions against a tape measure
    private void unitTestOdometryDrive() {
        double fast_power=0.50, fast_xy_tol = 2.00, fast_ang_tol = 2.5;   // power, inches, degrees
        double slow_power=0.25, slow_xy_tol = 0.75, slow_ang_tol = 1.0;   // power, inches, degrees

        driveToPosition( 10.0, 0.0, 0.0, slow_power,slow_power, slow_xy_tol, slow_ang_tol );

        robot.setAllDriveZero();
        sleep( 2000 );
//
//        driveToPosition( 10.0, 10.0, 0.0, slow_power,slow_power, slow_xy_tol, slow_ang_tol );
//
//        robot.setAllDriveZero();
//        sleep( 2000 );
//
//        driveToPosition( 10.0, 10.0, 90.0, slow_power,slow_power, slow_xy_tol, slow_ang_tol );
    } // unitTestOdometryDrive

    /*--------------------------------------------------------------------------------------------*/
    /**
     * Move robot to specified target position/orientation
     * @param x_target (inches)
     * @param y_target (inches)
     * @param drive_angle (degrees; 0deg is straight ahead)
     * @param move_power
     * @param turn_power
     * @param xy_tol (inches)
     * @param ang_tol (deg)
     * @return boolean true/false for DONE?
     */
    private void driveToPosition( double x_target, double y_target, double drive_angle,
                                  double move_power, double turn_power,
                                  double xy_tol, double ang_tol ) {
        // Loop until we reach the target (or autonomous program aborts)
        while( opModeIsActive() ) {
            if( ODOMETRY_DEBUG ) {
                telemetry.addData("World X (inches)", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                telemetry.addData("World Y (inches)", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                telemetry.addData("Orientation (deg)", globalPositionUpdate.returnOrientation());
                telemetry.update();
                sleep(3500);  // so we can read it
            } // ODOMETRY_DEBUG
            else{
                telemetry.addData("World X (inches)", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                telemetry.addData("World Y (inches)", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                telemetry.addData("Orientation (deg)", globalPositionUpdate.returnOrientation());
                telemetry.update();
            }
            // Power drivetrain motors to move to where we want to be
            if( moveToPosition( x_target, y_target, drive_angle, move_power, turn_power, xy_tol, ang_tol ) )
                break;
            // Don't recompute until we've had an odometry position update
            sleep( ODOMETRY_UPDATE_DELAY );
            // Do we need to stop and assess?
            if( ODOMETRY_DEBUG ) {
                // Allow time for motors to produce movement (and 3 odometry updates)
                sleep(225);
                // DEBUG!  Power-off motors until our next computational cycle
                robot.setAllDriveZero();
            } // ODOMETRY_DEBUG
        } // opModeIsActive()

    } // driveToPosition

    /*--------------------------------------------------------------------------------------------*/
    /**
     * Compute instantaneous motor powers needed to move toward the specified target position/orientation
     * @param x_target (inches)
     * @param y_target (inches)
     * @param drive_angle (degrees; 0deg is straight ahead)
     * @param move_power
     * @param turn_power
     * @param xy_tol (inches)
     * @param ang_tol (deg)
     * @return boolean true/false for DONE?
     */
    private boolean moveToPosition( double x_target, double y_target, double drive_angle,
                                    double move_power, double turn_power,
                                    double xy_tol, double ang_tol ) {
        double smallAngleSpeed = 0.25;
        // Query the current robot position/orientation
        double x_world = globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH;  // inches
        double y_world = globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH;  // inches
        double angle_world = Math.toRadians( globalPositionUpdate.returnOrientation() );    // radians
        // Compute distance and angle-offset to the target point
        double distanceToPoint   = Math.sqrt( Math.pow((x_target - x_world),2.0) + Math.pow((y_target - y_world),2.0) );
        double angleToPoint      = Math.atan2( (y_target - y_world), (x_target - x_world) ); // radians`
        double deltaAngleToPoint = AngleWrap( angleToPoint - angle_world );                  // radians
        // Compute x & y components required to move toward point (with angle correction)
        double relative_x_to_point = Math.cos(deltaAngleToPoint) * distanceToPoint;
        double relative_y_to_point = Math.sin(deltaAngleToPoint) * distanceToPoint;
        // Compute absolute-value x and y distances for scaling
        double relative_x_abs = Math.abs( relative_x_to_point );
        double relative_y_abs = Math.abs( relative_y_to_point );
        // Compute movement power, while preserving shape/ratios of the intended movement direction
        double movement_x_power = (relative_x_to_point / (relative_y_abs + relative_x_abs)) * move_power;
        double movement_y_power = (relative_y_to_point / (relative_y_abs + relative_x_abs)) * move_power;
        // Compute robot orientation-angle error
        double robot_radian_err = AngleWrap( Math.toRadians(drive_angle) - angle_world );  // radians
        // If within 10deg of target angle, use reduced turn_power
        double small_rad_error = Math.abs( robot_radian_err / Math.toRadians(10.0) );
        double adjusted_turn_power = (small_rad_error <= 1.0)? (small_rad_error * smallAngleSpeed) : turn_power;
        double rotation_power = (robot_radian_err > 0.0)? adjusted_turn_power : -adjusted_turn_power;
        // Translate X,Y,rotation power levels into mecanum wheel power values
        double frontRight  = movement_x_power - movement_y_power - rotation_power;
        double frontLeft = movement_x_power + movement_y_power + rotation_power;
        double backRight  = movement_x_power + movement_y_power - rotation_power;
        double backLeft   = movement_x_power - movement_y_power + rotation_power;
        // Determine the maximum motor power
        double maxWheelPower = Math.max( Math.max( Math.abs(backLeft),  Math.abs(backRight)  ),
                Math.max( Math.abs(frontLeft), Math.abs(frontRight) ) );
        if( ODOMETRY_DEBUG ) {
            telemetry.addData("distanceToPoint", "%.2f in", distanceToPoint);
            telemetry.addData("angleToPoint", "%.4f deg", Math.toDegrees(angleToPoint));
            telemetry.addData("deltaAngleToPoint", "%.4f deg", Math.toDegrees(angleToPoint));
            telemetry.addData("relative_x_to_point", "%.2f in", relative_x_to_point);
            telemetry.addData("relative_y_to_point", "%.2f in", relative_y_to_point);
            telemetry.addData("robot_radian_err", "%.4f deg", Math.toDegrees(robot_radian_err));
            telemetry.addData("movement_x_power", "%.2f", movement_x_power);
            telemetry.addData("movement_y_power", "%.2f", movement_y_power);
            telemetry.addData("rotation_power", "%.2f", rotation_power);
            telemetry.update();
            sleep(3500);  // so we can read it
        }
        // Are we within tolerance of our target position/orientation?
        if( (Math.abs(distanceToPoint) <= xy_tol) && (Math.abs(robot_radian_err) <= Math.toRadians(ang_tol))  )
            return true;
        // NOT DONE!  Ensure no wheel powers exceeds 1.0
        if( maxWheelPower > 1.0 )
        {
            backLeft   /= maxWheelPower;
            backRight  /= maxWheelPower;
            frontLeft  /= maxWheelPower;
            frontRight /= maxWheelPower;
        }
        // Update motor power settings:
        //robot.setPowerforAll( frontRight, backRight, frontLeft, backLeft );
        return false;
    } // moveToPosition

    /**
     * Ensure angle is in the range of -PI to +PI (-180 to +180 deg)
     * @param angle
     * @return
     */
    public double AngleWrap( double angle ){
        while( angle < -Math.PI ) {
            angle += 2.0*Math.PI;
        }
        while( angle > Math.PI ){
            angle -= 2.0*Math.PI;
        }
        return angle;
    }

} /* Autonomous0 */
