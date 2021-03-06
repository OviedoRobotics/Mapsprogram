package org.firstinspires.ftc.teamcode.Mapsprogram.Odometry;

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
@TeleOp(name = "Odometry OpMode")
public class OdometryOpmode_1 extends LinearOpMode {

    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "r1", rbName = "r2", lfName = "l1", lbName = "l2";
    String verticalLeftEncoderName = lbName, verticalRightEncoderName = rfName, horizontalEncoderName = lfName;
    /*
    For Mapping purposes the front of the robot's face is the of the "A" of the chassis. Ignore the confusing names they wil be changed later.
     */

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();


        while(opModeIsActive()){
            //Display Global (x, y, theta) coordinates
            //  telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            //  telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            //  telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            //telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            // telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            //  telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            //  telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }




        //Stop the thread
        globalPositionUpdate.stop();

    }
    public void goToPosition(double targetXPosition, double targetYPosition, double robotPower, double desiredRobotOrientation, double allowableDistanceError){
        double x_world = globalPositionUpdate.returnXCoordinate();
        double y_world = globalPositionUpdate.returnYCoordinate();
        double angle_world = Math.toRadians(globalPositionUpdate.returnOrientation());
        double smallAngleSpeed = 0.25;

        double distance = Math.hypot(targetXPosition-x_world, targetYPosition - y_world);
        while(opModeIsActive() && distance > allowableDistanceError){

            distance = Math.hypot(targetXPosition-x_world, targetYPosition - y_world);


            double robotMovementAngle = Math.atan2(targetXPosition-x_world, targetYPosition - y_world);

            double relativeAngleToPoint = AngleWrap(robotMovementAngle - (Math.toRadians(globalPositionUpdate.returnOrientation())));

            double relativeXToPoint = calculateX(relativeAngleToPoint, distance);
            double relativeYToPoint = calculateY(relativeAngleToPoint, distance);

            double movementXPower = relativeXToPoint/ (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint/ (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

            double movement_x = movementXPower * robotPower;
            double movement_y = movementYPower * robotPower;

            double robot_radian_err = AngleWrap( Math.toRadians(desiredRobotOrientation) - angle_world );
            double small_rad_error = Math.abs( robot_radian_err / Math.toRadians(10.0) );
            double adjusted_turn_power = (small_rad_error <= 1.0)? (small_rad_error * smallAngleSpeed) : robotPower;
            double rotation_power = (robot_radian_err > 0.0)? adjusted_turn_power : -adjusted_turn_power;

            double relativeTurnAngle = relativeYToPoint - Math.toRadians(180) + desiredRobotOrientation;

            double movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * robotPower;

            double rf = movement_x - movement_y - movement_turn;
            double rb = movement_x + movement_y + movement_turn;
            double lb = movement_x + movement_y - movement_turn;
            double lf = movement_x - movement_y + movement_turn;

            right_front.setPower(rf);
            right_back.setPower(rb);
            left_front.setPower(lf);
            left_back.setPower(lb);

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


            telemetry.update();
        }

        right_front.setPower(0);
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.FORWARD);
        left_back.setDirection(DcMotorSimple.Direction.FORWARD);
        right_back.setDirection(DcMotorSimple.Direction.FORWARD);
        right_front.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param distance robot's relative distsnce
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double distance) {
        return Math.cos(Math.toRadians(desiredAngle)) * distance;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param distance robot's relative distance
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double distance) {
        return Math.sin(Math.toRadians(desiredAngle)) * distance;
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