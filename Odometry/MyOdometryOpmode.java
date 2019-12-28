package org.firstinspires.ftc.teamcode.Mapsprogram.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Mapsprogram.HardwareOmnibotDrive;
import org.firstinspires.ftc.teamcode.Mapsprogram.Odometry.OdometryGlobalCoordinatePosition;

/**
 * Created by Sarthak on 10/4/2019.
 */
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {

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

        goToPosition(20*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, 0, 3*COUNTS_PER_INCH);


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
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);
        while(opModeIsActive() && distance > allowableDistanceError){
            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            distanceToYTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToXTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();


            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToYTarget, distanceToXTarget));

            double relativeXToPoint = calculateX(robotMovementAngle, distance) * robotPower;
            double relativeYToPoint = calculateY(robotMovementAngle, distance) * robotPower;

            double movementXPower = relativeXToPoint/ (Math.abs(relativeXToPoint) * Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint/ (Math.abs(relativeXToPoint) * Math.abs(relativeYToPoint));


            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();
            double movement_turn = Range.clip(pivotCorrection/Math.toRadians(30), -1, 1)* robotPower;

            double rf = movementXPower - movementYPower;
            double rb = movementXPower + movementYPower;
            double lf = movementYPower - movementXPower;
            double lb = (-movementXPower - movementYPower);
//
//            right_front.setPower(rf);
//            right_back.setPower(rb);
//            left_front.setPower(lf);
//            left_back.setPower(lb);

            telemetry.addData("right front power", rf);
            telemetry.addData("right back power", rb);
            telemetry.addData("left front power", lf);
            telemetry.addData("left back power", lb);
            telemetry.addData("Robot movement x", relativeXToPoint);
            telemetry.addData("Robot movement y", relativeYToPoint);

            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate());
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Distance", distance);
//            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
//
//            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
//            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
//            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

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
}