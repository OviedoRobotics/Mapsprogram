///**
// *
// */
//package org.firstinspires.ftc.teamcode.Mapsprogram.Odometry;
//
//import org.firstinspires.ftc.teamcode.Mapsprogram.HardwareOmnibotDrive;
//
//public class Odomtest2rick {
//}
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;
//import org.firstinspires.ftc.teamcode.RobotUtilities.MovementVars;
//
//import org.firstinspires.ftc.teamcode.Mapsprogram.HardwareOmnibotDrive;
//import org.firstinspires.ftc.teamcode.Mapsprogram.Odometry.OdometryGlobalCoordinatePosition;
//
//
///**
//
// * Created by 12090 STEM Punk - Modified by MAPS
//
// */
//
//public abstract class OmniAutoXYOdoClass extends LinearOpMode {
//
//    protected ElapsedTime timer;
//
//    public static float mmPerInch = OmniAutoXYOdoClass.MM_PER_INCH;
//    public static float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
//    public static float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
////waypoints
//
//    protected WayPoint startLocation;
//    protected WayPoint distanceFromWall;
//    protected WayPoint positionToGrabSkystone1;
//    protected WayPoint grabSkystone1;
//    protected WayPoint pullBackSkystone1;
//    protected WayPoint quarryBeforeBridge;
//    protected WayPoint buildSiteUnderBridge;
//    protected WayPoint alignToFoundation;
//    protected WayPoint snuggleFoundation;
//    protected WayPoint grabFoundation;
//    protected WayPoint pullFoundation;
//    protected WayPoint pushFoundation;
//    protected WayPoint buildSiteReadyToRun;
//    protected WayPoint quarryUnderBridge;
//    protected WayPoint positionToGrabSkystone2;
//    protected WayPoint grabSkystone2;
//    protected WayPoint pullBackSkystone2;
//    protected WayPoint foundationDeposit;
//    protected WayPoint park;
//
//    protected ElapsedTime autoTimer = new ElapsedTime();
//    protected boolean liftIdle = true;
//
//    HardwareOmnibotDrive robot = new HardwareOmnibotDrive();
//
//    protected void driveToWayPoint(WayPoint destination, boolean passThrough, boolean pullingFoundation) {
//        // Loop until we get to destination.
//        updatePosition();
//        while(!driveToXY(destination.x, destination.y, destination.angle,
//                destination.speed, passThrough, pullingFoundation)
//                && opModeIsActive()) {
//            updatePosition();
//        }
//    }
//
//    // This is a special case where we want to pass through a point if we get
//    // the lift down in time.
//
//    protected void driveToWayPointMindingLift(WayPoint destination) {
//        // Loop until we get to destination.
//        updatePosition();
//        while(!driveToXY(destination.x, destination.y, destination.angle,
//                destination.speed, !liftIdle, false)
//                && opModeIsActive()) {
//            updatePosition();
//        }
//    }
//
//
//
//    protected void rotateToWayPointAngle(WayPoint destination, boolean pullingFoundation) {
//        // Move the robot away from the wall.
//        updatePosition();
//        rotateToAngle(destination.angle, pullingFoundation, true);
//        // Loop until we get to destination.
//        updatePosition();
//        while(!rotateToAngle(destination.angle, pullingFoundation, false) && opModeIsActive()) {
//            updatePosition();
//        }
//    }
//
//    public static final float MM_PER_INCH = 25.4f;
//
//    // Have to set this when we start motions
//
//    public double lastDriveAngle;
//
//    // This function is called during loops to progress started activities
//
//    public void performRobotActivities() {
//
//        robot.performLifting();
//        robot.performReleasing();
//        robot.performAligning();
//        robot.performGrabbing();
//
//    }
//
//    /**
//     * @param newWheelSize  - The size of the wheels, used to calculate encoder clicks per inch
//     * @param newMotorRatio - The motor gearbox ratio, used to calculate encoder clicks per inch
//     */
// //   public void setupRobotParameters(double newWheelSize, double newMotorRatio) {
// //       robot.init(hardwareMap);
// //       timer = new ElapsedTime();
//
////        robot.resetEncoders();
////        robot.setInputShaping(false);
////        myWheelSize = newWheelSize;
////        myMotorRatio = newMotorRatio;
//
////        clicksPerCm = (myMotorRatio * encoderClicksPerRev) / (Math.PI * myWheelSize * 2.54);
////    }
//
//    /**
//     * @param targetAngle  - The angle the robot should try to face when reaching destination.
//     * @param pullingFoundation - If we are pulling the foundation.
//     * @param resetDriveAngle - When we start a new drive, need to reset the starting drive angle.
//     * @return - Boolean true we have reached destination, false we have not
//     */
//    public boolean rotateToAngle(double targetAngle, boolean pullingFoundation, boolean resetDriveAngle) {
//        boolean reachedDestination = false;
//        double errorMultiplier = pullingFoundation ? 0.022 : 0.016;
//        double minSpinRate = pullingFoundation ? robot.MIN_FOUNDATION_SPIN_RATE : robot.MIN_SPIN_RATE;
//        double deltaAngle = MyPosition.AngleWrap(targetAngle - MyPosition.worldAngle_rad);
//        double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;
//        // This should be set on the first call to start us on a new path.
//        if(resetDriveAngle) {
//            lastDriveAngle = deltaAngle;
//        }
//
//        // We are done if we are within 2 degrees
//        if(Math.abs(Math.toDegrees(deltaAngle)) < 2) {
//            // We have reached our destination if the angle is close enough
//            robot.setAllDriveZero();
//            reachedDestination = true;
//            // We are done when we flip signs.
//        } else if(lastDriveAngle < 0) {
//            // We have reached our destination if the delta angle sign flips from last reading
//            if(deltaAngle >= 0) {
//                robot.setAllDriveZero();
//                reachedDestination = true;
//            } else {
//                // We still have some turning to do.
//                MovementVars.movement_x = 0;
//                MovementVars.movement_y = 0;
//                if(turnSpeed > -minSpinRate) {
//                    turnSpeed = -minSpinRate;
//                }
//                MovementVars.movement_turn = turnSpeed;
//                robot.ApplyMovement();
//            }
//        } else {
//            // We have reached our destination if the delta angle sign flips
//            if(deltaAngle <= 0) {
//                robot.setAllDriveZero();
//                reachedDestination = true;
//            } else {
//                // We still have some turning to do.
//                MovementVars.movement_x = 0;
//                MovementVars.movement_y = 0;
//                if(turnSpeed < minSpinRate) {
//                    turnSpeed = minSpinRate;
//                }
//                MovementVars.movement_turn = turnSpeed;
//                robot.ApplyMovement();
//            }
//        }
//        lastDriveAngle = deltaAngle;
//
//        return reachedDestination;
//    }
//
//    /**
//     * @param x           - The X field coordinate to go to.
//     * @param y           - The Y field coordinate to go to.
//     * @param targetAngle  - The angle the robot should try to face when reaching destination in radians.
//     * @param maxSpeed    - Sets the speed when we are driving through the point.
//     * @param passThrough - Slows the robot down to stop at destination coordinate.
//     * @param pullingFoundation - If we are pulling the foundation.
//     * @return - Boolean true we have reached destination, false we have not
//     */
//    public boolean driveToXY(double x, double y, double targetAngle, double maxSpeed,
//                             boolean passThrough, boolean pullingFoundation) {
//        boolean reachedDestination = false;
//        double errorMultiplier = pullingFoundation ? 0.020 : 0.014;
//        double minDriveMagnitude = pullingFoundation ? robot.MIN_FOUNDATION_DRIVE_MAGNITUDE : robot.MIN_DRIVE_MAGNITUDE;
//        double deltaX = x - MyPosition.worldXPosition;
//        double deltaY = y - MyPosition.worldYPosition;
//        double driveAngle = Math.atan2(deltaY, deltaX);
//        double deltaAngle = MyPosition.AngleWrap(targetAngle - MyPosition.worldAngle_rad);
//        double magnitude = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
//        double driveSpeed;
//        double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;
//        // Have to convert from world angles to robot centric angles.
//        double robotDriveAngle = driveAngle - MyPosition.worldAngle_rad + Math.toRadians(90);
//        double error = 2;
//
//        if(passThrough) {
//            error = 5;
//        }
//
//        // This will allow us to do multi-point routes without huge slowdowns.
//        // Such use cases will be changing angles, or triggering activities at
//        // certain points.
//        if(!passThrough) {
//            driveSpeed = magnitude * errorMultiplier;
//        } else {
//            driveSpeed = maxSpeed;
//        }
//
//        // Check if we passed through our point
//        if(magnitude <= error) {
//            reachedDestination = true;
//            if(!passThrough) {
//                robot.setAllDriveZero();
//            }
//        } else {
//            if(driveSpeed < minDriveMagnitude) {
//                driveSpeed = minDriveMagnitude;
//            }
//            MovementVars.movement_x = driveSpeed * Math.cos(robotDriveAngle);
//            MovementVars.movement_y = driveSpeed * Math.sin(robotDriveAngle);
//            MovementVars.movement_turn = turnSpeed;
//            robot.ApplyMovement();
//        }
//
//        return reachedDestination;
//    }
//
//    /**
//     * @param position    - The current encoder position
//     * @param destination - The desired encoder position
//     * @param speed       - The speed of travel used to get direction
//     * @return - Boolean true we have reached destination, false we have not
//     */
//    public boolean reachedClickPosition(int position, int destination, double speed, boolean reverseEncoders) {
//        boolean result = false;
//        if (reverseEncoders) {
//            if (speed < 0) {
//                if (position >= destination) {
//                    result = true;
//                }
//            } else {
//                if (position <= destination) {
//                    result = true;
//                }
//            }
//        } else {
//            if (speed > 0) {
//                if (position >= destination) {
//                    result = true;
//                }
//            } else {
//                if (position <= destination) {
//                    result = true;
//                }
//            }
//        }
//        return result;
//    }
//
//    /**
//     * @param speed        - The driving power
//     * @param rotateSpeed  - The rotational speed to correct heading errors
//     * @param driveAngle   - The angle of movement to drive the robot
//     * @param headingAngle - The heading angle to hold while driving
//     */
//    public void driveAtHeading(double speed, double rotateSpeed, double driveAngle, double headingAngle) {
//        double xPower = 0.0;
//        double yPower = 0.0;
//        double deltaAngle = 0.0;
//        final double SAME_ANGLE = 1;
//        double gyroReading = robot.readIMU();
//        deltaAngle = deltaAngle(headingAngle, gyroReading);
//
//        if (Math.abs(deltaAngle) > SAME_ANGLE) {
//            if (deltaAngle > 0.0) {
//                rotateSpeed = -rotateSpeed;
//            }
//        } else {
//            rotateSpeed = 0.0;
//        }
//
//        xPower = speed * Math.cos(Math.toRadians(driveAngle));
//        yPower = speed * Math.sin(Math.toRadians(driveAngle));
//        robot.drive(xPower, yPower, rotateSpeed, 0.0, false);
//    }
//
//    /**
//     * @param destinationAngle - The target angle to reach, between 0.0 and 360.0
//     * @param gyroReading      - The current angle of the robot
//     * @return The minumum angle to travel to get to the destination angle
//     */
//    private double deltaAngle(double destinationAngle, double gyroReading) {
//        double result = 0.0;
//        double leftResult = 0.0;
//        double rightResult = 0.0;
//
//        if (gyroReading > destinationAngle) {
//            leftResult = gyroReading - destinationAngle;
//            rightResult = 360.0 - gyroReading + destinationAngle;
//        } else {
//            leftResult = gyroReading + 360.0 - destinationAngle;
//            rightResult = destinationAngle - gyroReading;
//        }
//
//        if (leftResult < rightResult) {
//            result = -leftResult;
//        } else {
//            result = rightResult;
//        }
//
//        return result;
//    }
//
//    /**
//     * @param distanceToTravelMm - How far we are traveling in mm
//     * @param maxSpeed           - Top speed to travel
//     * @return The speed to go with the distance remaining
//     */
//    private double controlledDeceleration(double distanceToTravelMm, double maxSpeed) {
//        final double superFastDistance = 400.0;
//        final double fastDistance = 200.0;
//        final double mediumDistance = 100.0;
//        final double fastDivider = 1.4;
//        final double mediumDivider = 2.0;
//        final double slowDivider = 3.0;
//        double result = 0.0;
//
//        if (distanceToTravelMm > superFastDistance) {
//            result = maxSpeed;
//        } else if (distanceToTravelMm > fastDistance) {
//            result = maxSpeed / fastDivider;
//        } else if (distanceToTravelMm > mediumDistance) {
//            result = maxSpeed / mediumDivider;
//        } else {
//            result = maxSpeed / slowDivider;
//        }
//
//        return result;
//    }
//
//    /**
//     * @param distanceToTravelMm - How far we are traveling in mm
//     * @param maxSpeed           - Top speed to travel
//     * @return The speed to go with the distance remaining
//     */
//    private double controlledRotationMm(double distanceToTravelMm, double maxSpeed) {
//        final double fastDistance = 75.0;
//        final double mediumDistance = 50.0;
//        final double mediumDivider = 2.0;
//        final double slowDivider = 4.0;
//
//        double result = 0.0;
//
//        if (distanceToTravelMm > fastDistance) {
//            result = maxSpeed;
//        } else if (distanceToTravelMm > mediumDistance) {
//            result = maxSpeed / mediumDivider;
//        } else {
//            result = maxSpeed / slowDivider;
//
//        }
//
//        return result;
//    }
//
//    /**
//     * @param angleToTravel - How far we are traveling in degrees
//     * @param maxSpeed      - Top speed to travel
//     * @return The speed to go with the distance remaining
//     */
//    private double controlledRotationAngle(double angleToTravel, double maxSpeed) {
//        final double fastAngle = 30.0;
//        final double mediumAngle = 15.0;
//        final double mediumDivider = 1.5;
//        final double slowDivider = 3.0;
//        double angleToTravelAbs = Math.abs(angleToTravel);
//        double result = 0.0;
//
//        if (angleToTravelAbs > fastAngle) {
//            result = maxSpeed;
//        } else if (angleToTravelAbs > mediumAngle) {
//            result = maxSpeed / mediumDivider;
//        } else {
//            result = maxSpeed / slowDivider;
//        }
//
//        return result;
//    }
//}