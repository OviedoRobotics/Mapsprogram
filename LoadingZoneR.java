//package org.firstinspires.ftc.teamcode.Mapsprogram;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Autonomous
//public class LoadingZoneR extends LinearOpMode{
//    private ElapsedTime runtime = new ElapsedTime();
//    HardwareOmnibotDrive robot = new HardwareOmnibotDrive();
//    HardwareSensors onbot = new HardwareSensors();
//    SkystoneDetection detect = new SkystoneDetection();
//    int location = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        robot.init(hardwareMap);
//        onbot.init(hardwareMap);
//        robot.initIMU();
//
//        detect.initCamera(hardwareMap);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        // run until the end of the match (driver presses STOP)
//        telemetry.addData("Status", "Running");
//        telemetry.update();
//
//
//        while(location == 0){
//            location = detect.skystoneLocation();
//            telemetry.addData("Skystone", location);
//            telemetry.update();
//        }
//        onbot.denest();
//        sleep(1000);
//        if(location == 1){
//            shuffle(-1);
//            sleep(500);
//        }
//        else if(location == 2)
//            ;
//        else if(location == 3){
//            shuffle(0.5);
//            sleep(450);
//        }
//        drive(0.30);
//        sleep(1000);
//        onbot.arm.setPosition(1.0);
//        sleep(100);
//        drive(0.30);
//        sleep(200);
//        delift();
//        sleep(100);
//        onbot.arm.setPosition(.6);
//        sleep(100);
//        onbot.fineMovemnt.setPosition(1.0);
//        drive(-0.40);
//        sleep(100);
//        if(location == 1){
//            shuffle(-1);
//            sleep(500);
//        }
//        else if(location == 2)
//            ;
//        else if(location == 3){
//        double position = robot.readIMU();
//        while(position != 90) {
//            turn(1);
//        }
//
//        }
//
//
//
//
//
//
//
//    }
//
//    public void shuffle(double power) {
//        robot.frontRight.setPower(power*-1);
//        robot.rearRight.setPower(power/2);
//        robot.frontLeft.setPower((power/2)*-1);
//        robot.rearLeft.setPower(power);
//    }
//    public void drive( double power) {
//        robot.frontRight.setPower(power);
//        robot.rearRight.setPower(power);
//        robot.frontLeft.setPower(power*-1);
//        robot.rearLeft.setPower(power*-1);
//    }
//    public void turn(double power) {
//        robot.frontRight.setPower(power*-1);
//        robot.rearRight.setPower(power);
//        robot.frontLeft.setPower(power);
//        robot.rearLeft.setPower(power*-1);
//    }
//    public void lift(){
//        onbot.acq2.setTargetPosition(1300);
//        onbot.acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        onbot.acq2.setPower(1);
//    }
//    public void nest(){
//        onbot.acq2.setTargetPosition(1300);
//        onbot.acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        onbot.acq2.setPower(1);
//
//        onbot.acq1.setTargetPosition(0);
//        onbot.acq1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        onbot.acq1.setPower(1);
//    }
//    public void delift(){
//        onbot.acq2.setTargetPosition(0);
//        onbot.acq2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        onbot.acq2.setPower(1);
//    }
//
//
//}


