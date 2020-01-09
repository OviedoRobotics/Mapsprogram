package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class LoadingZoneR extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    HardwareOmnibotDrive robot = new HardwareOmnibotDrive();
    HardwareSensors onbot = new HardwareSensors();
    SkystoneDetection detect = new SkystoneDetection();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        onbot.init(hardwareMap);
        robot.initIMU();

        detect.initCamera(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        telemetry.addData("Status", "Running");
        telemetry.update();

        int location = detect.skystoneLocation();
        telemetry.addData("Skystone", location);
        telemetry.update();
        sleep(10000);


        shuffle(0.5, 1);
        sleep(1000);
        drive(0.5, 1);
        sleep(1000);


    }

    public void shuffle(double power, int direction ) {
        robot.frontRight.setPower(power*-1);
        robot.rearRight.setPower(power);
        robot.frontLeft.setPower(power*-1);
        robot.rearLeft.setPower(power);
    }
    public void drive( double power,int direction) {
        robot.frontRight.setPower(power);
        robot.rearRight.setPower(power);
        robot.frontLeft.setPower(power*-1);
        robot.rearLeft.setPower(power*-1);
    }
    public void turn(double degrees, int direction) {

    }

}


