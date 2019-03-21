package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.EasyRobot;

@Autonomous(name = "Regional - Timisoara(upp)", group = "Official")
public class AutoMode5 extends EasyRobot {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot(this);

        initVuforia();
        initTfod(2);
        initGyro(BNO055IMU.AngleUnit.DEGREES);

        waitForStart();

        touchDown();

        runObjectDetection(2, new ObjectDetected() {
            @Override
            public void pickup() {
//
//                driveTicks(getExtendLift(), 2000, 1.0, true, false);// TODO: Extinde glisiera in pozitia in care poate strange minerale
            }

            @Override
            public void onLeft() {
                turnTo(0.5, -27);
            }

            @Override
            public void onCenter() {

            }

            @Override
            public void onRight() {
                turnTo(0.5, 27);
            }

            @Override
            public void loadCargo() {
                getCollectorServo().setPower(1);
                driveForward(45,0.8);
                waitForMotors("m1", "m2", "m3", "m4");
                driveForward(15,0.5);
                waitForMotors("m1", "m2", "m3", "m4");
                getTeamMarker().setPosition(1);
                sleep(4000);
                driveForward(-60, 0.8);
                waitForMotors("m1", "m2", "m3", "m4");
//
//                getAxLift().setBrake(false);
//                driveTicks(getAxLift(), 1000, 1.0, true, false);// TODO: 3/16/2019 pozitie 45 grade
//                getAxLift().setBrake(true);
//
//                sleep(2000);
//
//                getAxLift().setBrake(false);
//                driveTicks(getAxLift(), 1000, 1.0, true, false);// TODO: 3/16/2019 pozitie sus
//                getAxLift().setBrake(true);

            }
        });

        sleep(20000);
        disable();
    }

    private void touchDown() {
        driveTicks(getLandMotor(),1000 , 1, true, false);
        waitForMotors(getLandMotor().getName());

        driveWidth(0.3, 20, 20, -20, -20);
        driveForward(10, 0.3);
        driveWidth(0.3, -20, -20, 20, 20);

        driveTicks(getLandMotor(),0 , 1, true, false);
    }

    public void driveForward(int distance, double speed) {
        drive(getLeftMotorUp(), -distance, speed, false);
        drive(getRightMotorUp(), distance, speed, false);
        drive(getLeftMotorDown(), -distance, speed, false);
        drive(getRightMotorDown(), distance, speed, false);
        update("m1", "m2", "m3", "m4");
    }


    public void driveWidth(double speed, int distanceLeftUp, int distanceRightUp, int distanceLeftDown, int distanceRightDown) {
        drive(getLeftMotorUp(), distanceLeftUp, speed, false);
        drive(getRightMotorUp(), distanceRightUp, speed, false);
        drive(getLeftMotorDown(), distanceLeftDown, speed, false);
        drive(getRightMotorDown(), distanceRightDown, speed, false);
        update("m1", "m2", "m3", "m4");
    }

}