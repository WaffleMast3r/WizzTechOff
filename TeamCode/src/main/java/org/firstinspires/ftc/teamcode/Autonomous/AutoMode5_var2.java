package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.EasyRobot;

@Autonomous(name = "Autonomous - Nationala (only down)", group = "Official")
public class AutoMode5_var2 extends EasyRobot {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot(this);


//        initVuforia();
//        initTfod(2);
        initGyro(BNO055IMU.AngleUnit.DEGREES);

        waitForStart();
        touchDown();
//
//        drive(getLeftMotorDown(), 10, 0.7, false);
//        drive(getLeftMotorUp(), 10, 0.7, false);
//        update("m1", "m3");
//
//        sleep(1000);

        driveWidth(1, -20, 20, 20, -20);
        waitForMotors("m1", "m2", "m3", "m4");
        driveForward(-10,1);
        waitForMotors("m1", "m2", "m3", "m4");


        driveWidth(1, -20, 20, 20, -20);
        waitForMotors("m1", "m2", "m3", "m4");
        driveForward(-10,1);
        waitForMotors("m1", "m2", "m3", "m4");


        driveWidth(1, -20, 20, 20, -20);
        waitForMotors("m1", "m2", "m3", "m4");
        driveForward(-10,1);
        waitForMotors("m1", "m2", "m3", "m4");

        driveWidth(1, -20, 20, 20, -20);
        waitForMotors("m1", "m2", "m3", "m4");
        driveForward(-10,1);
        waitForMotors("m1", "m2", "m3", "m4");

//        runObjectDetection(2, new ObjectDetected() {
//            @Override
//            public void pickup() {
//            }
//
//            @Override
//            public void onLeft() {
//                turnTo(0.3, -27);
//                driveForward(-30, 0.6);
//                waitForMotors("m1", "m2", "m3", "m4");
//                turnTo(0.3, 10);
//
//                driveForward(-20, 0.4);
//                waitForMotors("m1", "m2", "m3", "m4");
//                getTeamMarker().setPosition(1);
//                sleep(4000);
//                turnTo(0.3, 10);
//
//                driveForward(20, 0.4);
//                waitForMotors("m1", "m2", "m3", "m4");
//                getTeamMarker().setPosition(0);
//
//                turnTo(0.3, -27);
//                driveForward(40, 0.6);
//                waitForMotors("m1", "m2", "m3", "m4");
//            }
//
//            @Override
//            public void onCenter() {
//                driveForward(-30, 0.4);
//                waitForMotors("m1", "m2", "m3", "m4");
//
//                driveForward(-30, 0.4);
//                waitForMotors("m1", "m2", "m3", "m4");
//                getTeamMarker().setPosition(1);
//                sleep(4000);
//                turnTo(0.4, 0);
//
//                driveForward(20, 0.4);
//                waitForMotors("m1", "m2", "m3", "m4");
//                getTeamMarker().setPosition(0);
//
//                driveForward(40, 0.4);
//                waitForMotors("m1", "m2", "m3", "m4");
//            }
//
//            @Override
//            public void onRight() {
//                turnTo(0.3, 27);
//                driveForward(-30, 0.6);
//                waitForMotors("m1", "m2", "m3", "m4");
//                turnTo(0.3, -10);
//
//                driveForward(-20, 0.4);
//                waitForMotors("m1", "m2", "m3", "m4");
//                getTeamMarker().setPosition(1);
//                sleep(4000);
//                turnTo(0.3, -10);
//
//                driveForward(20, 0.4);
//                waitForMotors("m1", "m2", "m3", "m4");
//                getTeamMarker().setPosition(0);
//
//                turnTo(0.3, 27);
//                driveForward(40, 0.6);
//                waitForMotors("m1", "m2", "m3", "m4");
//            }
//
//            @Override
//            public void loadCargo() {
//            }
//        });
        disable();
        sleep(50000);

    }

    private void touchDown() {
//        getLandMotor().getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
//        driveTicks(getLandMotor(),6000 , 1, true, true);
        drive(getLandMotor(), -470, 1, true); //aprox 4cm=100
        waitForMotors(getLandMotor().getName());

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