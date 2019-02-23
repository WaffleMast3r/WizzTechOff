package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.EasyRobot;

@Autonomous(name = "Regional - Timisoara", group = "Official")
public class Mode3 extends EasyRobot {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot(this);

        getCollectorRotateServo().setPosition(0);

        initVuforia();
        initTfod(2);
        initGyro(BNO055IMU.AngleUnit.DEGREES);

//        lock();

        waitForStart();

        getCollectorRotateServo().setPosition(1);

//        touchDown();

        getCollectorRotateServo().setPosition(0.5);
        driveForward(15, 1);

        runObjectDetection(2, new ObjectDetected() {
            @Override
            public void pickup() {
                drive(getHandMotor(), -20, 0.3, true);
                waitForMotors(getHandMotor().getName());
                getCollectorRotateServo().setPosition(1);
                getCollectorServo().setPower(1);
                drive(getHandMotor(), -20, 0.05, true);
                waitForMotors(getHandMotor().getName());
                getCollectorServo().setPower(0);
                getCollectorRotateServo().setPosition(0.5);
            }

            @Override
            public void onLeft() {
                turnTo(0.3, 0);
            }

            @Override
            public void onCenter() {
                getCollectorRotateServo().setPosition(0.5);
                sleep(500);
                driveForward(50, 1);
                getCollectorRotateServo().setPosition(1);
                getCollectorServo().setPower(1);
                driveForward(30, 0.3);
                waitForMotors("m1", "m2", "m3", "m4");
                getCollectorRotateServo().setPosition(0.5);
                getCollectorServo().setPower(0);
                driveForward(-30, 1);
            }

            @Override
            public void onRight() {
                turnTo(0.3, 0);
            }

            @Override
            public void loadCargo() {
                drive(getHandMotor(), 35, 0.3, true);
                waitForMotors(getHandMotor().getName());
                getCollectorRotateServo().setPosition(0);
                sleep(1500);
                getCollectorRotateServo().setPosition(0.5);

                driveForward(-13, 1);
                waitForMotors("m1", "m2", "m3", "m4");
//                driveTicks(getExtendLift(), 1240, 0.5, true, true);
//                waitForMotors(getExtendLift().getName());
//                getExtendLift().setBrake(true);
//
//                getLiftServo1().setPosition(0);
//                getLiftServo2().setPosition(1);
//
//                getExtendLift().setBrake(false);


//                getLiftServo1().setPosition(0.5);
//                getLiftServo2().setPosition(0.5);
            }
        });

        driveForward(50, 0.8);
//        drive(getHandMotor(), -40, 0.5, true);
//        waitForMotors(getHandMotor().getName());
//        getCollectorRotateServo().setPosition(0.5);


        sleep(5000);
        disable();
    }

    private void lock() {
        getExtendLift2().setBrake(true);
        getExtendLift().setBrake(true);
        getCollectorRotateServo().setPosition(0);
    }

    private void touchDown() {
        getSortatorServo1().setPosition(1);
        getSortatorServo2().setPosition(1);
        getExtendLift2().setBrake(false);
        getExtendLift().setBrake(false);

        driveTicks(getExtendLift(), 1240, 0.1, true, false);
        waitForMotors(getExtendLift().getName());

        driveWith(0.3, 15, 15, -15, -15);
        driveForward(10, 0.3);
        driveWith(0.3, -15, -15, 15, 15);
        driveForward(-10, 0.3);

        driveTicks(getExtendLift2(), 0, 0.1, true, false);
        waitForMotors(getExtendLift2().getName());
    }

    public void driveForward(int distance, double speed) {
        drive(getLeftMotorUp(), -distance, speed, false);
        drive(getRightMotorUp(), distance, speed, false);
        drive(getLeftMotorDown(), -distance, speed, false);
        drive(getRightMotorDown(), distance, speed, false);
        update("m1", "m2", "m3", "m4");
    }

    public void driveWith(double speed, int distanceLeftUp, int distanceRightUp, int distanceLeftDown, int distanceRightDown) {
        drive(getLeftMotorUp(), distanceLeftUp, speed, false);
        drive(getRightMotorUp(), distanceRightUp, speed, false);
        drive(getLeftMotorDown(), distanceLeftDown, speed, false);
        drive(getRightMotorDown(), distanceRightDown, speed, false);
        update("m1", "m2", "m3", "m4");
    }

}
