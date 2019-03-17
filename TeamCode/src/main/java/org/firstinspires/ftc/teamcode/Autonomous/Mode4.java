package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.EasyRobot;

@Autonomous(name = "Regional - Timisoara(up)", group = "Official")
public class Mode4 extends EasyRobot {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot(this);

        getCollectorRotateServo().setPosition(0);

        initVuforia();
        initTfod(2);
        initGyro(BNO055IMU.AngleUnit.DEGREES);

        lock();

        waitForStart();

        getCollectorRotateServo().setPosition(1);

        touchDown();

        getCollectorRotateServo().setPosition(0.5);
        driveForward(15, 1);

        runObjectDetection(2, new ObjectDetected() {
            @Override
            public void pickup() {
                drive(getLandMotor(), -20, 0.3, true);

                waitForMotors(getLandMotor().getName());
                getCollectorRotateServo().setPosition(1);
                getCollectorServo().setPower(1);
                drive(getLandMotor(), -20, 0.05, true);
                waitForMotors(getLandMotor().getName());
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
                drive(getLandMotor(), 35, 0.3, true);
                waitForMotors(getLandMotor().getName());
                getCollectorRotateServo().setPosition(0);
                sleep(1500);
                getCollectorRotateServo().setPosition(0.5);

                driveForward(-13, 1);
                waitForMotors("m1", "m2", "m3", "m4");
                driveTicks(getAxLift(), 1240, 0.5, true, true);
                waitForMotors(getAxLift().getName());
                getAxLift().setBrake(true);

                getLiftServo1().setPosition(0);
                getLiftServo2().setPosition(1);

                getAxLift().setBrake(false);


                getLiftServo1().setPosition(0.5);
                getLiftServo2().setPosition(0.5);
            }
        });

//        driveForward(50, 0.8);
//        drive(getLandMotor(), -40, 0.5, true);
//        waitForMotors(getLandMotor().getName());
//        getCollectorRotateServo().setPosition(0.5);


        sleep(5000);
        disable();
    }

    private void lock() {
        getExtendLift().setBrake(true);
        getAxLift().setBrake(true);
        getCollectorRotateServo().setPosition(0);
    }

    private void touchDown() {
        getExtendLift().setBrake(false);
        getAxLift().setBrake(false);

        driveTicks(getAxLift(), 1240, 0.1, true, false);
        waitForMotors(getAxLift().getName());

        driveWith(0.3, 15, 15, -15, -15);
        driveForward(10, 0.3);
        driveWith(0.3, -15, -15, 15, 15);
        driveForward(-10, 0.3);

        driveTicks(getExtendLift(), 0, 0.1, true, false);
        waitForMotors(getExtendLift().getName());
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