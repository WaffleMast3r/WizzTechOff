package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.EasyRobot;
import org.firstinspires.ftc.teamcode.MotorEncoderController;

@Autonomous(name = "Demo - Bucuresti", group = "Official")
public class Mode3 extends EasyRobot {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot(this);

        initVuforia();
        initTfod(2);
        initGyro(BNO055IMU.AngleUnit.DEGREES);
        lock();

        waitForStart();

//        touchDown();\
        drive(10, 1);

        runObjectDetection(2, new ObjectDetected() {
            @Override
            public void pickup() {
                MotorEncoderController.getInstance().drive(getHandMotor(), -20, 0.3, true);
                MotorEncoderController.getInstance().waitForMotors(getHandMotor().getName());
                getCollectorRotateServo().setPosition(1);
                getCollectorServo().setPower(1);
                MotorEncoderController.getInstance().drive(getHandMotor(), -20, 0.05, true);
                MotorEncoderController.getInstance().waitForMotors(getHandMotor().getName());
                getCollectorServo().setPower(0);
                getCollectorRotateServo().setPosition(0.5);
            }

            @Override
            public void onLeft() {
                turnTo(0.5, 0);
            }

            @Override
            public void onCenter() {
                drive(50, 1);
                getCollectorRotateServo().setPosition(1);
                getCollectorServo().setPower(1);
                drive(30, 0.3);
                MotorEncoderController.getInstance().waitForMotors("m1", "m2", "m3", "m4");
                getCollectorRotateServo().setPosition(0.5);
                getCollectorServo().setPower(0);
                drive(-30, 1);
            }

            @Override
            public void onRight() {
                turnTo(0.5, 0);
            }

            @Override
            public void loadCargo() {
                MotorEncoderController.getInstance().drive(getHandMotor(), 35, 0.3, true);
                MotorEncoderController.getInstance().waitForMotors(getHandMotor().getName());
                getCollectorRotateServo().setPosition(0);

                drive(-20, 1);
                turnTo(0.5, -10);
                MotorEncoderController.getInstance().drive(getExtendLift(), 10, 0.3, true);
                MotorEncoderController.getInstance().waitForMotors(getExtendLift().getName());

                getLiftServo1().setPosition(0);
                getLiftServo2().setPosition(1);


                getLiftServo1().setPosition(0.5);
                getLiftServo2().setPosition(0.5);
            }
        });

        sleep(20000);

        MotorEncoderController.disable();
        disable();
    }

    private void lock() {
        getExtendLift2().setBrake(true);
        getExtendLift().setBrake(true);
        getCollectorRotateServo().setPosition(0);
    }

    private void touchDown() {
        getExtendLift2().setBrake(false);
        getExtendLift().setBrake(false);

        getExtendLift().getMotor().setPower(0.1);
        getExtendLift2().getMotor().setPower(0.1);

        sleep(1000);
        driveWith(0.5, -10, 10, 10, -10);
        drive(10, 1);
        driveWith(0.5, 10, -10, -10, 10);
        drive(10, 1);
    }

    public void drive(int distance, double speed) {
        MotorEncoderController.getInstance().drive(getLeftMotorUp(), -distance, speed, false);
        MotorEncoderController.getInstance().drive(getRightMotorUp(), distance, speed, false);
        MotorEncoderController.getInstance().drive(getLeftMotorDown(), -distance, speed, false);
        MotorEncoderController.getInstance().drive(getRightMotorDown(), distance, speed, false);
        MotorEncoderController.getInstance().update("m1", "m2", "m3", "m4");
    }

    public void driveWith(double speed, int distanceLeftUp, int distanceRightUp, int distanceLeftDown, int distanceRightDown) {
        MotorEncoderController.getInstance().drive(getLeftMotorUp(), distanceLeftUp, speed, false);
        MotorEncoderController.getInstance().drive(getRightMotorUp(), distanceRightUp, speed, false);
        MotorEncoderController.getInstance().drive(getLeftMotorDown(), distanceLeftDown, speed, false);
        MotorEncoderController.getInstance().drive(getRightMotorDown(), distanceRightDown, speed, false);
        MotorEncoderController.getInstance().update("m1", "m2", "m3", "m4");
    }

}
