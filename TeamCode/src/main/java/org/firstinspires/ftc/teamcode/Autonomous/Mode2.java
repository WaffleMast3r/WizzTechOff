package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.EasyRobot;
import org.firstinspires.ftc.teamcode.MotorEncoderController;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Autonom Demo", group = "Official")
@Disabled
public class Mode2 extends EasyRobot {

    @Override
    public void runOpMode() throws InterruptedException {

          initRobot(this);
        initGyro(BNO055IMU.AngleUnit.DEGREES);
        getHandMotor().getMotor().setPower(0.1f);
        initVuforia();
//        initTfod();

        //getExtendLift2().getMotor().setPower(0.01);

        waitForStart();

        getHandMotor().getMotor().setPower(1.0f);
        getHandMotor().getMotor().setPower(0.0f);

        sleep(5000);
        getHandMotor().getMotor().setPower(0.0f);

        //Drive stanga
        MotorEncoderController.getInstance().drive(getLeftMotorUp(), -15, 0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorUp(), -15, 0.5, false);
        MotorEncoderController.getInstance().drive(getLeftMotorDown(), 15, 0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorDown(), 15, 0.5, false);
        MotorEncoderController.getInstance().update("m1","m2","m3","m4");
        sleep(4000);

        //Drive putin inainte
        MotorEncoderController.getInstance().drive(getLeftMotorUp(), -10, 0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorUp(), 10, 0.5, false);
        MotorEncoderController.getInstance().drive(getLeftMotorDown(), -10, 0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorDown(), 10, 0.5, false);
        MotorEncoderController.getInstance().update("m1","m2","m3","m4");
        sleep(3000);

        //Drive dreapta
        MotorEncoderController.getInstance().drive(getLeftMotorUp(), 15, 0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorUp(), 15, 0.5, false);
        MotorEncoderController.getInstance().drive(getLeftMotorDown(), -15, 0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorDown(), -15, 0.5, false);
        MotorEncoderController.getInstance().update("m1","m2","m3","m4");
        sleep(4000);


        //Drive putin inapoi
        MotorEncoderController.getInstance().drive(getLeftMotorUp(), 10, 0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorUp(), -10, 0.5, false);
        MotorEncoderController.getInstance().drive(getLeftMotorDown(), 10, 0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorDown(), -10, 0.5, false);
        MotorEncoderController.getInstance().update("m1","m2","m3","m4");
        sleep(3000);

        runObjectDetection(2, new ObjectDetected() {
            @Override
            public void pickup() {

            }

            @Override
            public void onLeft() {

            }

            @Override
            public void onCenter() {

            }

            @Override
            public void onRight() {

            }

            @Override
            public void loadCargo() {

            }

            public void onLeft(int... values) {
                MotorEncoderController.getInstance().drive(getLeftMotorUp(), 15, 1.0, false);
                MotorEncoderController.getInstance().drive(getRightMotorUp(), 15, -0.8, false);
                MotorEncoderController.getInstance().drive(getLeftMotorDown(), 15, 1.0, false);
                MotorEncoderController.getInstance().drive(getRightMotorDown(), 15, -0.8, false);
                MotorEncoderController.getInstance().update("m1","m2","m3","m4");

                telemetry.addData("Found", "Left (" + values[0] + ", " + values[1] + ", " + values[2] + ")");
                telemetry.update();
                sleep(8000);

                MotorEncoderController.getInstance().drive(getLeftMotorUp(), 50, 0.9, false);
                MotorEncoderController.getInstance().drive(getRightMotorUp(), 50, -1.0, false);
                MotorEncoderController.getInstance().drive(getLeftMotorDown(), 50, 0.9, false);
                MotorEncoderController.getInstance().drive(getRightMotorDown(), 50, -1.0, false);
                MotorEncoderController.getInstance().update("m1","m2","m3","m4");
            }

            public void onCenter(int... values) {

                //Drive inainte
                MotorEncoderController.getInstance().drive(getLeftMotorUp(), -50, 0.5, false);
                MotorEncoderController.getInstance().drive(getRightMotorUp(), 50, 0.5, false);
                MotorEncoderController.getInstance().drive(getLeftMotorDown(), -50, 0.5, false);
                MotorEncoderController.getInstance().drive(getRightMotorDown(), 50, 0.5, false);
                MotorEncoderController.getInstance().update("m1","m2","m3","m4");
                sleep(3000);

                telemetry.addData("Found", "Center (" + values[0] + ", " + values[1] + ", " + values[2] + ")");
                telemetry.update();
                sleep(8000);

                MotorEncoderController.getInstance().drive(getLeftMotorUp(), 50, 1.0, false);
                MotorEncoderController.getInstance().drive(getRightMotorUp(), 50, -1.0, false);
                MotorEncoderController.getInstance().drive(getLeftMotorDown(), 50, 1.0, false);
                MotorEncoderController.getInstance().drive(getRightMotorDown(), 50, -1.0, false);
                MotorEncoderController.getInstance().update("m1","m2","m3","m4");
            }

            public void onRight(int... values) {
                MotorEncoderController.getInstance().drive(getLeftMotorUp(), 10, 0.8, false);
                MotorEncoderController.getInstance().drive(getRightMotorUp(), 10, -1.0, false);
                MotorEncoderController.getInstance().drive(getLeftMotorDown(), 10, 0.8, false);
                MotorEncoderController.getInstance().drive(getRightMotorDown(), 10, -1.0, false);
                MotorEncoderController.getInstance().update("m1","m2","m3","m4");

                telemetry.addData("Found", "Right (" + values[0] + ", " + values[1] + ", " + values[2] + ")");
                telemetry.update();
                sleep(8000);

                MotorEncoderController.getInstance().drive(getLeftMotorUp(), 10, 1.0, false);
                MotorEncoderController.getInstance().drive(getRightMotorUp(), 10, -0.9, false);
                MotorEncoderController.getInstance().drive(getLeftMotorDown(), 10, 1.0, false);
                MotorEncoderController.getInstance().drive(getRightMotorDown(), 10, -0.9, false);
                MotorEncoderController.getInstance().update("m1","m2","m3","m4");
            }
        });

        sleep(7000);
//
//        //Drop relic
//
        MotorEncoderController.disable();
        Robot.disable();
    }

}
