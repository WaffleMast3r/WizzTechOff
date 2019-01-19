package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.EasyRobot;
import org.firstinspires.ftc.teamcode.MotorEncoderController;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Autonom Demo", group = "Official")
public class Mode2 extends EasyRobot {

    @Override
    public void runOpMode() throws InterruptedException {

        initRobot(this);
        initGyro(BNO055IMU.AngleUnit.DEGREES);
        getExtendLift().getMotor().setPower(0.01);
        initVuforia();

        waitForStart();

        getExtendLift().getMotor().setPower(0);

        //Drive stanga
        MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 4, -0.5, false);
        MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorUp(), 4, -0.5, false);
        MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorDown(), 4, 0.5, false);
        MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorDown(), 4, 0.5, false);
        MotorEncoderController.getInstance().update("*");
        sleep(4000);

        //Drive putin inainte
        MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 3, 0.5, false);
        MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorUp(), 3, -0.5, false);
        MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorDown(), 3, 0.5, false);
        MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorDown(), 3, -0.5, false);
        MotorEncoderController.getInstance().update("*");
        sleep(2000);

        //Drive dreapta
        MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 4, 0.5, false);
        MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorUp(), 4, 0.5, false);
        MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorDown(), 4, -0.5, false);
        MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorDown(), 4, -0.5, false);
        MotorEncoderController.getInstance().update("*");
        sleep(4000);

        runObjectDetection(CameraOrientation.PORTRAIT, 2, new ObjectDetected() {
            @Override
            public void onLeft(int... values) {
                MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 15, 1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorUp(), 15, -0.7, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorDown(), 15, 1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorDown(), 15, -0.7, false);
                MotorEncoderController.getInstance().update("*");

                telemetry.addData("Found", "Left (" + values[0] + ", " + values[1] + ", " + values[2] + ")");
                telemetry.update();
                sleep(8000);

                MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 10, 0.9, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorUp(), 10, -1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorDown(), 10, 0.9, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorDown(), 10, -1.0, false);
                MotorEncoderController.getInstance().update("*");
            }

            @Override
            public void onCenter(int... values) {
                MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 10, 1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorUp(), 10, -1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorDown(), 10, 1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorDown(), 10, -1.0, false);
                MotorEncoderController.getInstance().update("*");

                telemetry.addData("Found", "Center (" + values[0] + ", " + values[1] + ", " + values[2] + ")");
                telemetry.update();
                sleep(8000);

                MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 7, 1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorUp(), 7, -1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorDown(), 7, 1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorDown(), 7, -1.0, false);
                MotorEncoderController.getInstance().update("*");
            }

            @Override
            public void onRight(int... values) {
                MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 10, 0.7, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorUp(), 10, -1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorDown(), 10, 0.7, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorDown(), 10, -1.0, false);
                MotorEncoderController.getInstance().update("*");

                telemetry.addData("Found", "Right (" + values[0] + ", " + values[1] + ", " + values[2] + ")");
                telemetry.update();
                sleep(8000);

                MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 10, 1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorUp(), 10, -0.9, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorDown(), 10, 1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorDown(), 10, -0.9, false);
                MotorEncoderController.getInstance().update("*");
            }
        });

        sleep(7000);

        //Drop relic

        MotorEncoderController.disable();
        Robot.disable();
    }

}
