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
        initTfod();
        
        waitForStart();

        getExtendLift().getMotor().setPower(0);

        //Drive stanga
        MotorEncoderController.getInstance().drive(getLeftMotorUp(), 4, -0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorUp(), 4, -0.5, false);
        MotorEncoderController.getInstance().drive(getLeftMotorDown(), 4, 0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorDown(), 4, 0.5, false);
        MotorEncoderController.getInstance().update("m1","m2","m3","m4");
        sleep(4000);

        //Drive putin inainte
        MotorEncoderController.getInstance().drive(getLeftMotorUp(), 3, 0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorUp(), 3, -0.5, false);
        MotorEncoderController.getInstance().drive(getLeftMotorDown(), 3, 0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorDown(), 3, -0.5, false);
        MotorEncoderController.getInstance().update("m1","m2","m3","m4");
        sleep(2000);

        //Drive dreapta
        MotorEncoderController.getInstance().drive(getLeftMotorUp(), 4, 0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorUp(), 4, 0.5, false);
        MotorEncoderController.getInstance().drive(getLeftMotorDown(), 4, -0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorDown(), 4, -0.5, false);
        MotorEncoderController.getInstance().update("m1","m2","m3","m4");
        sleep(4000);

        runObjectDetection(CameraOrientation.PORTRAIT
                , 2, new ObjectDetected() {
            @Override
            public void onLeft(int... values) {
                MotorEncoderController.getInstance().drive(getLeftMotorUp(), 15, 1.0, false);
                MotorEncoderController.getInstance().drive(getRightMotorUp(), 15, -0.8, false);
                MotorEncoderController.getInstance().drive(getLeftMotorDown(), 15, 1.0, false);
                MotorEncoderController.getInstance().drive(getRightMotorDown(), 15, -0.8, false);
                MotorEncoderController.getInstance().update("m1","m2","m3","m4");

                telemetry.addData("Found", "Left (" + values[0] + ", " + values[1] + ", " + values[2] + ")");
                telemetry.update();
                sleep(8000);

                MotorEncoderController.getInstance().drive(getLeftMotorUp(), 10, 0.9, false);
                MotorEncoderController.getInstance().drive(getRightMotorUp(), 10, -1.0, false);
                MotorEncoderController.getInstance().drive(getLeftMotorDown(), 10, 0.9, false);
                MotorEncoderController.getInstance().drive(getRightMotorDown(), 10, -1.0, false);
                MotorEncoderController.getInstance().update("m1","m2","m3","m4");
            }

            @Override
            public void onCenter(int... values) {
                MotorEncoderController.getInstance().drive(getLeftMotorUp(), 10, 1.0, false);
                MotorEncoderController.getInstance().drive(getRightMotorUp(), 10, -1.0, false);
                MotorEncoderController.getInstance().drive(getLeftMotorDown(), 10, 1.0, false);
                MotorEncoderController.getInstance().drive(getRightMotorDown(), 10, -1.0, false);
                MotorEncoderController.getInstance().update("m1","m2","m3","m4");

                telemetry.addData("Found", "Center (" + values[0] + ", " + values[1] + ", " + values[2] + ")");
                telemetry.update();
                sleep(8000);

                MotorEncoderController.getInstance().drive(getLeftMotorUp(), 7, 1.0, false);
                MotorEncoderController.getInstance().drive(getRightMotorUp(), 7, -1.0, false);
                MotorEncoderController.getInstance().drive(getLeftMotorDown(), 7, 1.0, false);
                MotorEncoderController.getInstance().drive(getRightMotorDown(), 7, -1.0, false);
                MotorEncoderController.getInstance().update("m1","m2","m3","m4");
            }

            @Override
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

        //Drop relic

        MotorEncoderController.disable();
        Robot.disable();
    }

}
