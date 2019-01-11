package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MotorEncoderController;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Mode1", group = "tests")
public class Mode1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.getInstance().init(hardwareMap);

        waitForStart();

        Robot.getInstance().runObjectDetection(Robot.CameraOrientation.PORTRAIT, 2, new Robot.ObjectDetected() {
            @Override
            public void onLeft() {
                // TODO: 1/11/2019 Turn left and forward or go sideways towards left
            }

            @Override
            public void onCenter() {
                MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 10, 1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorUp(), 10, -1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorDown(), 10, 1.0, false);
                MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorDown(), 10, -1.0, false);
                MotorEncoderController.getInstance().update("*");
            }

            @Override
            public void onRight() {
                // TODO: 1/11/2019 Turn right and forward or go sideways towards right
            }
        });

        MotorEncoderController.disable();
        Robot.disable();
    }

}
