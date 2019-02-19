package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MotorEncoderController;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Mode1", group = "tests")
@Disabled  
public class Mode1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.getInstance().init(this);

        waitForStart();

        Robot.getInstance().runObjectDetection(Robot.CameraOrientation.PORTRAIT, 2, new Robot.ObjectDetected() {
            @Override
            public void onLeft(int... values) {
                // TODO: 1/11/2019 Turn left and forward or go sideways towards left
                telemetry.addData("Found", "Left (" + values[0] + ", " + values[1] + ", " + values[2] + ")");
                telemetry.update();
                sleep(10000);
            }

            @Override
            public void onCenter(int... values) {
//                MotorEncoderController.getInstance().driveForward(Robot.getInstance().getLeftMotorUp(), 10, 1.0, false);
//                MotorEncoderController.getInstance().driveForward(Robot.getInstance().getRightMotorUp(), 10, -1.0, false);
//                MotorEncoderController.getInstance().driveForward(Robot.getInstance().getLeftMotorDown(), 10, 1.0, false);
//                MotorEncoderController.getInstance().driveForward(Robot.getInstance().getRightMotorDown(), 10, -1.0, false);
//                MotorEncoderController.getInstance().update("*");

                telemetry.addData("Found", "Center (" + values[0] + ", " + values[1] + ", " + values[2] + ")");
                telemetry.update();
                sleep(10000);
            }

            @Override
            public void onRight(int... values) {
                // TODO: 1/11/2019 Turn right and forward or go sideways towards right
                telemetry.addData("Found", "Right (" + values[0] + ", " + values[1] + ", " + values[2] + ")");
                telemetry.update();
                sleep(10000);
            }
        });

        telemetry.addData("Finished", "Done");
        telemetry.update();

        sleep(10000);

        MotorEncoderController.disable();
        Robot.disable();
    }

}
