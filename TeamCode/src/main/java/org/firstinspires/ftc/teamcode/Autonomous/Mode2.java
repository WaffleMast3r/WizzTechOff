package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.EasyRobot;
import org.firstinspires.ftc.teamcode.MotorEncoderController;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Mode2", group = "tests")
public class Mode2 extends EasyRobot {

    @Override
    public void runOpMode() throws InterruptedException {

        init();
        waitForStart();

        MotorEncoderController.getInstance().drive(getLeftMotorUp(), 10, -0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorUp(), 10, 0.5, false);
        MotorEncoderController.getInstance().drive(getLeftMotorDown(), 10, -0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorDown(), 10, 0.5, false);
        MotorEncoderController.getInstance().update("*");


        MotorEncoderController.disable();
        Robot.disable();
    }

}
