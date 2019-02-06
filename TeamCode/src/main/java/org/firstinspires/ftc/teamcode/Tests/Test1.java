package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.EasyRobot;
import org.firstinspires.ftc.teamcode.MotorEncoderController;

@Autonomous(name = "Test 1", group = "tests")
public class Test1 extends EasyRobot {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot(this);

        waitForStart();


        idle();

        MotorEncoderController.disable();
        disable();
    }

    public void drive(int distance, double speed) {
        MotorEncoderController.getInstance().drive(getLeftMotorUp(), -distance, speed, false);
        MotorEncoderController.getInstance().drive(getRightMotorUp(), distance, speed, false);
        MotorEncoderController.getInstance().drive(getLeftMotorDown(), -distance, speed, false);
        MotorEncoderController.getInstance().drive(getRightMotorDown(), distance, speed, false);
        MotorEncoderController.getInstance().update("m1", "m2", "m3", "m4");
    }

}
