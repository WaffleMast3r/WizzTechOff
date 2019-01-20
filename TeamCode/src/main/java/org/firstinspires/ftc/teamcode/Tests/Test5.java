package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EasyRobot;
import org.firstinspires.ftc.teamcode.MotorEncoderController;

@Autonomous(name = "Test 5", group = "tests")
public class Test5 extends EasyRobot {

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot(this);

        waitForStart();

        MotorEncoderController.getInstance().drive(getLeftMotorUp(), -10, 0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorUp(), -10, 0.5, false);
        MotorEncoderController.getInstance().drive(getLeftMotorDown(), 10, 0.5, false);
        MotorEncoderController.getInstance().drive(getRightMotorDown(), 10, 0.5, false);
        MotorEncoderController.getInstance().update("m1","m2","m3","m4");
        sleep(4000);
        MotorEncoderController.disable();
        disable();
    }
}
