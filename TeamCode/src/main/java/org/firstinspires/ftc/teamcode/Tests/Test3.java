package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.EasyRobot;
import org.firstinspires.ftc.teamcode.MotorEncoderController;

@Autonomous(name = "Test 3", group = "tests")
public class Test3 extends EasyRobot {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot(this);

        getExtendLift2().setBrake(true);
        getExtendLift().setBrake(true);
        getCollectorRotateServo().setPosition(0);

        waitForStart();

        MotorEncoderController.disable();
        disable();
    }
}
