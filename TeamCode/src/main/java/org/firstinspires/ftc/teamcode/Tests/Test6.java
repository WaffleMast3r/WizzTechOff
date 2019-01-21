package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.EasyRobot;

@Autonomous(name = "Test 6")
public class Test6 extends EasyRobot {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot(this);
        initVuforia();
        initTrackable(1, new TrackableSettings());
//        initTfod();
        waitForStart();
        while (opModeIsActive())
            getLocation().print();



        sleep(10000);
    }
}
