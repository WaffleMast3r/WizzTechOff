package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.EasyRobot;

@Autonomous(name = "Test 6")
@Disabled
public class Test6 extends EasyRobot {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot(this);
        initVuforia();
        initTrackable(2, new TrackableSettings());
//        initTfod();
        waitForStart();
        
        activateTrackable();

        while (opModeIsActive())
            getLocation().print();


        sleep(10000);
    }
}
