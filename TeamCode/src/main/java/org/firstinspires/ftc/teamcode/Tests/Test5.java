package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EasyRobot;

@Autonomous(name = "Test 5", group = "tests")
public class Test5 extends EasyRobot {

    @Override
    public void runOpMode() throws InterruptedException {
        init(this);

        waitForStart();

        runObjectDetection(CameraOrientation.PORTRAIT, 2, new ObjectDetected() {
            @Override
            public void onLeft(int... values) {
                
            }

            @Override
            public void onCenter(int... values) {

            }

            @Override
            public void onRight(int... values) {

            }
        });

        disable();
    }
}
