package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Gyro")
public class Test4 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot.getInstance().init(this);

        waitForStart();

        while (opModeIsActive()) {
            double angle = Robot.getInstance().getAngle(Robot.Axis.Z) < 0 ? 360 + Robot.getInstance().getAngle(Robot.Axis.Z) : Robot.getInstance().getAngle(Robot.Axis.Z);

            telemetry.addData("Z", angle);
            //telemetry.addData("X", Robot.getInstance().getAngle(Robot.Axis.X));
            //telemetry.addData("Y", Robot.getInstance().getAngle(Robot.Axis.Y));
            telemetry.update();
        }
        Robot.disable();
    }
}
