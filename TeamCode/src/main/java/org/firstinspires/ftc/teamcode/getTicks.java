package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Ticks", group = "Official")
public class getTicks extends LinearOpMode {

    DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("m1");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Ticks", motor.getCurrentPosition());
            telemetry.update();
        }

    }
}
