package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Ticks", group = "Official")
public class getTicks extends EasyRobot {


    @Override
    public void runOpMode() throws InterruptedException {
        getAxLift().getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getAxLift().getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Ticks", getAxLift().getMotor().getCurrentPosition());
            telemetry.update();
            if (gamepad2.dpad_down) {
                getAxLift().setBrake(false);
                driveTicks(getAxLift(), 0, 1.0, true, false);
                getAxLift().setBrake(true);
            } else if (gamepad2.dpad_up) {
                getAxLift().setBrake(false);
                driveTicks(getAxLift(), -221, 1.0, true, false);
                getAxLift().setBrake(true);
            } else if (gamepad2.y){
                getAxLift().setBrake(false);
                driveTicks(getAxLift(), -371, 1.0, true, false);
                getAxLift().setBrake(true);
            }
        }

    }
}
