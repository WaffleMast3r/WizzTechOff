package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MotorEncoderController;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Tele Op V2", group = "Official")
public class Mode2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot.getInstance().init(hardwareMap);

        double angle;
        double left_stick_x, left_stick_y, right_stick_x;

        waitForStart();

        while (Robot.getInstance().getSide().equals(Robot.TeamSide.UNKNOWN)) {
            telemetry.addData(">", "Please press < or > to define your side");
            if (gamepad1.dpad_left) Robot.getInstance().setSide(Robot.TeamSide.LEFT);
            else if (gamepad1.dpad_right) Robot.getInstance().setSide(Robot.TeamSide.RIGHT);
        }

        int offset = Robot.getInstance().getSide().getAngle();

        while (opModeIsActive()) {
//            angle = (Robot.getInstance().getAngle(Robot.Axis.Z) < 0 ? 360 + Robot.getInstance().getAngle(Robot.Axis.Z) : Robot.getInstance().getAngle(Robot.Axis.Z)) + offset;
            angle = Robot.getInstance().getAngle(Robot.Axis.Z);
            if (angle < 0) {
                angle = 360 + Robot.getInstance().getAngle(Robot.Axis.Z);
            } else {
                angle = Robot.getInstance().getAngle(Robot.Axis.Z);
            }
            angle -= offset;
            left_stick_x = Math.cos(angle) * gamepad1.left_stick_x;
            left_stick_y = Math.sin(angle) * gamepad1.left_stick_y;
            right_stick_x = gamepad1.right_stick_x;
            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                Robot.getInstance().getLeftMotorUp().getMotor().setPower(Range.clip(-left_stick_x + left_stick_y - right_stick_x, -1, 1) / 6);
                Robot.getInstance().getRightMotorUp().getMotor().setPower(Range.clip(-left_stick_x - left_stick_y - right_stick_x, -1, 1) / 6);
                Robot.getInstance().getLeftMotorDown().getMotor().setPower(Range.clip(left_stick_x + left_stick_y - right_stick_x, -1, 1) / 6);
                Robot.getInstance().getRightMotorDown().getMotor().setPower(Range.clip(left_stick_x - left_stick_y - right_stick_x, -1, 1) / 6);
            } else if (gamepad1.left_bumper) {
                Robot.getInstance().getLeftMotorUp().getMotor().setPower(Range.clip(-left_stick_x + left_stick_y - right_stick_x, -1, 1) / 4);
                Robot.getInstance().getRightMotorUp().getMotor().setPower(Range.clip(-left_stick_x - left_stick_y - right_stick_x, -1, 1) / 4);
                Robot.getInstance().getLeftMotorDown().getMotor().setPower(Range.clip(left_stick_x + left_stick_y - right_stick_x, -1, 1) / 4);
                Robot.getInstance().getRightMotorDown().getMotor().setPower(Range.clip(left_stick_x - left_stick_y - right_stick_x, -1, 1) / 4);
            } else if (gamepad1.right_bumper) {
                Robot.getInstance().getLeftMotorUp().getMotor().setPower(Range.clip(-left_stick_x + left_stick_y - right_stick_x, -1, 1));
                Robot.getInstance().getRightMotorUp().getMotor().setPower(Range.clip(-left_stick_x - left_stick_y - right_stick_x, -1, 1));
                Robot.getInstance().getLeftMotorDown().getMotor().setPower(Range.clip(left_stick_x + left_stick_y - right_stick_x, -1, 1));
                Robot.getInstance().getRightMotorDown().getMotor().setPower(Range.clip(left_stick_x - left_stick_y - right_stick_x, -1, 1));
            } else {
                Robot.getInstance().getLeftMotorUp().getMotor().setPower(Range.clip(-left_stick_x + left_stick_y - right_stick_x, -1, 1) / 2);
                Robot.getInstance().getRightMotorUp().getMotor().setPower(Range.clip(-left_stick_x - left_stick_y - right_stick_x, -1, 1) / 2);
                Robot.getInstance().getLeftMotorDown().getMotor().setPower(Range.clip(left_stick_x + left_stick_y - right_stick_x, -1, 1) / 2);
                Robot.getInstance().getRightMotorDown().getMotor().setPower(Range.clip(left_stick_x - left_stick_y - right_stick_x, -1, 1) / 2);
            }
        }
        Robot.disable();
        MotorEncoderController.disable();
    }
}