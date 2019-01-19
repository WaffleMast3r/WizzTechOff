package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MotorEncoderController;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Tele Op V1", group = "Official")
public class Mode1 extends LinearOpMode {

    public final double SERVO_POS_1 = 1, SERVO_POS_2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        boolean frana = false;

        Robot.getInstance().init(this);

        waitForStart();

        while (opModeIsActive()) {

            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                Robot.getInstance().getLiftMotor().getMotor().setPower(gamepad2.left_stick_y);
            } else {
                Robot.getInstance().getLiftMotor().getMotor().setPower(0.01);
            }

            if (gamepad2.a) {
                Robot.getInstance().getCollectorPivotServo().setPosition(SERVO_POS_1);
            } else {
                Robot.getInstance().getCollectorPivotServo().setPosition(SERVO_POS_2);
            }

            if (gamepad1.x || gamepad1.dpad_left) {
                Robot.getInstance().getLeftMotorUp().getMotor().setPower(-0.1);
                Robot.getInstance().getRightMotorUp().getMotor().setPower(0.1);
                Robot.getInstance().getLeftMotorDown().getMotor().setPower(0.1);
                Robot.getInstance().getRightMotorDown().getMotor().setPower(-0.1);
                frana = true;
            } else {
                frana = false;
            }

            if (!frana) {
                if (gamepad1.left_bumper && gamepad1.right_bumper) {
                    Robot.getInstance().getLeftMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 6);
                    Robot.getInstance().getRightMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 6);
                    Robot.getInstance().getLeftMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 6);
                    Robot.getInstance().getRightMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 6);
                } else if (gamepad1.left_bumper) {
                    Robot.getInstance().getLeftMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 4);
                    Robot.getInstance().getRightMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 4);
                    Robot.getInstance().getLeftMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 4);
                    Robot.getInstance().getRightMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 4);
                } else if (gamepad1.right_bumper) {
                    Robot.getInstance().getLeftMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    Robot.getInstance().getRightMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    Robot.getInstance().getLeftMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    Robot.getInstance().getRightMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                } else {
                    Robot.getInstance().getLeftMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 2);
                    Robot.getInstance().getRightMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 2);
                    Robot.getInstance().getLeftMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 2);
                    Robot.getInstance().getRightMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 2);
                }
            }
        }
        Robot.disable();
        MotorEncoderController.disable();
    }
}
