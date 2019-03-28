package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.EasyRobot;
import org.firstinspires.ftc.teamcode.MotorEncoderController;


@TeleOp(name = "(nu asta) TeleOp - Bucuresti", group = "Official")
public class Mode3 extends EasyRobot {


    @Override
    public void runOpMode() throws InterruptedException {
        initRobot(this);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                getLeftMotorUp().getMotor().setPower(-0.1);
                getRightMotorUp().getMotor().setPower(0.1);
                getLeftMotorDown().getMotor().setPower(0.1);
                getRightMotorDown().getMotor().setPower(-0.1);
            } else {
                getLeftMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                getRightMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                getLeftMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                getRightMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
            }

            if (gamepad1.right_trigger > 0) {
                if (getLandMotor().getMotor().getCurrentPosition() >= -2300) {
                    getLandMotor().getMotor().setPower(-gamepad1.right_trigger);
                } else {
                    getLandMotor().getMotor().setPower(0);
                }
            } else if (gamepad1.left_trigger > 0) {
                if (getLandMotor().getMotor().getCurrentPosition() <= 0) {
                    getLandMotor().getMotor().setPower(gamepad1.left_trigger);
                } else {
                    getLandMotor().getMotor().setPower(0);
                }
            } else {
                getLandMotor().getMotor().setPower(0);
            }

            if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.a) {

                driveTicks(getAxLift(), 0, 1, true, false);
                driveTicks(getExtendLift(), 500, 0.3, true, false);

                while (areBusyMotors(getAxLift().getName(), getExtendLift().getName())) {
                    getLeftMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getRightMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getLeftMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getRightMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                }

//                waitForMotors(getAxLift().getName(), getExtendLift().getName());

                getAxLift().setBrake(true);
                getExtendLift().setBrake(true);
                while (opModeIsActive()) {
                }

                getExtendLift().setBrake(false);
                getAxLift().setBrake(false);

                driveTicks(getAxLift(), 1240, 0.1, true, false);
                waitForMotors(getAxLift().getName());

                disable();
                MotorEncoderController.disable();
            } else if (gamepad1.left_bumper) {
                driveTicks(getAxLift(), 1240, 1, true, false);
                getAxLift().getMotor().setPower(0);
            } else if (gamepad1.right_bumper) {
                driveTicks(getExtendLift(), 0, 1, true, false);
                getExtendLift().getMotor().setPower(0);
            }


            if (gamepad2.y) {
                if (gamepad2.left_bumper && gamepad2.right_bumper) {
                    getLiftServo1().setPosition(0);
                    getLiftServo2().setPosition(1);
                } else if (gamepad2.right_bumper) {
                    getLiftServo1().setPosition(0);
                } else if (gamepad2.left_bumper) {
                    getLiftServo2().setPosition(1);
                } else {
                    getLiftServo1().setPosition(1);
                    getLiftServo2().setPosition(0);
                }
            }


            if (gamepad2.left_bumper) {
                getCollectorServo().setPower(1);
            } else if (gamepad2.right_bumper) {
                getCollectorServo().setPower(-1);
            } else {
                getCollectorServo().setPower(0);
            }

            if (gamepad2.dpad_left) {
                getCollectorRotateServo().setPosition(0);
            } else if (gamepad2.dpad_up) {
                getCollectorRotateServo().setPosition(0.5);
            } else if (gamepad2.dpad_right) {
                getCollectorRotateServo().setPosition(1);
            }

        }
    }
}