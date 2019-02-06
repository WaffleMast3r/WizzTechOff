package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.EasyRobot;
import org.firstinspires.ftc.teamcode.MotorEncoderController;


@TeleOp(name = "TeleOp - Bucuresti", group = "Official")
public class Mode3 extends EasyRobot {

    private final double SERVO_POS_1 = 0.6, SERVO_POS_2 = 0.5, SERVO_POS_3 = 0;
    int hand_sw = 0;

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
                if (gamepad1.left_bumper && gamepad1.right_bumper) {
                    getLeftMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 6);
                    getRightMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 6);
                    getLeftMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 6);
                    getRightMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 6);
                } else if (gamepad1.left_bumper) {
                    getLeftMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 4);
                    getRightMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 4);
                    getLeftMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 4);
                    getRightMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 4);
                } else if (gamepad1.right_bumper) {
                    getLeftMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getRightMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getLeftMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getRightMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                } else {
                    getLeftMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 2);
                    getRightMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 2);
                    getLeftMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 2);
                    getRightMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1) / 2);
                }
            }

            if (gamepad1.right_trigger > 0) {
                if (getHandMotor().getMotor().getCurrentPosition() >= -2300) {
                    getHandMotor().getMotor().setPower(-gamepad1.right_trigger);
                } else {
                    getHandMotor().getMotor().setPower(0);
                }
            } else if (gamepad1.left_trigger > 0) {
                if (getHandMotor().getMotor().getCurrentPosition() <= 0) {
                    getHandMotor().getMotor().setPower(gamepad1.left_trigger);
                } else {
                    getHandMotor().getMotor().setPower(0);
                }
            } else {
                getHandMotor().getMotor().setPower(0);
            }

            if (gamepad2.a) {
                if (gamepad2.left_bumper && gamepad2.right_bumper) {
                    getPaletaServo1().setPosition(0);
                    getPaletaServo2().setPosition(1);
                } else if (gamepad2.right_bumper) {
                    getPaletaServo1().setPosition(0);
                } else if (gamepad2.left_bumper) {
                    getPaletaServo2().setPosition(1);
                } else {
                    getPaletaServo1().setPosition(1);
                    getPaletaServo2().setPosition(0);
                }
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

            if (gamepad2.left_stick_y > 0) {
                if (getExtendLift2().getMotor().getCurrentPosition() >= 0) {
                    getExtendLift2().getMotor().setPower(gamepad2.left_stick_y);
                } else {
                    getExtendLift2().getMotor().setPower(0);
                }
            } else if (gamepad2.left_stick_y < 0) {
                if (getExtendLift().getMotor().getCurrentPosition() <= 1240) {
                    getExtendLift().getMotor().setPower(-gamepad2.left_stick_y);
                } else {
                    getExtendLift().getMotor().setPower(0);
                }
            } else {
                getExtendLift().getMotor().setPower(0);
                getExtendLift2().getMotor().setPower(0);
            }

            if (getExtendLift().getMotor().getCurrentPosition() > 0)
                getExtendLift().getMotor().setPower(-gamepad2.left_trigger);

            telemetry.addData("ExtendLift1", getExtendLift().getMotor().getCurrentPosition());
            telemetry.addData("ExtendLift2", getExtendLift2().getMotor().getCurrentPosition());
            telemetry.update();

            if (gamepad2.x) {
                getCollectorServo().setPower(1);
            } else if (gamepad2.b) {
                getCollectorServo().setPower(-1);
            } else {
                getCollectorServo().setPower(0);
            }

            if (gamepad2.dpad_down) {
                getLiftServo1().setPosition(0.5);
                getLiftServo2().setPosition(0.5);
            }

            if (gamepad2.dpad_left) {
                getCollectorRotateServo().setPosition(0);
            } else if (gamepad2.dpad_up) {
                getCollectorRotateServo().setPosition(0.5);
            } else if (gamepad2.dpad_right) {
                getCollectorRotateServo().setPosition(1);
            }

        }

        disable();
        MotorEncoderController.disable();
    }
}
