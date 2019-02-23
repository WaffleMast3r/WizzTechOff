package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.EasyRobot;
import org.firstinspires.ftc.teamcode.MotorEncoderController;


@TeleOp(name = "TeleOp - Bucuresti", group = "Official")
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

            if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.a) {

                driveTicks(getExtendLift(), 0, 1, true, false);
                driveTicks(getExtendLift2(), 500, 0.3, true, false);

                while (areBusyMotors(getExtendLift().getName(), getExtendLift2().getName())) {
                    getLeftMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getRightMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getLeftMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getRightMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                }

//                waitForMotors(getExtendLift().getName(), getExtendLift2().getName());

                getExtendLift().setBrake(true);
                getExtendLift2().setBrake(true);
                while (opModeIsActive()) {
                }

                getExtendLift2().setBrake(false);
                getExtendLift().setBrake(false);

                driveTicks(getExtendLift(), 1240, 0.1, true, false);
                waitForMotors(getExtendLift().getName());

                disable();
                MotorEncoderController.disable();
            } else if (gamepad1.left_bumper) {
                driveTicks(getExtendLift(), 1240, 1, true, false);
                getExtendLift().getMotor().setPower(0);
            } else if (gamepad1.right_bumper) {
                driveTicks(getExtendLift2(), 0, 1, true, false);
                getExtendLift2().getMotor().setPower(0);
            }

            if (gamepad2.y) {
                //sus
                getLiftServo1().setPosition(0);
                getLiftServo2().setPosition(0);
                getSortatorServo1().setPosition(0);
                getSortatorServo2().setPosition(0);
            } else if (gamepad2.a) {
                getLiftServo1().setPosition(1);
                getLiftServo2().setPosition(1);
                getSortatorServo1().setPosition(1);
                getSortatorServo2().setPosition(1);
            } else if (gamepad2.x) {
                // nu stiu
                getLiftServo1().setPosition(0.5);
                getLiftServo1().setPosition(0.5);
                getSortatorServo1().setPosition(0);
                getSortatorServo2().setPosition(0);
                sleep( 1000);
                getLiftServo1().setPosition(0);
                getLiftServo2().setPosition(0);


            } else if (gamepad2.b) {
                //goleste
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
