package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.EasyRobot;


@TeleOp(name = "TeleOp - Nationala", group = "Official")
public class TeleOpMode4 extends EasyRobot {

    boolean classic = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot(this);

        waitForStart();

        while (!gamepad1.dpad_left && !gamepad1.dpad_right && opModeIsActive()) {
            classic = gamepad1.dpad_left;
        }

        while (opModeIsActive()) {

            //Driver 1
            if (classic) {      //Mod clasic
                if (gamepad1.x) {
                    getLeftMotorUp().getMotor().setPower(-0.1);     //-1
                    getRightMotorUp().getMotor().setPower(0.1);     //1
                    getLeftMotorDown().getMotor().setPower(0.1);    //1
                    getRightMotorDown().getMotor().setPower(-0.1);  //-1
                } else {
                    getLeftMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getRightMotorUp().getMotor().setPower(Range.clip(-gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getLeftMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x + gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                    getRightMotorDown().getMotor().setPower(Range.clip(gamepad1.left_stick_x - gamepad1.left_stick_y - gamepad1.right_stick_x, -1, 1));
                }

                if (gamepad1.right_trigger > 0) {
                    getLandMotor().getMotor().setPower(-gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > 0) {
                    getLandMotor().getMotor().setPower(gamepad1.left_trigger);
                } else {
                    getLandMotor().getMotor().setPower(0);
                }

            } else {            //Mod nou
                if (gamepad1.right_trigger > 0) {
                    getLeftMotorUp().getMotor().setPower(-gamepad1.right_trigger);
                    getRightMotorUp().getMotor().setPower(gamepad1.right_trigger);
                    getLeftMotorDown().getMotor().setPower(-gamepad1.right_trigger);
                    getRightMotorDown().getMotor().setPower(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > 0) {
                    getLeftMotorUp().getMotor().setPower(gamepad1.right_trigger);
                    getRightMotorUp().getMotor().setPower(-gamepad1.right_trigger);
                    getLeftMotorDown().getMotor().setPower(gamepad1.right_trigger);
                    getRightMotorDown().getMotor().setPower(-gamepad1.right_trigger);
                }

                if (gamepad1.right_bumper) {
                    getLeftMotorUp().getMotor().setPower(-1);
                    getRightMotorUp().getMotor().setPower(-1);
                    getLeftMotorDown().getMotor().setPower(-1);
                    getRightMotorDown().getMotor().setPower(-1);
                } else if (gamepad1.left_bumper) {
                    getLeftMotorUp().getMotor().setPower(1);
                    getRightMotorUp().getMotor().setPower(1);
                    getLeftMotorDown().getMotor().setPower(1);
                    getRightMotorDown().getMotor().setPower(1);
                }

                if (gamepad1.dpad_left) {
                    getLeftMotorUp().getMotor().setPower(1);
                    getRightMotorUp().getMotor().setPower(1);
                    getLeftMotorDown().getMotor().setPower(-1);
                    getRightMotorDown().getMotor().setPower(-1);
                } else if (gamepad1.dpad_right) {
                    getLeftMotorUp().getMotor().setPower(-1);
                    getRightMotorUp().getMotor().setPower(-1);
                    getLeftMotorDown().getMotor().setPower(1);
                    getRightMotorDown().getMotor().setPower(1);
                }

                if (gamepad1.dpad_up) {
                    getLandMotor().getMotor().setPower(-1);
                } else if (gamepad1.dpad_down) {
                    getLandMotor().getMotor().setPower(1);
                } else {
                    getLandMotor().getMotor().setPower(0);
                }
            }

            //Driver 2
            if (gamepad2.a) {
                getCollectorServo().setPower(1);
            } else if (gamepad2.b) {
                getCollectorServo().setPower(-1);
            } else {
                getCollectorServo().setPower(0);
            }


//            getExtendLift().getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
            if (gamepad2.right_bumper) {
                if (getExtendLift().getMotor().getCurrentPosition() < 1200) {
                    getExtendLift().getMotor().setPower(1);
                } else {
                    getExtendLift().getMotor().setPower(0);
                }
            } else if (gamepad2.left_bumper) {
                if (getExtendLift().getMotor().getCurrentPosition() > 0) {
                    getExtendLift().getMotor().setPower(-1);
                } else {
                    getExtendLift().getMotor().setPower(0);
                }
            } else getExtendLift().getMotor().setPower(0);
            telemetry.addData("1", "extendlft %d", getExtendLift().getMotor().getCurrentPosition());
            telemetry.update();
            //Axlift este motorul care roteste bratul ce include sortatorul
            getAxLift().getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
//            if (gamepad2.dpad_down) {       //TODO: After final, make the brakes work
////                getAxLift().setBrake(false);
//                driveTicks(getAxLift(), 0, 0.5, true, false);
//                //getAxLift().setBrake(true);
//                getAxLift().getMotor().setPower(0);
//            } else if (gamepad2.dpad_up) {
////                getAxLift().setBrake(false);
//                driveTicks(getAxLift(), -221, 1, true, false);
//                //getAxLift().setBrake(true);
//                getAxLift().getMotor().setPower(0);
//            } else if (gamepad2.y){
////                getAxLift().setBrake(false);
//                driveTicks(getAxLift(), -371, 1, true, false);
//                //getAxLift().setBrake(true);
//                getAxLift().getMotor().setPower(0);
//            }
            if(gamepad2.dpad_down)
                getAxLift().getMotor().setPower(-1);
            else if(gamepad2.dpad_up)
                getAxLift().getMotor().setPower(1);
            else
                getAxLift().getMotor().setPower(0);
        }
    }
}
