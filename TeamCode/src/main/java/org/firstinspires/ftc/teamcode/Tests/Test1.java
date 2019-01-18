package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MotorEncoderController;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Test1", group = "tests")
public class Test1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //am facut o clasa in care initializam robotul si vreau acum sa modific ceva la ea
        Robot.getInstance().init(this);

        waitForStart();

        // daca update = true; si nu avem parametrii ca in exemplul urmator atunci o sa se updateze doar motorul curent
        MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 100, 1.0, false);
        MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorUp(), 200, 1.0, false);
        MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorDown(), 300, 1.0, false);
        MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorDown(), 400, 1.0, false);
        MotorEncoderController.getInstance().update("m1", "m2", "m3", "m4");
        Robot.getInstance().getCollectorServo().setPower(1.0);
        sleep(20000);
        Robot.getInstance().getCollectorServo().setPower(0.0);
//        // daca update = true; atunci o sa updateze motoarele care pot fii date ca parametrii in continuare (m1 este numele definit in robot configuration)
//        MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 1, 0.5, true, "m1");
//        MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 1, 0.6, true, "m1", "m2", "m3", "m4");
//        //daca am dat mai multe comenzi pe mai multe motoare si dorim sa porneasca toate in acelasi timp putem sa setam update = false; iar la final sa le updatam:
//        MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 1, 0.5, false); // m1
//        MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorDown(), 1, 0.5, false);    //m2
//        MotorEncoderController.getInstance().drive(Robot.getInstance().getRightMotorUp(), 1, 0.5, false);    //m3
//        MotorEncoderController.getInstance().drive(Robot.getInstance().getLeftMotorUp(), 1, 0.5, false); // evident putem sa updatam aici adica sa setam update = true; si sa punem * sau all sau numele motoarelor
//        MotorEncoderController.getInstance().update("*"); // putem sa folosim simbolul * sau keyword-ul all pentru a updata toate actiunile inregistrate precedent
//        MotorEncoderController.getInstance().update("all");
//        MotorEncoderController.getInstance().update("m1", "m2", "m3"); // sau putem sa precizam numele motoarelor care dorim sa fie updatate

        Robot.disable();
        MotorEncoderController.disable();
    }
}
