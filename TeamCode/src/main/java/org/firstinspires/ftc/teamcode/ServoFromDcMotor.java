package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public abstract class ServoFromDcMotor extends WizzTechDcMotor {

    private ArrayList<Integer> positions;
    private int currentPosition = 0;

    public ServoFromDcMotor(String name, DcMotor motor) {
        super(null,name);
        positions = setPositions();
    }

    abstract ArrayList<Integer> setPositions();

    public void goTo(int index, double speed) {
        currentPosition = positions.get(index) > currentPosition ? positions.get(index) - currentPosition : -(currentPosition - positions.get(index));
        MotorEncoderController.getInstance().drive(this, currentPosition, speed, true);
    }
}
