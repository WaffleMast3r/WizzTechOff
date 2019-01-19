package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;

public class MotorEncoderController {

    private static MotorEncoderController instance = null;
    private int TICKS_PER_REVOLUTION = 575;
    private double COUNTS_PER_INCH = TICKS_PER_REVOLUTION / 5;   //16.5 // TODO: 12/1/2018 Change here
    private HashMap<String, Thread> queued = new HashMap<>();
    private HashMap<String, Thread> running = new HashMap<>();
    private boolean emergencyStop = false;

    private MotorEncoderController() {
    }

    public static MotorEncoderController getInstance() {
        if (instance == null) instance = new MotorEncoderController();
        return instance;
    }

    public static void disable() {
        if (instance != null)
            instance.emergencyStop();
        instance = null;
    }

    public void waitForMotors(String... names){
        if (names.length == 1){
            if (names[0].equalsIgnoreCase("*") || names[0].equalsIgnoreCase("all")){
                while (!running.isEmpty()){}
            }else{
                for (String name : names){
                    while (running.get(name) != null){}
                }
            }
        }else if (names.length > 1){
            for (String name : names){
                while (running.get(name) != null){}
            }
        }
    }

    public synchronized void drive(final WizzTechDcMotor motor, final int distance, final double speed, boolean update, String... names) {
        try {
            queued.put(motor.getName(), new Thread(new Runnable() {
                @Override
                public void run() {

                    double inch = distance * (1 / 2.54);
                    motor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    int newPosition = (int) (COUNTS_PER_INCH * inch);

                    motor.getMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor.getMotor().setTargetPosition(newPosition);
                    motor.getMotor().setPower(speed);

                    while (motor.getMotor().isBusy() && !emergencyStop) {

                    }

                    motor.getMotor().setPower(0.01);
                    motor.getMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    running.remove(motor.getName());
                }
            }));
            if (update) {
                if (names.length == 0) {
                    update(motor.getName());
                } else {
                    update(names);
                }
            }
        } catch (Exception e) {
            System.out.println("drive");
        }
    }

    public synchronized void update(String... names) {
        try {
            if (names[0].equalsIgnoreCase("all") || names[0].equalsIgnoreCase("*")) {
                for (String name : queued.keySet()) {
                    running.put(name, queued.get(name));
                    queued.remove(name).start();
                }
            } else {
                for (String name : names) {
                    running.put(name, queued.get(name));
                    queued.remove(name).start();
                }
            }
        } catch (Exception e) {
            System.out.println("update");
        }
    }

    @SuppressWarnings("deprecation")
    public void emergencyStop() {
        for (String name : running.keySet()) {
            running.remove(name).stop();
        }
        emergencyStop = true;
    }

}
