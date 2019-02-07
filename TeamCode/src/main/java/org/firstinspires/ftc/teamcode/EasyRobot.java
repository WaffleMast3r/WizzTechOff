package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


public abstract class EasyRobot extends LinearOpMode {

    private static final String VUFORIA_KEY = "AWIKGT7/////AAABmS3bsjJazEclpQEbA+BwmGRv5e1zEDZUgBVwvH+PniaFdrTOn96jCmryjubsOJdYXcsAkilFSWI4OSu4CNdB/AG6Q4JKSdwrvYwQIQ6H03QXP28Vf3gQynijHYcAVwymB939toFmo/hujvmzOTaecqE1F9dlVs03PREtZKQ4N1OOxS3mDoq4BxD8ZXFxvdAB/+aIrW//rGjWNBnf99CzCLZZruLyCvNcg/lu3DhH7o6PwVqjDZV8xNFoNGnHcKKbEgxibyYobK0uHZoxcIwLDFo7YLf+HYUSPWs4Pc6TS/KW8sa47Q8lpdDUiu0TSnJOQLdMc2C5H10nDo65mTkWJOnVuak+FKJL9LH7Ix3Ux5/A";
    private static TFObjectDetector[] tfods = new TFObjectDetector[3];
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private WizzTechDcMotor leftMotorUp, rightMotorUp, leftMotorDown, rightMotorDown, handMotor, extendLift, extendLift2;
    private CRServo collectorServo;
    private Servo collectorRotateServo, paletaServo1, paletaServo2, liftServo1, liftServo2;
    private BNO055IMU imu;
    private Orientation angles;
    private TeamSide side = TeamSide.UNKNOWN;
    private VuforiaLocalizer[] cams = new VuforiaLocalizer[3];
    private List<VuforiaTrackable> allTrackables;
    private VuforiaTrackables targetsRoverRuckus;
    private OpenGLMatrix lastLocation;
    private boolean targetVisible;

    public EasyRobot() {
    }

    public static void disable() {
        for (int i = 0; i < tfods.length; i++) {
            if (tfods[i] != null) {
                tfods[i].shutdown();
            }
        }
    }


    public void initRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;

        leftMotorUp = new WizzTechDcMotor(opMode, "m1");
        rightMotorUp = new WizzTechDcMotor(opMode, "m2");
        leftMotorDown = new WizzTechDcMotor(opMode, "m3");
        rightMotorDown = new WizzTechDcMotor(opMode, "m4");

        handMotor = new WizzTechDcMotor(opMode, "m5");
        extendLift = new WizzTechDcMotor(opMode, "m6");
        extendLift2 = new WizzTechDcMotor(opMode, "m7");

        handMotor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendLift.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendLift2.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        handMotor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendLift.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendLift2.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extendLift.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collectorServo = opMode.hardwareMap.crservo.get("s1");
        collectorRotateServo = opMode.hardwareMap.servo.get("s2");

        paletaServo1 = opMode.hardwareMap.servo.get("s3");
        paletaServo2 = opMode.hardwareMap.servo.get("s4");
        liftServo1 = opMode.hardwareMap.servo.get("s5");
        liftServo2 = opMode.hardwareMap.servo.get("s6");
    }

    public void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters internalBack = new VuforiaLocalizer.Parameters();
//        VuforiaLocalizer.Parameters internalFront = new VuforiaLocalizer.Parameters();
        VuforiaLocalizer.Parameters extern1 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

//        internalBack.vuforiaLicenseKey = VUFORIA_KEY;
//        internalFront.vuforiaLicenseKey = VUFORIA_KEY;
        extern1.vuforiaLicenseKey = VUFORIA_KEY;

//        internalBack.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        internalFront.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        extern1.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

//        cams[0] = ClassFactory.getInstance().createVuforia(internalBack);
//        cams[1] = ClassFactory.getInstance().createVuforia(internalFront);
        cams[2] = ClassFactory.getInstance().createVuforia(extern1);

    }

    public void initTfod(int index) {
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            if (cams[index] != null) {
                TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
                tfods[index] = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, cams[index]);
                tfods[index].loadModelFromAsset("RoverRuckus.tflite", "Gold Mineral", "Silver Mineral");
            }

        } else {
            System.err.println("Cannot init tfod");
        }

    }

    public void runObjectDetection(int cameraIndex, ObjectDetected action) {
        TFObjectDetector tf = tfods[cameraIndex];
        tf.activate();
        for (int i = 0; i < 3; i++) {
            List<Recognition> updatedRecognitions = tf.getUpdatedRecognitions();
            while (true) {
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() == 1) break;
                }
                updatedRecognitions = tf.getUpdatedRecognitions();
            }
            if (updatedRecognitions.get(0).getLabel().equals("Gold Mineral")) {
                action.pickup();
                switch (i) {
                    case 0:
                        action.onCenter();
                    case 1:
                        action.onLeft();
                    case 2:
                        action.onRight();
                }
                action.loadCargo();
                return;

            } else {
                if (i == 0) {
                    turnTo(0.3, 27);
                } else {
                    turnTo(0.3, -27);
                }
            }
        }
    }

    public void initGyro(BNO055IMU.AngleUnit angle) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = angle;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @SuppressLint("DefaultLocale")
    public double getAngle(Axis axis) {
        switch (axis) {
            case X:
                return Double.parseDouble(String.format("%.00f", imu.getAngularOrientation(EXTRINSIC, XYZ, DEGREES).firstAngle));
            case Y:
                return Double.parseDouble(String.format("%.00f", imu.getAngularOrientation(EXTRINSIC, XYZ, DEGREES).secondAngle));
            case Z:
                return Double.parseDouble(String.format("%.00f", imu.getAngularOrientation(EXTRINSIC, XYZ, DEGREES).thirdAngle));
            default:
                return 0;
        }
    }

    public void initTrackable(int cameraIndex, TrackableSettings settings) {
        targetsRoverRuckus = cams[cameraIndex].loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, settings.getMmFTCFieldWidth(), settings.getMmTargetHeight())
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -settings.getMmFTCFieldWidth(), settings.getMmTargetHeight())
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-settings.getMmFTCFieldWidth(), 0, settings.getMmTargetHeight())
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(settings.getMmFTCFieldWidth(), 0, settings.getMmTargetHeight())
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(settings.getCAMERA_FORWARD_DISPLACEMENT(), settings.getCAMERA_LEFT_DISPLACEMENT(), settings.getCAMERA_VERTICAL_DISPLACEMENT())
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, -90, 0, 0)); // TODO: 1/21/2019 Poate e 90

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            if (cameraIndex >= 2) {
                ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(cams[cameraIndex].getCameraName(), phoneLocationOnRobot);
            } else {
                if (cameraIndex == 0) {
                    ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, VuforiaLocalizer.CameraDirection.BACK);
                } else {
                    ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, VuforiaLocalizer.CameraDirection.FRONT);
                }
            }
        }
    }

    public void activateTrackable() {
        targetsRoverRuckus.activate();
    }

    public Location getLocation() {

        targetVisible = false;

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {

                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
            }
        }

        if (targetVisible) {
            VectorF translation = lastLocation.getTranslation();
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            return new Location(translation.get(0) / 25.4f, translation.get(1) / 25.4f, translation.get(2) / 25.4f, rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }

        return new Location(-1);
    }

    @SuppressWarnings("deprecation")
    public void turnTo(double power, int degrees) {

        double angle = getAngle(Axis.Z);
        while (Math.abs(angle - degrees) > 0) {
            angle = getAngle(Axis.Z);

            telemetry.addData("Trying to get to", degrees + "(" + power + ")");
            telemetry.addData("Angle", angle);
            telemetry.update();

            if (angle < degrees) {
                getLeftMotorUp().getMotor().setPower(power);
                getRightMotorUp().getMotor().setPower(power);
                getLeftMotorDown().getMotor().setPower(power);
                getRightMotorDown().getMotor().setPower(power);
            } else if (angle > degrees) {
                getLeftMotorUp().getMotor().setPower(-power);
                getRightMotorUp().getMotor().setPower(-power);
                getLeftMotorDown().getMotor().setPower(-power);
                getRightMotorDown().getMotor().setPower(-power);
            }
            try {
                opMode.waitOneFullHardwareCycle();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        try {
            opMode.waitOneFullHardwareCycle();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        getLeftMotorUp().getMotor().setPower(0.01);
        getRightMotorUp().getMotor().setPower(0.01);
        getLeftMotorDown().getMotor().setPower(0.01);
        getRightMotorDown().getMotor().setPower(0.01);
        opMode.sleep(500);
        getLeftMotorUp().getMotor().setPower(0);
        getRightMotorUp().getMotor().setPower(0);
        getLeftMotorDown().getMotor().setPower(0);
        getRightMotorDown().getMotor().setPower(0);
    }

    public WizzTechDcMotor getLeftMotorUp() {
        return leftMotorUp;
    }

    public WizzTechDcMotor getRightMotorUp() {
        return rightMotorUp;
    }

    public WizzTechDcMotor getLeftMotorDown() {
        return leftMotorDown;
    }

    public WizzTechDcMotor getRightMotorDown() {
        return rightMotorDown;
    }

    public WizzTechDcMotor getExtendLift2() {
        return extendLift2;
    }

    public WizzTechDcMotor getExtendLift() {
        return extendLift;
    }

    public Servo getPaletaServo1() {
        return paletaServo1;
    }

    public Servo getPaletaServo2() {
        return paletaServo2;
    }

    public Servo getLiftServo1() {
        return liftServo1;
    }

    public Servo getLiftServo2() {
        return liftServo2;
    }

    public Servo getCollectorRotateServo() {
        return collectorRotateServo;
    }

    public CRServo getCollectorServo() {
        return collectorServo;
    }

    public TeamSide getSide() {
        return side;
    }

    public void setSide(TeamSide side) {
        this.side = side;
    }

    public WizzTechDcMotor getHandMotor() {
        return handMotor;
    }

    public VuforiaLocalizer getVuforia(int index) {
        return cams[index];
    }

    @Deprecated
    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public enum Axis {X, Y, Z}

    public enum TeamSide {
        RIGHT(135), LEFT(225), UNKNOWN(0);

        private int angle;

        TeamSide(int angle) {
            this.angle = angle;
        }

        public int getAngle() {
            return angle;
        }

    }

    public interface ObjectDetected {
        void pickup();

        void onLeft();

        void onCenter();

        void onRight();

        void loadCargo();
    }

    public class Location {
        float x;
        float y;
        float z;
        float roll;
        float pitch;
        float heading;

        int status = 1;

        public Location(int status) {
            this.status = status;
        }

        public Location(float x, float y, float z, float roll, float pitch, float heading) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.roll = roll;
            this.pitch = pitch;
            this.heading = heading;
        }

        public float getX() {
            return x;
        }

        public float getY() {
            return y;
        }

        public float getZ() {
            return z;
        }

        public float getRoll() {
            return roll;
        }

        public float getPitch() {
            return pitch;
        }

        public float getHeading() {
            return heading;
        }

        public void print() {
            if (status != -1) {
                telemetry.addData("Position(X, Y, Z)", "%.00f, %.00f, %.00f", x, y, z);
                telemetry.addData("Position(Roll, Pitch, Heading)", "%.00f, %.00f, %.00f", roll, pitch, heading);
//            telemetry.addData("X", x);
//            telemetry.addData("Y", y);
//            telemetry.addData("Z", z);
//            telemetry.addData("Roll", roll);
//            telemetry.addData("Pitch", pitch);
//            telemetry.addData("Heading", heading);
            }
            telemetry.update();
        }
    }

    public class TrackableSettings {
        float mmPerInch = 25.4f;
        float mmFTCFieldWidth = (12 * 6) * mmPerInch;
        float mmTargetHeight = (6) * mmPerInch;
        int CAMERA_FORWARD_DISPLACEMENT = 110;
        int CAMERA_VERTICAL_DISPLACEMENT = 200;
        int CAMERA_LEFT_DISPLACEMENT = 0;

        public TrackableSettings() {
        }

        public float getMmFTCFieldWidth() {
            return mmFTCFieldWidth;
        }

        public float getMmTargetHeight() {
            return mmTargetHeight;
        }

        public int getCAMERA_FORWARD_DISPLACEMENT() {
            return CAMERA_FORWARD_DISPLACEMENT;
        }

        public int getCAMERA_VERTICAL_DISPLACEMENT() {
            return CAMERA_VERTICAL_DISPLACEMENT;
        }

        public int getCAMERA_LEFT_DISPLACEMENT() {
            return CAMERA_LEFT_DISPLACEMENT;
        }
    }

//      if (updatedRecognitions.size() == 3) {
//                        int goldMineralX = -1;
//                        int silverMineral1X = -1;
//                        int silverMineral2X = -1;
//                        for (Recognition recognition : updatedRecognitions) {
//                            if (recognition.getLabel().equals("Gold Mineral")) {
//                                goldMineralX = (int) recognition.getRight();
//                            } else if (silverMineral1X == -1) {
//                                silverMineral1X = (int) recognition.getRight();
//                            } else {
//                                silverMineral2X = (int) recognition.getRight();
//                            }
//                        }
//                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                                if (positions[0]++ == 10) {
//                                    action.onLeft(goldMineralX, silverMineral1X, silverMineral2X);
//                                    tf.shutdown();
//                                    return;
//                                }
//                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                                if (positions[2]++ == 10) {
//                                    action.onRight(goldMineralX, silverMineral1X, silverMineral2X);
//                                    tf.shutdown();
//                                    return;
//                                }
//                            } else {
//                                if (positions[1]++ == 10) {
//                                    action.onCenter(goldMineralX, silverMineral1X, silverMineral2X);
//                                    tf.shutdown();
//                                    return;
//                                }
//                            }
//                        }
//                    }
}
