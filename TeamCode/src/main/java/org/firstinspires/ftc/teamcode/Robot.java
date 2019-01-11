package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class Robot {

    private static final String VUFORIA_KEY = "Ae1B0KT/////AAAAmb1XYh8lnkJApASHU4GlfoqG1HM2p/vcZ5IxoIMZChOo2PH0w70nDnGNquAykLVE1+9dA+dH8LGl5G1s0ts72YIhfH7FShO4GtIjsIkf8Sgolfi3qdzfQ+t0ga1a90ISGY3ZxKFoz6M6I8URFSPwju493j1WM73/xTwIWyMy3SSgz8O0S+MSrYTUG8e97iY3RLcH6OefPQNWzvH9Lh8+rxnjwR9RR40WHD/Oefh83kN7EanocJi/PUxTc+zAlfrcurVQCUTOd3yHlZeFtrZ9zVMgPZ/p9RKYK+/gUKYmmdBALrtjkFC6YI6XPRgCUnVZU9QP6DWj7XKT93PDRlaSvmhBztDG+GGGb9/Vu2Hwbg5b";

    private static Robot instance = null;
    private HardwareMap hardwareMap;
    private WizzTechDcMotor leftMotorUp, rightMotorUp, leftMotorDown, rightMotorDown, extendCollectorMotorArm, extendLiftUp, extendLiftDown;
    private ServoFromDcMotor collectorMotor;
    private CRServo collectorServo;
    private BNO055IMU imu;
    private Orientation angles;
    private TeamSide side = TeamSide.UNKNOWN;
    private VuforiaLocalizer[] cams = new VuforiaLocalizer[3];
    private TFObjectDetector[] tfods = new TFObjectDetector[3];

    private Robot() {}

    public static Robot getInstance() {
        if (instance == null) instance = new Robot();
        return instance;
    }

    public static void disable() {
        instance = null;
    }

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        //Init of the chassis motor
        //modificare suspicioasa
        leftMotorUp = new WizzTechDcMotor("m1");
        rightMotorUp = new WizzTechDcMotor("m2");
        leftMotorDown = new WizzTechDcMotor("m3");
        rightMotorDown = new WizzTechDcMotor("m4");

        extendLiftUp = new WizzTechDcMotor("m5");
        extendLiftDown = new WizzTechDcMotor("m6");

        initGyro(BNO055IMU.AngleUnit.DEGREES);
        initVuforia();
    }

    public void initVuforia() {
        VuforiaLocalizer.Parameters internalBack = new VuforiaLocalizer.Parameters();
        VuforiaLocalizer.Parameters internalFront = new VuforiaLocalizer.Parameters();
        VuforiaLocalizer.Parameters extern1 = new VuforiaLocalizer.Parameters();

        internalBack.vuforiaLicenseKey = VUFORIA_KEY;
        internalFront.vuforiaLicenseKey = VUFORIA_KEY;
        extern1.vuforiaLicenseKey = VUFORIA_KEY;
        internalBack.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        internalFront.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        extern1.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        cams[0] = ClassFactory.getInstance().createVuforia(internalBack);
        cams[1] = ClassFactory.getInstance().createVuforia(internalFront);
        cams[2] = ClassFactory.getInstance().createVuforia(extern1);
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            System.err.println("Cannot init tfod");
        }
    }

    public void runObjectDetection(CameraOrientation orientation, int cameraIndex, ObjectDetected action) {
        tfods[cameraIndex].activate();

        List<Recognition> updatedRecognitions = tfods[cameraIndex].getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            if (updatedRecognitions.size() == 3) {
                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;
                for (Recognition recognition : updatedRecognitions) {
                    switch (orientation) {
                        case PORTRAIT:
                            if (recognition.getLabel().equals("Gold Mineral")) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        case LANDSCAPE:
                            if (recognition.getLabel().equals("Gold Mineral")) {
                                goldMineralX = (int) recognition.getTop();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getTop();
                            } else {
                                silverMineral2X = (int) recognition.getTop();
                            }
                        case PORTRAIT_FLIPPED:
                            if (recognition.getLabel().equals("Gold Mineral")) {
                                goldMineralX = (int) recognition.getRight();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getRight();
                            } else {
                                silverMineral2X = (int) recognition.getRight();
                            }
                        case LANDSCAPE_FLIPPED:
                            if (recognition.getLabel().equals("Gold Mineral")) {
                                goldMineralX = (int) recognition.getBottom();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getBottom();
                            } else {
                                silverMineral2X = (int) recognition.getBottom();
                            }
                    }
                }
                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                        action.onLeft();
                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                        action.onRight();
                    } else {
                        action.onCenter();
                    }
                }
            }
        }

        tfods[cameraIndex].shutdown();
    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
        final String LABEL_GOLD_MINERAL = "Gold Mineral";
        final String LABEL_SILVER_MINERAL = "Silver Mineral";

        for (int i = 0; i < cams.length; i++) {
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfods[i] = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, cams[i]);
            tfods[i].loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
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
                return Double.parseDouble(String.format("%.0f", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle));
            case Y:
                return Double.parseDouble(String.format("%.0f", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle));
            case Z:
                return Double.parseDouble(String.format("%.0f", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle));
            default:
                return 0;
        }
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

    public WizzTechDcMotor getExtendCollectorMotorArm() {
        return extendCollectorMotorArm;
    }

    public WizzTechDcMotor getCollectorMotor() {
        return collectorMotor;
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

    public WizzTechDcMotor getExtendLiftUp() {
        return extendLiftUp;
    }

    public WizzTechDcMotor getExtendLiftDown() {
        return extendLiftDown;
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

    public enum CameraOrientation {
        PORTRAIT(4), LANDSCAPE(3), PORTRAIT_FLIPPED(2), LANDSCAPE_FLIPPED(1);

        private int id;

        CameraOrientation(int id) {
            this.id = id;
        }

        public int getOrientationID() {
            return id;
        }
    }

    public interface ObjectDetected {
        void onLeft();

        void onCenter();

        void onRight();
    }
}