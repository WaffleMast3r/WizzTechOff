package org.firstinspires.ftc.teamcode.Tests;

import android.app.Activity;
import android.content.Context;
import android.view.View;
import android.view.ViewGroup;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.android.JavaCameraView;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

@Autonomous(name = "Test 2 - OpenCV", group = "tests")
public class Test2 extends LinearOpMode {

    private View view;

    @Override
    public void runOpMode() throws InterruptedException {

        OpenCVLoader.loadOpenCV();
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        telemetry.addData("OpenCV", OpenCVLoader.initDebug());
        telemetry.update();

        waitForStart();
        VideoCapture cap = new VideoCapture();
        Mat frame = new Mat();
//        try{
//            cap.open(0);
//            cap.read(frame);
//        } catch (Exception e){
//            telemetry.addData("Err", e.getMessage());
//            telemetry.update();
//        }


//        Imgcodecs.imwrite("/storage/emulated/0/DCIM/Camera", frame);
//        cap.release();

        //Robot.disable();
        //MotorEncoderController.disable();
    }

    public void setCurrentView(Context context, View newView) {
        // ResourceID represents the id of the resource used from the FTC application (this way this can exist outside of the Teamcode sandbox
        final int resourceID = context.getResources().getIdentifier("RelativeLayout", "id", context.getPackageName());
        final Activity activity = (Activity) context;       //Get the parameter activity context
        final View queuedView = newView;
        activity.runOnUiThread(new Runnable() {         //Run on the Activity UI Thread
            @Override
            public void run() {
                ViewGroup cameraView = (ViewGroup) activity.findViewById(resourceID); //R.id.RelativeLayout) aka current view

                if (view != null) {
                    cameraView.removeView(view);        //Delete current view
                }
                cameraView.addView(queuedView);     //Add the modified view to the screen
                view = queuedView;                  //Update current view
            }
        });
    }

    private class sub extends JavaCameraView {
        public sub(Context context, int cameraId) {
            super(context, cameraId);
        }

        public View getView() {
            return findViewById(android.R.id.content);
        }
    }
}
