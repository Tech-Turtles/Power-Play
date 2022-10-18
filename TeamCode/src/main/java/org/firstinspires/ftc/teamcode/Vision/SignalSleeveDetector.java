package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class SignalSleeveDetector {
    private OpenCvCamera camera;
    private final String webcamName;
    private final HardwareMap hardwareMap;
    private SignalSleeveTracker pipeline;

    public SignalSleeveDetector(HardwareMap hardwareMap, String webcamName) {
        this.hardwareMap = hardwareMap;
        this.webcamName = webcamName;
    }

    public void init() {
        //This will instantiate an OpenCvCamera object for the camera we'll be using
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        // Use the phone camera if a phone is being used as the Robot Controller
//            camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //Set the pipeline the camera should use and start streaming
        pipeline = new SignalSleeveTracker();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public String getColor() {
        return pipeline.getSignal().name();
    }

    public int getOrdinal() {
        return pipeline.getSignal().ordinal();
    }
}
