package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class CombinedDetector {
    private OpenCvCamera camera;
    private final String webcamName;
    private final HardwareMap hardwareMap;
    private CombinedTracker pipeline;

    public CombinedDetector(HardwareMap hardwareMap, String webcamName) {
        this.hardwareMap = hardwareMap;
        this.webcamName = webcamName;
    }

    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        pipeline = new CombinedTracker();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    //ToDo Add wrapper methods to this class rather than return the pipeline itself
    public CombinedTracker getPipeline() {
        return pipeline;
    }
}
