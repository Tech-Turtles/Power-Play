package org.firstinspires.ftc.teamcode.Vision;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utility.Configuration;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class CombinedDetector {
    private final OpenCvWebcam[] camera = new OpenCvWebcam[2];
    private final String[] webcamName;
    private final HardwareMap hardwareMap;
    private CombinedTracker leftPipeline, rightPipeline;
    public static boolean dashboard = true;

    public CombinedDetector(HardwareMap hardwareMap, String... webcamName) {
        this.hardwareMap = hardwareMap;
        this.webcamName = webcamName;
    }

    public void init() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
//                .splitLayoutForMultipleViewports(
//                        cameraMonitorViewId,
//                        2,
//                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        camera[0] = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName[0]));
        Log.wtf("Camera 0 null: ", String.valueOf(camera[0] == null));
        if(webcamName.length > 1) {
            camera[1] = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName[1]));
            Log.wtf("Camera 1 null: ", String.valueOf(camera[1] == null));
        }

        leftPipeline = new CombinedTracker(Configuration.LEFT_CAMERA_DEG);
        camera[0].setPipeline(leftPipeline);
        camera[0].setMillisecondsPermissionTimeout(7000);
        camera[0].openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera[0].startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                Log.wtf("Camera 0 Error", String.valueOf(errorCode));
            }
        });

//        if(dashboard)
//            FtcDashboard.getInstance().startCameraStream(camera[0], 30);

        if(camera.length == 1)
            return;
        rightPipeline = new CombinedTracker(Configuration.RIGHT_CAMERA_DEG);
        camera[1].setPipeline(rightPipeline);
        camera[1].setMillisecondsPermissionTimeout(7000);
        camera[1].openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera[1].startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                Log.wtf("Camera 1 Error", String.valueOf(errorCode));
            }
        });
//        if(dashboard)
//            FtcDashboard.getInstance().startCameraStream(camera[1], 30);
    }

    //ToDo Add wrapper methods to this class rather than return the pipeline itself
    public CombinedTracker getLeftPipeline() {
        return leftPipeline;
    }

    public CombinedTracker getRightPipeline() {
        return rightPipeline;
    }
}
