package org.firstinspires.ftc.teamcode.Vision;

import static org.opencv.core.Core.inRange;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Opmodes.Autonomous.AutoOpmode;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.AllianceColor;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Signal;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Config
public class CombinedTracker extends OpenCvPipeline {

    public static TrackType trackType = TrackType.SLEEVE;

    public double CONTOUR_AREA = 250.0;
    private final Scalar CONTOUR_COLOR = new Scalar(255,0,255);
    private final Scalar HORIZON_COLOR = new Scalar(0,255,0);
    private final Scalar TEXT_COLOR = new Scalar(0, 0, 0);
    private int contourIndex = 0;
    private final double cameraAngle;

    public enum DETECT_COLOR {
        RED,
        BLUE,
        BOTH
    }

    public static DETECT_COLOR coneColor = DETECT_COLOR.RED;
    private Signal signalColor = Signal.NONE;

    public double horizon = 5;

    private Rect redRect = new Rect();
    private Rect blueRect = new Rect();
    private Rect poleRect = new Rect();

    private final List<MatOfPoint> redContours = new ArrayList<>();
    private final List<MatOfPoint> blueContours = new ArrayList<>();

    private final ArrayList<MatOfPoint> contours = new ArrayList<>();
    private final ArrayList<Rect> possibleSignals = new ArrayList<>();
    // Cone mask scalars
    private final Scalar redLow = new Scalar(0, 170, 60);
    private final Scalar redHigh = new Scalar(80, 255, 255);
    private final Scalar blueLow = new Scalar(0, 61, 138);
    private final Scalar blueHigh = new Scalar(61, 255, 255);
    // Pole mask scalars
    public Scalar poleLower = new Scalar(60, 135, 10);
    public Scalar poleHigher = new Scalar(190, 180, 105);
    // Signal sleeve mask scalars
    private final Scalar greenLower = new Scalar(32, 10, 75);
    private final Scalar greenUpper = new Scalar(86, 255,255);
    private final Scalar blueLower = new Scalar(80, 140, 75);
    private final Scalar blueUpper = new Scalar(133, 255, 190);
    private final Scalar yellowLower = new Scalar(17, 100, 100);
    private final Scalar yellowUpper = new Scalar(40, 255, 255);
    // Mat objects
    private final Mat maskRed = new Mat();
    private final Mat maskBlue = new Mat();
    private final Mat yCrCb = new Mat();
    private final Mat hsvMat = new Mat();
    private final Mat binaryMat = new Mat();

    private final Size kSize = new Size(5, 5);
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kSize);

    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    Mat cameraMatrix;

    Scalar blue = new Scalar(7,197,235,255);
    Scalar red = new Scalar(255,0,0,255);
    Scalar green = new Scalar(0,255,0,255);
    Scalar white = new Scalar(255,255,255,255);

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics, found in teamwebcamcalibrations.xml in res/xml/
    // 640x480 for C920 Logitech
    // UNITS ARE PIXELS
    private final double fx = 622.001;
    private final double fy = 622.001;
    private final double cx = 319.803;
    private final double cy = 241.251;

    // UNITS ARE METERS
    private final double tagsize = 0.166;
    private double tagsizeX;
    private double tagsizeY;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    public CombinedTracker(double cameraAngle) {
        this.cameraAngle = cameraAngle;

        constructMatrix();
        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    public CombinedTracker() {
        this.cameraAngle = 0.0;
        constructMatrix();
        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public Mat processFrame(Mat input) {
        switch (trackType) {
            case CONE:
                return detectCone(input);
            case POLE:
                return detectPole(input);
            case SLEEVE:
                return detectApriltag(input);
            default:
                return input;
        }
    }

    private Mat detectApriltag(Mat input) {
        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync)
        {
            if(needToSetDecimation)
            {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

        synchronized (detectionsUpdateSync)
        {
            detectionsUpdate = detections;
        }
        //ToDo re-implement this

        // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
        // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
//        for(AprilTagDetection detection : detections)
//        {
//            AprilTagTracker.Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
//            drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
//            draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
//        }

        return input;
    }

    //ToDo Convert to using only a single contour list
    private Mat detectCone(Mat input) {

        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.erode(yCrCb, yCrCb, kernel);

        if (coneColor.equals(DETECT_COLOR.RED) || coneColor.equals(DETECT_COLOR.BOTH)) {
            inRange(yCrCb, redLow, redHigh, maskRed);

            redContours.clear();

            Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            redContours.removeIf(c -> Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon);
            Imgproc.drawContours(input, redContours, -1, CONTOUR_COLOR);

            if(!redContours.isEmpty()) {
                redContours.sort(Collections.reverseOrder(Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width)));
                MatOfPoint biggestRedContour = redContours.get(0);
                try {
                    biggestRedContour = redContours.get(contourIndex);
                } catch (IndexOutOfBoundsException ignore) {}
                if(Imgproc.contourArea(biggestRedContour) > CONTOUR_AREA) {
                    redRect = Imgproc.boundingRect(biggestRedContour);

                    Imgproc.rectangle(input, redRect, CONTOUR_COLOR, 2);
                    Imgproc.putText(input, "Red Cone", new Point(redRect.x, redRect.y < 10 ? (redRect.y+redRect.height+20) : (redRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 1);
                    Imgproc.circle(input, new Point(redRect.x + (redRect.width/2.0), redRect.y + (redRect.height/2.0)), 3, HORIZON_COLOR, 3);
                }
            }

            maskRed.release();
        }

        if (coneColor.equals(DETECT_COLOR.BLUE) || coneColor.equals(DETECT_COLOR.BOTH)) {
            inRange(yCrCb, blueLow, blueHigh, maskBlue);

            blueContours.clear();

            Imgproc.findContours(maskBlue, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            blueContours.removeIf(c -> Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon);
            Imgproc.drawContours(input, blueContours, -1, CONTOUR_COLOR);

            if(!blueContours.isEmpty()) {
                blueContours.sort(Collections.reverseOrder(Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width)));
                MatOfPoint biggestBlueContour = blueContours.get(0);
                try {
                    biggestBlueContour = blueContours.get(contourIndex);
                } catch (IndexOutOfBoundsException ignore) {}
                if(Imgproc.contourArea(biggestBlueContour) > CONTOUR_AREA) {
                    blueRect = Imgproc.boundingRect(biggestBlueContour);

                    Imgproc.rectangle(input, blueRect, CONTOUR_COLOR, 2);
                    Imgproc.putText(input, "Blue Cone", new Point(blueRect.x, blueRect.y < 10 ? (blueRect.y+blueRect.height+20) : (blueRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 1);
                    Imgproc.circle(input, new Point(blueRect.x + (blueRect.width/2.0), blueRect.y + (blueRect.height/2.0)), 3, HORIZON_COLOR, 3);
                }
            }
            maskBlue.release();
        }

//        if(coneColor.equals(DETECT_COLOR.BOTH))
//            Imgproc.rectangle(input, new Rect(new Point(Math.min(redRect.x, blueRect.x), Math.min(redRect.y, blueRect.y)), new Point(Math.max(redRect.x + redRect.width, blueRect.x + blueRect.width), Math.max(redRect.y + redRect.height, blueRect.y + blueRect.height))), CONTOUR_COLOR);

        Imgproc.line(input, new Point(0,horizon), new Point(640, horizon), HORIZON_COLOR);

        yCrCb.release();

        return input;
    }

    private Mat detectPole(Mat input) {
        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.erode(yCrCb, yCrCb, kernel);

        inRange(yCrCb, poleLower, poleHigher, binaryMat);

        Imgproc.findContours(binaryMat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        contours.removeIf(c -> Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon);
        Imgproc.drawContours(input, contours, -1, CONTOUR_COLOR);

        if(!contours.isEmpty()) {
            MatOfPoint biggestPole = Collections.max(contours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).height));
            if(Imgproc.contourArea(biggestPole) > CONTOUR_AREA) {
                poleRect = Imgproc.boundingRect(biggestPole);

                Imgproc.rectangle(input, poleRect, CONTOUR_COLOR, 2);
                Imgproc.putText(input, "Pole " + (poleRect.x + (poleRect.width/2.0)) +","+(poleRect.y + (poleRect.height/2.0)), new Point(poleRect.x, poleRect.y < 10 ? (poleRect.y+poleRect.height+20) : (poleRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
                Imgproc.circle(input, new Point(poleRect.x + (poleRect.width/2.0), poleRect.y + (poleRect.height/2.0)), 3, HORIZON_COLOR, 3);
            }
        }

        Imgproc.line(input, new Point(0,horizon), new Point(320, horizon), HORIZON_COLOR);
        Imgproc.circle(input, new Point(320, 240), 3, HORIZON_COLOR, 3);
//        Imgproc.line(input, new Point(320, 240), new Point(poleRect.x + (poleRect.width/2.0), poleRect.y + (poleRect.height/2.0)), HORIZON_COLOR, 2);

        contours.clear();
        yCrCb.release();
        binaryMat.release();

        return input;
    }

    /**
     * Use color for signal sleeve detection
     */
    @Deprecated
    private Mat detectSleeve(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Imgproc.erode(hsvMat, hsvMat, kernel);

        inRange(hsvMat, greenLower, greenUpper, binaryMat);
        Imgproc.findContours(binaryMat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        drawContour(contours);
        contours.clear();
        binaryMat.release();

        inRange(hsvMat, yellowLower, yellowUpper, binaryMat);
        Imgproc.findContours(binaryMat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        drawContour(contours);
        contours.clear();
        binaryMat.release();

        inRange(hsvMat, blueLower, blueUpper, binaryMat);
        Imgproc.findContours(binaryMat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        drawContour(contours);
        contours.clear();
        binaryMat.release();
        hsvMat.release();

        // Check what contour has the largest area.
        if(possibleSignals.get(0).area() > possibleSignals.get(1).area() && possibleSignals.get(0).area() > possibleSignals.get(2).area()) {
            signalColor = Signal.LEFT;
            Imgproc.rectangle(input, possibleSignals.get(0), CONTOUR_COLOR);
            Imgproc.putText(input, "Green", new Point(possibleSignals.get(0).x, wrapText(0)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
        } else if(possibleSignals.get(1).area() > possibleSignals.get(2).area()) {
            signalColor = Signal.RIGHT;
            Imgproc.rectangle(input, possibleSignals.get(1), CONTOUR_COLOR);
            Imgproc.putText(input, "Yellow", new Point(possibleSignals.get(1).x, wrapText(1)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
        } else if(possibleSignals.get(2).area() > 1) {
            signalColor = Signal.MIDDLE;
            Imgproc.rectangle(input, possibleSignals.get(2), CONTOUR_COLOR);
            Imgproc.putText(input, "Blue", new Point(possibleSignals.get(2).x, wrapText(2)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
        } else
            signalColor = Signal.NONE;

        possibleSignals.clear();

        Imgproc.line(input, new Point(0,horizon), new Point(640, horizon), HORIZON_COLOR);

        return input;
    }

    private void drawContour(ArrayList<MatOfPoint> contours) {
        // ToDO Remove sorting of contours and just loop through them with a largestRect variable
        // Order contours in descending order by width
        contours.sort(Collections.reverseOrder(Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width)));
        Rect r = new Rect(0,0,0,0);
        for (MatOfPoint c : contours) {
            r = Imgproc.boundingRect(c.clone());
            c.release();
            if (r.y + (r.height/2.0) > horizon && r.area() > 50.0)
                break;
            r = new Rect(0,0,0,0);
        }
        possibleSignals.add(r);
    }

    private double wrapText(int i) {
        return possibleSignals.get(0).y < 10 ? (possibleSignals.get(i).y+possibleSignals.get(i).height+20) : (possibleSignals.get(i).y - 8);
    }

    public void setContourIndex(int contourIndex) {
        this.contourIndex = contourIndex;
    }

    public void setTrackType(TrackType trackType) {
        CombinedTracker.trackType = trackType;
    }

    public Rect getBiggestCone() {
        if(trackType.equals(TrackType.POLE))
            return poleRect;
        if(coneColor.equals(DETECT_COLOR.BOTH)) {
            if(almostEqual(redRect.x + (redRect.width/2.0), blueRect.x + (blueRect.width/2.0), 10.0))
                return new Rect(new Point(Math.min(redRect.x, blueRect.x), Math.min(redRect.y, blueRect.y)), new Point(Math.max(redRect.x + redRect.width, blueRect.x + blueRect.width), Math.max(redRect.y + redRect.height, blueRect.y + blueRect.height)));
            else {
                return AutoOpmode.robotColor.equals(AllianceColor.RED) ? redRect : blueRect;
            }
        }
        return coneColor.equals(DETECT_COLOR.RED) ? redRect : blueRect;
    }

    private boolean almostEqual(double a, double b, double eps){
        return Math.abs(a-b) < eps;
    }

    public double getPoleAngle() {
        if(poleRect == null)
            return 0.0;
        return((poleRect.x + (poleRect.width/2.0) - 160.0) / 6.23333);
    }

    public double getObjectAngle() {
        Rect r = getBiggestCone();
        if(r == null)
            return 0.0;
        return((r.x + (r.width/2.0) - 160.0) / 6.23333);
    }

    public double getCameraAngle() {
        return cameraAngle;
    }

    public double getPoleDistance() {
        return poleRect == null ? 0 : ((1.05 * 853.33333333333) / poleRect.width);
    }

    public static TrackType getTrackType() {
        return trackType;
    }

    public void setDecimation(float decimation)
    {
        synchronized (decimationSync)
        {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    public ArrayList<AprilTagDetection> getLatestDetections()
    {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate()
    {
        synchronized (detectionsUpdateSync)
        {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }

    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param buf the RGB buffer on which to draw the marker
     * @param length the length of each of the marker 'poles'
     * @param rvec the rotation vector of the detection
     * @param tvec the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0,0,0),
                new Point3(length,0,0),
                new Point3(0,length,0),
                new Point3(0,0,-length)
        );

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }

    void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
        //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(-tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2,-tagHeight/2,-length),
                new Point3(-tagWidth/2,-tagHeight/2,-length));

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Pillars
        for(int i = 0; i < 4; i++)
        {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, thickness);
        }

        // Base lines
        //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
        //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
        //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
        //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

        // Top lines
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
    }

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsizeX the original width of the tag
     * @param tagsizeY the original height of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    AprilTagTracker.Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY)
    {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an 'ideal projection'
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Using this information, actually solve for pose
        AprilTagTracker.Pose pose = new AprilTagTracker.Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    static class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat();
            tvec = new Mat();
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }

    @Override
    protected void finalize() throws Throwable {
        super.finalize();
        if(nativeApriltagPtr != 0)
        {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        } else {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }
}
