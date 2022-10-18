package org.firstinspires.ftc.teamcode.Vision;

import static org.opencv.core.Core.inRange;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.Autonomous.Signal;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Config
public class CombinedTracker extends OpenCvPipeline {

    public TrackType trackType = TrackType.NONE;

    public double CONTOUR_AREA = 250.0;
    private final Scalar CONTOUR_COLOR = new Scalar(255,0,255);
    private final Scalar HORIZON_COLOR = new Scalar(0,255,0);
    private final Scalar TEXT_COLOR = new Scalar(0, 0, 0);

    enum DETECT_COLOR {
        RED,
        BLUE,
        BOTH
    }

    public DETECT_COLOR coneColor = DETECT_COLOR.BOTH;
    private Signal signalColor = Signal.NONE;

    public double horizon = 100;

    private Rect redRect = new Rect();
    private Rect blueRect = new Rect();
    private Rect poleRect = new Rect();

    private final List<MatOfPoint> redContours = new ArrayList<>();
    private final List<MatOfPoint> blueContours = new ArrayList<>();

    private final ArrayList<MatOfPoint> contours = new ArrayList<>();
    private final ArrayList<Rect> possibleSignals = new ArrayList<>();
    // Cone mask scalars
    private final Scalar redLow = new Scalar(0, 161, 60);
    private final Scalar redHigh = new Scalar(200, 255, 255);
    private final Scalar blueLow = new Scalar(0, 80, 138);
    private final Scalar blueHigh = new Scalar(100, 255, 255);
    // Pole mask scalars
    public Scalar poleLower = new Scalar(59, 134, 48);
    public Scalar poleHigher = new Scalar(180, 180, 105);
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

    @Override
    public Mat processFrame(Mat input) {
        switch (trackType) {
            case CONE:
                return detectCone(input);
            case POLE:
                return detectPole(input);
            case SLEEVE:
                return detectSleeve(input);
            default:
                return input;
        }
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
                MatOfPoint biggestRedContour = Collections.max(redContours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width));
                if(Imgproc.contourArea(biggestRedContour) > CONTOUR_AREA) {
                    redRect = Imgproc.boundingRect(biggestRedContour);

                    Imgproc.rectangle(input, redRect, CONTOUR_COLOR, 2);
                    Imgproc.putText(input, "Red Cone", new Point(redRect.x, redRect.y < 10 ? (redRect.y+redRect.height+20) : (redRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 1);
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
                MatOfPoint biggestBlueContour = Collections.max(blueContours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width));
                if(Imgproc.contourArea(biggestBlueContour) > CONTOUR_AREA) {
                    blueRect = Imgproc.boundingRect(biggestBlueContour);

                    Imgproc.rectangle(input, blueRect, CONTOUR_COLOR, 2);
                    Imgproc.putText(input, "Blue Cone", new Point(blueRect.x, blueRect.y < 10 ? (blueRect.y+blueRect.height+20) : (blueRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 1);
                }
            }
            maskBlue.release();
        }

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
                Imgproc.putText(input, "Pole", new Point(poleRect.x, poleRect.y < 10 ? (poleRect.y+poleRect.height+20) : (poleRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
            }
        }


        Imgproc.line(input, new Point(0,horizon), new Point(640, horizon), HORIZON_COLOR);

        contours.clear();
        yCrCb.release();
        binaryMat.release();

        return input;
    }

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
            signalColor = Signal.GREEN;
            Imgproc.rectangle(input, possibleSignals.get(0), CONTOUR_COLOR);
            Imgproc.putText(input, "Green", new Point(possibleSignals.get(0).x, wrapText(0)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
        } else if(possibleSignals.get(1).area() > possibleSignals.get(2).area()) {
            signalColor = Signal.YELLOW;
            Imgproc.rectangle(input, possibleSignals.get(1), CONTOUR_COLOR);
            Imgproc.putText(input, "Yellow", new Point(possibleSignals.get(1).x, wrapText(1)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
        } else if(possibleSignals.get(2).area() > 1) {
            signalColor = Signal.BLUE;
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

    public void setTrackType(TrackType trackType) {
        this.trackType = trackType;
    }


    public Signal getSignalColor() {
        return signalColor;
    }

    public Rect getBiggestCone() {
        return coneColor.equals(DETECT_COLOR.RED) ? redRect : blueRect;
    }
}
