package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.teamcode.Utility.Autonomous.Signal;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

public class SignalSleeveTracker extends OpenCvPipeline {

    Signal color = Signal.NONE;

    public Scalar greenLower = new Scalar(32, 10, 75);
    public Scalar greenUpper = new Scalar(86, 255,255);

    public Scalar blueLower = new Scalar(80, 140, 75);
    public Scalar blueUpper = new Scalar(133, 255, 190);

    public Scalar yellowLower = new Scalar(17, 100, 100);
    public Scalar yellowUpper = new Scalar(40, 255, 255);

    private final Scalar CONTOUR_COLOR = new Scalar(255,0,255);
    private final Scalar HORIZON_COLOR = new Scalar(0,255,0);
    private final Scalar TEXT_COLOR = new Scalar(0,200,0);

    public double horizon = 130;

    private final Mat hsvMat = new Mat();
    private final Mat binaryMat = new Mat();
    private final Mat maskedInputMat = new Mat();

    private final ArrayList<MatOfPoint> contours = new ArrayList<>();
    private final ArrayList<Rect> possibleSignals = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        possibleSignals.clear();
        maskedInputMat.release();

        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsvMat, greenLower, greenUpper, binaryMat);
        Imgproc.findContours(binaryMat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        drawContour(contours);
        contours.clear();
        binaryMat.release();

        Core.inRange(hsvMat, yellowLower, yellowUpper, binaryMat);
        Imgproc.findContours(binaryMat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        drawContour(contours);
        contours.clear();
        binaryMat.release();

        Core.inRange(hsvMat, blueLower, blueUpper, binaryMat);
        Imgproc.findContours(binaryMat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        drawContour(contours);
        contours.clear();

        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        if(possibleSignals.get(0).area() > possibleSignals.get(1).area() && possibleSignals.get(0).area() > possibleSignals.get(2).area()) {
            color = Signal.LEFT;
            Imgproc.rectangle(input, possibleSignals.get(0), CONTOUR_COLOR);
            Imgproc.putText(input, "Green", new Point(possibleSignals.get(0).x, wrapText(0)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
        } else if(possibleSignals.get(1).area() > possibleSignals.get(2).area()) {
            color = Signal.RIGHT;
            Imgproc.rectangle(input, possibleSignals.get(1), CONTOUR_COLOR);
            Imgproc.putText(input, "Yellow", new Point(possibleSignals.get(1).x, wrapText(1)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
        } else if(possibleSignals.get(2).area() > 1) {
            color = Signal.MIDDLE;
            Imgproc.rectangle(input, possibleSignals.get(2), CONTOUR_COLOR);
            Imgproc.putText(input, "Blue", new Point(possibleSignals.get(2).x, wrapText(2)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
        } else
            color = Signal.NONE;

        binaryMat.release();
        hsvMat.release();
        maskedInputMat.release();

        Imgproc.line(input, new Point(0,horizon), new Point(640, horizon), HORIZON_COLOR);

        return input;
    }

    private void drawContour(ArrayList<MatOfPoint> contours) {
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

    public Signal getSignal() {
        return color;
    }
}
