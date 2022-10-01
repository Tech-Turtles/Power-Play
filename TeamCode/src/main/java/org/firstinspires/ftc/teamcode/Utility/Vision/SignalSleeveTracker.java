package org.firstinspires.ftc.teamcode.Utility.Vision;

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

    enum Signal {
        NONE,
        GREEN,
        BLUE,
        YELLOW
    }


    public Scalar greenLower = new Scalar(32, 10, 75);
    public Scalar greenUpper = new Scalar(86, 255,255);

    public Scalar blueLower = new Scalar(92, 48, 130);
    public Scalar blueUpper = new Scalar(143, 255, 255);

    public Scalar yellowLower = new Scalar(17, 100, 100);
    public Scalar yellowUpper = new Scalar(40, 255, 255);

    private Mat hsvMat = new Mat();
    private Mat binaryMat      = new Mat();
    private Mat maskedInputMat = new Mat();

    private ArrayList<MatOfPoint> contours = new ArrayList<>();
    private ArrayList<Rect> possibleSignals = new ArrayList<>();

    private Signal color;

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
            color = Signal.GREEN;
            Imgproc.rectangle(input, possibleSignals.get(0), new Scalar(255,0,255));
            Imgproc.putText(input, "Green", new Point(possibleSignals.get(0).x, possibleSignals.get(0).y < 10 ? (possibleSignals.get(0).y+possibleSignals.get(0).height+20) : (possibleSignals.get(0).y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255,255,255), 1);
        } else if(possibleSignals.get(1).area() > possibleSignals.get(2).area()) {
            color = Signal.YELLOW;
            Imgproc.rectangle(input, possibleSignals.get(1), new Scalar(255,0,255));
            Imgproc.putText(input, "Yellow", new Point(possibleSignals.get(1).x, possibleSignals.get(1).y < 10 ? (possibleSignals.get(1).y+possibleSignals.get(1).height+20) : (possibleSignals.get(1).y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255,255,255), 1);
        } else if(possibleSignals.get(2).area() > 1) {
            color = Signal.BLUE;
            Imgproc.rectangle(input, possibleSignals.get(2), new Scalar(255,0,255));
            Imgproc.putText(input, "Blue", new Point(possibleSignals.get(2).x, possibleSignals.get(2).y < 10 ? (possibleSignals.get(2).y+possibleSignals.get(2).height+20) : (possibleSignals.get(2).y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255,255,255), 1);
        } else
            color = Signal.NONE;

        binaryMat.release();

        return input;
    }

    public void drawContour(ArrayList<MatOfPoint> contours) {
        if(!contours.isEmpty()) {
            MatOfPoint biggestContour = Collections.max(contours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width));
            Rect rect = Imgproc.boundingRect(biggestContour).clone();
            possibleSignals.add(rect);
        } else
            possibleSignals.add(new Rect(0,0,0,0));
    }

    public Signal getSignal() {
        return color;
    }
}
