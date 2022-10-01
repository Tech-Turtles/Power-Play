package org.firstinspires.ftc.teamcode.Utility.Roadrunner.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareTypes.ColorSensor;
import org.firstinspires.ftc.teamcode.HardwareTypes.Motors;
import org.firstinspires.ftc.teamcode.Utility.RobotHardware;

import java.io.Serializable;
import java.util.EnumMap;
import java.util.HashMap;

public class TelemetryLog implements Serializable {
    private double time;
    private double x;
    private double y;
    private double heading;
    private double current_intake;
    private int encoder_launcher;
    private double range_wall;
    private double range_back;
    private double range_wobble;
    private int encoder_frontLeft;
    private int encoder_frontRight;
    private int encoder_backLeft;
    private int encoder_backRight;

    public static final String[] fieldOrder = {"TIME","X","Y","HEADING",
        "ENCODER_FRONTLEFT", "ENCODER_FRONTRIGHT", "ENCODER_BACKLEFT", "ENCODER_BACKRIGHT"};

    public TelemetryLog() {
        this(0.0, new Pose2d(), new EnumMap<Motors, Integer>(Motors.class){{
            put(Motors.FRONT_RIGHT,0);
            put(Motors.FRONT_LEFT,0);
            put(Motors.BACK_RIGHT,0);
            put(Motors.BACK_LEFT,0);
        }});
    }

    public TelemetryLog(RobotHardware robotHardware) {
        Pose2d poseEstimate = robotHardware.mecanumDrive.getPoseEstimate();
        this.time = robotHardware.getTime();
        this.x = poseEstimate.getX();
        this.y = poseEstimate.getY();
        this.heading = Math.toDegrees(poseEstimate.getHeading());
        this.encoder_frontRight = robotHardware.motorUtility.getEncoderValue(Motors.FRONT_RIGHT);
        this.encoder_frontLeft = robotHardware.motorUtility.getEncoderValue(Motors.FRONT_LEFT);
        this.encoder_backRight = robotHardware.motorUtility.getEncoderValue(Motors.BACK_RIGHT);
        this.encoder_backLeft = robotHardware.motorUtility.getEncoderValue(Motors.BACK_LEFT);
    }

    public TelemetryLog(double time, Pose2d pose, EnumMap<Motors, Integer> motorEncoders) {
        this.time = time;
        this.x = pose.getX();
        this.y = pose.getY();
        this.heading = Math.toDegrees(pose.getHeading());
        this.encoder_frontRight = motorEncoders.get(Motors.FRONT_RIGHT);
        this.encoder_frontLeft = motorEncoders.get(Motors.FRONT_LEFT);
        this.encoder_backRight = motorEncoders.get(Motors.BACK_RIGHT);
        this.encoder_backLeft = motorEncoders.get(Motors.BACK_LEFT);
    }

    public void setTime(double time) {
        this.time = time;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public void setEncoder_frontLeft(int encoder_frontLeft) {
        this.encoder_frontLeft = encoder_frontLeft;
    }

    public void setEncoder_frontRight(int encoder_frontRight) {
        this.encoder_frontRight = encoder_frontRight;
    }

    public void setEncoder_backLeft(int encoder_backLeft) {
        this.encoder_backLeft = encoder_backLeft;
    }

    public void setEncoder_backRight(int encoder_backRight) {
        this.encoder_backRight = encoder_backRight;
    }

    public void setCurrent_intake(double current_intake) {
        this.current_intake = current_intake;
    }

    public void setEncoder_launcher(int encoder_launcher) {
        this.encoder_launcher = encoder_launcher;
    }

    public void setRange_wall(double range_wall) {
        this.range_wall = range_wall;
    }

    public void setRange_back(double range_back) {
        this.range_back = range_back;
    }

    public void setRange_wobble(double range_wobble) {
        this.range_wobble = range_wobble;
    }

    public double getTime() {
        return time;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public int getEncoder_frontLeft() {
        return encoder_frontLeft;
    }

    public int getEncoder_frontRight() {
        return encoder_frontRight;
    }

    public int getEncoder_backLeft() {
        return encoder_backLeft;
    }

    public int getEncoder_backRight() {
        return encoder_backRight;
    }

    public double getCurrent_intake() {
        return current_intake;
    }

    public int getEncoder_launcher() {
        return encoder_launcher;
    }

    public double getRange_wall() {
        return range_wall;
    }

    public double getRange_back() {
        return range_back;
    }

    public double getRange_wobble() {
        return range_wobble;
    }
}
