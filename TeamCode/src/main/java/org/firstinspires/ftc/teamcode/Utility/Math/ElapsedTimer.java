package org.firstinspires.ftc.teamcode.Utility.Math;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Utility.Math.ListMath.average;

public class ElapsedTimer extends ElapsedTime {

    private ElapsedTime period = new ElapsedTime();
    private List<Double> pastPeriods = new ArrayList<>();
    private final double historyLength;

    public ElapsedTimer() {
        this.historyLength = 200;
    }

    public ElapsedTimer(double historyLength) {
        this.historyLength = historyLength;
    }

    public double updatePeriodTime(){
        pastPeriods.add(period.seconds());
        period.reset();
        if (pastPeriods.size()>= 200) {
            pastPeriods.remove(0);
        }
        return average(pastPeriods);
    }

    public double getAveragePeriodSec() {
        return average(pastPeriods);
    }

    public double getMaxPeriodSec() {
        return Collections.max(pastPeriods);
    }

    public double getLastPeriodSec() {
        if (pastPeriods.size() != 0) {
            return pastPeriods.get(pastPeriods.size()-1);
        } else {
            return 0;
        }
    }

    public List<Double> getPastPeriods() {
        return pastPeriods;
    }

    public double getHistoryLength() {
        return historyLength;
    }
}
