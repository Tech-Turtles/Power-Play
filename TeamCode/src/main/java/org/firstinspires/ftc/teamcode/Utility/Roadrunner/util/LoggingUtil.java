package org.firstinspires.ftc.teamcode.Utility.Roadrunner.util;

import com.opencsv.CSVWriter;
import com.opencsv.bean.HeaderColumnNameMappingStrategy;
import com.opencsv.bean.StatefulBeanToCsv;
import com.opencsv.bean.StatefulBeanToCsvBuilder;
import com.opencsv.bean.comparator.LiteralComparator;
import com.opencsv.exceptions.CsvDataTypeMismatchException;
import com.opencsv.exceptions.CsvRequiredFieldEmptyException;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Utility functions for log files.
 */
public class LoggingUtil {
    public static final File ROAD_RUNNER_FOLDER =
            new File(AppUtil.ROOT_FOLDER + "/RoadRunner/");

    private static final long LOG_QUOTA = 25 * 1024 * 1024; // 25MB log quota for now

    private static void buildLogList(List<File> logFiles, File dir) {
        for (File file : dir.listFiles()) {
            if (file.isDirectory()) {
                buildLogList(logFiles, file);
            } else {
                logFiles.add(file);
            }
        }
    }

    private static void pruneLogsIfNecessary() {
        List<File> logFiles = new ArrayList<>();
        buildLogList(logFiles, ROAD_RUNNER_FOLDER);
        Collections.sort(logFiles, (lhs, rhs) ->
                Long.compare(lhs.lastModified(), rhs.lastModified()));

        long dirSize = 0;
        for (File file: logFiles) {
            dirSize += file.length();
        }

        while (dirSize > LOG_QUOTA) {
            if (logFiles.size() == 0) break;
            File fileToRemove = logFiles.remove(0);
            dirSize -= fileToRemove.length();
            //noinspection ResultOfMethodCallIgnored
            fileToRemove.delete();
        }
    }

    /**
     * Obtain a log file with the provided name
     */
    public static File getLogFile(String name) {
        //noinspection ResultOfMethodCallIgnored
        ROAD_RUNNER_FOLDER.mkdirs();

        pruneLogsIfNecessary();

        return new File(ROAD_RUNNER_FOLDER, name);
    }


    public static void saveTelemetryLogListToFile(File file, List<TelemetryLog> telemetryLogs) {
        // Create FileWriter from File object
        try {
            FileWriter fw = new FileWriter(file);
            HeaderColumnNameMappingStrategy<TelemetryLog> strategy = new HeaderColumnNameMappingStrategy<>();
            strategy.setType(TelemetryLog.class);
            strategy.setColumnOrderOnWrite(new LiteralComparator<>(TelemetryLog.fieldOrder));

            StatefulBeanToCsv sbc = new StatefulBeanToCsvBuilder(fw)
                    .withSeparator(CSVWriter.DEFAULT_SEPARATOR)
                    .withMappingStrategy(strategy)
                    .build();
            sbc.write(telemetryLogs);
            fw.close();
        } catch (IOException | CsvRequiredFieldEmptyException | CsvDataTypeMismatchException e) {
            e.printStackTrace();
        }
    }
}
