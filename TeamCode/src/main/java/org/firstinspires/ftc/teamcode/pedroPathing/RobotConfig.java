package org.firstinspires.ftc.teamcode.pedroPathing;

import android.os.Environment;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;

/**
 * A central configuration file for robot constants and tuning values.
 * Includes a mechanism to load lookup tables from a CSV file.
 */
public class RobotConfig {

    // --- SHOOTER CONSTANTS ---
    public static double SHOOTER_TICKS_PER_REV = 28;
    public static double SHOOTER_RPM_TOLERANCE = 100;    // Initial tolerance to start firing
    public static double SHOOTER_STALL_TOLERANCE = 350; // Leeway once shooting to prevent stalling
    public static double SHOOTER_IDLE_RPM = 2400;
    public static double SHOOT_DURATION_MS = 500;       // Minimum time to keep feeder running once triggered
    public static PIDFCoefficients SHOOTER_PIDF = new PIDFCoefficients(0.011, 0.0, 0.001, 13.5);

    // --- INTAKE & FEEDER CONSTANTS ---
    public static double INTAKE_MOTOR_POWER_INWARD = -2.0; // Effective power
    public static double INTAKE_MOTOR_POWER_OUTWARD = 1.0;
    public static double FEEDER_SERVO_POWER_SHOOT = 1.0;
    public static double FEEDER_SERVO_POWER_REVERSE = -1.0;
    public static double FEEDER_SERVO_POWER_HOLD = 0.0;

    // --- AUTO-AIM CONSTANTS ---
    public static double AIM_TOLERANCE_DEGREES = 0.75;
    public static double AIM_MIN_ROTATION_POWER = 0.06;
    public static double AIM_MAX_ROTATION_POWER = 0.2;

    // --- RPM LOOKUP TABLE ---
    // Default values if CSV is not found
    public static double[][] RPM_TABLE = {
            {2.8, 3100},
            {5.0, 2930},
            {6.37, 2600},
            {10.0, 2525},
            {13.6, 2375},
            {17.0, 2300}
    };

    /**
     * Loads the RPM lookup table from /sdcard/FIRST/shooter_lookup.csv if it exists.
     * Expected CSV format: ty,rpm
     */
    public static void loadLookupTable() {
        File file = new File(Environment.getExternalStorageDirectory(), "/FIRST/shooter_lookup.csv");
        if (!file.exists()) return;

        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            ArrayList<double[]> lines = new ArrayList<>();
            String line;
            while ((line = br.readLine()) != null) {
                String[] parts = line.split(",");
                if (parts.length == 2) {
                    lines.add(new double[]{
                            Double.parseDouble(parts[0].trim()),
                            Double.parseDouble(parts[1].trim())
                    });
                }
            }
            if (!lines.isEmpty()) {
                RPM_TABLE = lines.toArray(new double[0][0]);
            }
        } catch (Exception ignored) {
            // Fallback to default RPM_TABLE if file reading fails
        }
    }

    /**
     * Calculates required RPM based on Limelight vertical angle (ty)
     */
    public static double getInterpolatedRPM(double ty) {
        if (ty <= RPM_TABLE[0][0]) return RPM_TABLE[0][1];
        if (ty >= RPM_TABLE[RPM_TABLE.length - 1][0]) return RPM_TABLE[RPM_TABLE.length - 1][1];

        for (int i = 0; i < RPM_TABLE.length - 1; i++) {
            double tyLow = RPM_TABLE[i][0];
            double tyHigh = RPM_TABLE[i+1][0];
            if (ty >= tyLow && ty <= tyHigh) {
                double rpmLow = RPM_TABLE[i][1];
                double rpmHigh = RPM_TABLE[i+1][1];
                double percent = (ty - tyLow) / (tyHigh - tyLow);
                return rpmLow + percent * (rpmHigh - rpmLow);
            }
        }
        return RPM_TABLE[0][1];
    }
}
