package org.firstinspires.ftc.teamcode.pedroPathing.DriverPractice;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.BLUE.Structure.BLUEstructureRow1;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.BLUE.Structure.BLUEstructureRow1ToRow2;

@Autonomous(name = "driver prac SELECTOR", group = "Auto")
public class DriverPracticeSelector extends SelectableOpMode {

    public DriverPracticeSelector() {
        super("Select Driver Practice Type", s -> {

            s.folder("🔵 BLUE", blue -> {
                    blue.add("Row 1 --> Park", BLUEstructureRow1::new);
                    blue.add("Row 1 --> 2", BLUEstructureRow1ToRow2::new);
                });
    });
    }
}