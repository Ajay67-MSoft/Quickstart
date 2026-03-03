package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.BLUE.BLUEFarEndRow2ToRow3;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.BLUE.BLUEFarEndRow3;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.BLUE.BLUEFarEndRow3ToRow2;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.BLUE.BLUEstructureRow1;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.BLUE.BLUEstructureRow1ToRow2;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.BLUE.BLUEstructureRow2;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.RED.REDFarEndRow2;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.RED.REDFarEndRow3;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.RED.REDFarEndRow3ToRow2;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.RED.REDstructureRow1;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.RED.REDstructureRow1ToRow2;
import org.firstinspires.ftc.teamcode.pedroPathing.AutoCode.RED.REDstructureRow2;

@Autonomous(name = "AUTO SELECTOR", group = "Auto")
public class AutoSelector extends SelectableOpMode {

    public AutoSelector() {
        super("Select Auto", s -> {

            s.folder("🔵 BLUE", blue -> {

                blue.folder("Structure", structure -> {
                    structure.add("Row 1 --> Park", BLUEstructureRow1::new);
                    structure.add("Row 1 --> 2", BLUEstructureRow1ToRow2::new);
                    structure.add("Row 2", BLUEstructureRow2::new);
                });

                blue.folder("Far End", far -> {
                    far.add("Row 3", BLUEFarEndRow3::new);
                    far.add("Row 3 --> 2", BLUEFarEndRow3ToRow2::new);
                    far.add("Row 2", BLUEFarEndRow2ToRow3::new);
                });

            });

            s.folder("🔴 RED", red -> {

                red.folder("Structure", structure -> {
                    structure.add("Row 1 --> Park", REDstructureRow1::new);
                    structure.add("Row 1 --> 2", REDstructureRow1ToRow2::new);
                    structure.add("Row 2", REDstructureRow2::new);
                });

                red.folder("Far End", far -> {
                    far.add("Row 3", REDFarEndRow3::new);
                    far.add("Row 3 --> 2", REDFarEndRow3ToRow2::new);
                    far.add("Row 2", REDFarEndRow2::new);
                });

            });

//            s.folder("🧪 TESTING", test -> {
//                test.add("Just Leave", LeaveOnlyAuto::new);
//            });

        });
    }
}