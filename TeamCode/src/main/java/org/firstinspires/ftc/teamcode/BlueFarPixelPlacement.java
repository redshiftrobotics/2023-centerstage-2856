package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "[Prototype] Blue Far")
public class BlueFarPixelPlacement extends OdometryEnabledNavigator {
    //private final OdometryEnabledNavigator navigator;
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor arm;

    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotor.class,"arm");
        telemetry.addData("Status", "Initialized").setRetained(true);
        telemetry.update();

        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: %s", runtime.toString());
            // Drive to backdrop
            driveLeftInches(29);
            try {TimeUnit.SECONDS.sleep(15);} catch (InterruptedException error) {while (opModeIsActive()) {telemetry.addData("Error: ", error);}return;}
            driveBackwardsInches(90);
            // Drop pixel
            // Target Position is -3306
            while (opModeIsActive()) {
                arm.setPower(-0.5);
                arm.setTargetPosition(-3306);
            }
        }
    }
}