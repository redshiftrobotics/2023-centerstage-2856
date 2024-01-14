package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "[Auto] Blue Far")
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
            waitSeconds(20);

            // Drive to backdrop
            driveLeftInches(28);
            driveBackwardsInches(83);

            // Drop pixel
            while (opModeIsActive()) {
                // The ordering herein is significant
                arm.setPower(-1);
                arm.setTargetPosition(Constants.ArmConstants.armUpSetPoint);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }
}