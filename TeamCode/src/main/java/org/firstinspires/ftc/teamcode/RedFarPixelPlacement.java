package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "[Auto] Red Far")
public class RedFarPixelPlacement extends OdometryEnabledNavigator {
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
            waitSeconds(17);

            // Drive to backdrop
            driveRightInches(29);
            driveBackwardsInches(90);

            // Drop pixel
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive()) {
                arm.setPower(-1);
                arm.setTargetPosition(Constants.ArmConstants.armUpSetPoint);
            }
        }
    }
}