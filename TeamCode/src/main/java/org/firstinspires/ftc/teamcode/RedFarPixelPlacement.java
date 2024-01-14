package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "[Prototype] Red Far")
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
            // Drive to backdrop
            driveRightInches(29);
            waitSeconds(15);
            driveBackwardsInches(90);
            // Drop pixel
            // Target Position is -3306
            arm.setPower(-1);
            arm.setTargetPosition(-3306);
            while (opModeIsActive()) {
                arm.setPower(-0.5);
                arm.setTargetPosition(-3306);
            }
        }
    }
}