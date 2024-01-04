package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "[Park] Red Far")
public class RedFar extends OdometryEnabledNavigator {
    //private final OdometryEnabledNavigator navigator;
    private final ElapsedTime runtime = new ElapsedTime();

    private DcMotor arm;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized").setRetained(true);
        telemetry.update();

        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: %s", runtime.toString());
            driveForwardInches(30);
            try {
                TimeUnit.SECONDS.sleep(15);} catch (InterruptedException error) {while (opModeIsActive()) {telemetry.addData("Error: ", error);}return;}
            driveRightInches(90);
        }
    }
}