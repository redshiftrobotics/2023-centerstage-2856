package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Print Arm Positon")
public class printDeg extends LinearOpMode {
    private DcMotor Arm;

    @Override
    public void runOpMode() {
        double controlSensitivity = 0.25;

        Arm = hardwareMap.get(DcMotor.class, "arm");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("::" + Arm.getCurrentPosition(),null);
                telemetry.update();
            }
        }
    }
}
