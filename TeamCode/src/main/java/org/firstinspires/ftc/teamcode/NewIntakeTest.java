package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "[Test] New Intake System")
public class NewIntakeTest extends LinearOpMode {
    private CRServo Intake;

    @Override
    public void runOpMode() {
        Intake = hardwareMap.get(CRServo.class, "intake");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.a) {
                    Intake.setPower(-0.25);
                } else if (gamepad1.b) {
                    Intake.setPower(0.25);
                } else {
                    Intake.setPower(0);
                }
                telemetry.addData("Servo position: ", Intake.getPower());
                telemetry.update();
            }
        }
    }
}
