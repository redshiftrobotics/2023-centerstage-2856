package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "[Test] Airplane Launcher")
public class AirplaneLauncherTest extends LinearOpMode {
    private Servo AirplaneLauncher;

    @Override
    public void runOpMode() {
        AirplaneLauncher = hardwareMap.get(Servo.class, "airplane");

        waitForStart();
        if (opModeIsActive()) {
            AirplaneLauncher.setPosition(0.5);
            try {
                // jesse why the hell does this need a try catch bloc
                TimeUnit.SECONDS.sleep(1);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            AirplaneLauncher.setPosition(0.0);
            while (opModeIsActive()) {
                telemetry.addData("Servo position: ", AirplaneLauncher.getPosition());
                telemetry.update();
            }
        }
    }
}
