package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Arm Test")
public class arm extends LinearOpMode {
  private DcMotor DangerouslyControlArm;
  
  @Override
  public void runOpMode() {
    double spin;
    double controlSensitivity = 0.5;

    DangerouslyControlArm = hardwareMap.get(DcMotor.class, "arm");

    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        spin = gamepad1.right_stick_y * controlSensitivity;
        DangerouslyControlArm.setPower(spin);
        telemetry.update();
      }
    }
  }
}
