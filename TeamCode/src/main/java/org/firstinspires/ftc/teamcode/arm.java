package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Arm Test")
public class arm extends LinearOpMode {
  private DcMotor Arm;
  
  @Override
  public void runOpMode() {
    double controlSensitivity = 0.25;

    Arm = hardwareMap.get(DcMotor.class, "arm");

    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        double spin = gamepad1.right_stick_y * 537.6; // 537.6t = 360d. 249t = arm is parallel to the ground

        Arm.setTargetPosition((int) spin); // Sets Target Tick Position
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0.5); // Sets Motor to go to position at 1 power.
        telemetry.addData("Motor was told to spin to " + spin + " and spun to " + Arm.getCurrentPosition(),null);
        telemetry.update();
      }
    }
  }
}
