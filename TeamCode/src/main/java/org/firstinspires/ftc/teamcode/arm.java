package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "[Test] Arm Tick Measurement")
public class arm extends LinearOpMode {
  private DcMotor Arm;
  
  @Override
  public void runOpMode() {
    Arm = hardwareMap.get(DcMotor.class, "arm");

    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        telemetry.addData("ARM POS", Arm.getCurrentPosition());
        telemetry.update();
      }
    }
  }
}
