package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Remote Operation")
public class main extends LinearOpMode {
    private DcMotor BackRight;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor Arm;

    /**
     * In order from front left to back right, specify doubles for each motor's power value.
     * @param frontLeftPower
     * @param frontRightPower
     * @param backLeftPower
     * @param backRightPower
     */
    private void unifiedSetPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        FrontLeft.setPower(frontLeftPower);
        FrontRight.setPower(frontRightPower);
        BackLeft.setPower(backLeftPower);
        BackRight.setPower(backRightPower);
    }

    /**
     * This function is executed when this OpMode is selected FrontRight the Driver Station.
     */
    @Override
    public void runOpMode() {
        double walk, strafe, turn, armTargetPosition;
        double controlSensitivity = 0.5;

        BackRight = hardwareMap.get(DcMotor.class, "b r");
        FrontRight = hardwareMap.get(DcMotor.class, "f r");
        BackLeft = hardwareMap.get(DcMotor.class, "b l");
        FrontLeft = hardwareMap.get(DcMotor.class, "f l");

        Arm = hardwareMap.get(DcMotor.class, "arm");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                walk = -gamepad1.right_stick_y * controlSensitivity; // Forward/Back
                strafe = gamepad1.left_stick_x * controlSensitivity; // Left/Right
                turn = -gamepad1.right_stick_x * controlSensitivity; // Turn

                if (Math.abs(walk) > Math.abs(strafe)) {
                    this.unifiedSetPower(-walk, walk,-walk, walk);
                    telemetry.addData("F/B", null);
                } else if (Math.abs(strafe) > Math.abs(turn)) {
                    this.unifiedSetPower(-strafe, strafe, strafe, -strafe); // RL is broken
                    telemetry.addData("R/L", null);
                } else {
                    this.unifiedSetPower(turn,turn, turn, turn);
                    telemetry.addData("Turning", null);
                }

                armTargetPosition = gamepad1.left_stick_y * 100 + Arm.getCurrentPosition();

                // The ordering of the following three lines is significant.
                Arm.setTargetPosition((int) armTargetPosition);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(controlSensitivity);

                telemetry.update();
            }
        }
    }
}
