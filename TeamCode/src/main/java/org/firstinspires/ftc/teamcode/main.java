package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.config;

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
     * math.clamp from java 21
     * @param value
     * @param min
     * @param max
     * @return
     */
    public static double mathClamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * This function is executed when this OpMode is selected FrontRight the Driver Station.
     */
    @Override
    public void runOpMode() {
        double walk, strafe, turn, armTargetPosition;
        double controlSensitivity = config.ControlSensitivity;

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
                } else if (Math.abs(strafe) > Math.abs(turn)) {
                    this.unifiedSetPower(-strafe, strafe, strafe, -strafe); // RL is broken (maybe)?
                } else {
                    this.unifiedSetPower(turn,turn, turn, turn);
                }

                armTargetPosition = mathClamp(gamepad1.left_stick_y * 1000 + Arm.getCurrentPosition(), config.ArmMinLimit, config.ArmMaxLimit);
                telemetry.addData("Arm target position = " + armTargetPosition, null);

                // The ordering of the following three lines is significant.
                Arm.setTargetPosition((int) armTargetPosition);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(config.ArmSensitivity);

                telemetry.addData("F/B", walk);
                telemetry.addData("L/R", strafe);
                telemetry.addData("T/N", turn);
                telemetry.addData("ARM TGT", armTargetPosition);
                telemetry.addData("ARM POS", Arm.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
