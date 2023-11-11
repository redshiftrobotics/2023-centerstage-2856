package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config;

@TeleOp(name = "Remote Operation")
public class main extends LinearOpMode {
    private DcMotor BackRight;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor Arm;
    private CRServo AirplaneLauncher;

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
        BackRight = hardwareMap.get(DcMotor.class, "b r");
        FrontRight = hardwareMap.get(DcMotor.class, "f r");
        BackLeft = hardwareMap.get(DcMotor.class, "b l");
        FrontLeft = hardwareMap.get(DcMotor.class, "f l");
        Arm = hardwareMap.get(DcMotor.class, "arm");

        AirplaneLauncher = hardwareMap.get(CRServo.class, "airplane");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                final double axial = gamepad1.left_stick_y * config.ControlSensitivity; // lat
                final double lateral = -gamepad1.right_stick_x * config.ControlSensitivity; // axial
                final double yaw = gamepad1.right_stick_x * config.ControlSensitivity;// Rotation

                double leftFrontPower = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower = axial - lateral + yaw;
                double rightBackPower = axial + lateral - yaw;

                unifiedSetPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

                Arm.setTargetPosition(
                        Arm.getCurrentPosition()
                                + 1024
                                * (int) gamepad2.left_stick_y
                );

                // works according to tech toolbox Arm.scaleRange(0, 1);
                // The ordering of the following 2 functions is significant
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(config.ArmSensitivity);

                telemetry.addData("Arm Position: ", Arm.getTargetPosition());
                telemetry.update();
            }
        }
    }
}
