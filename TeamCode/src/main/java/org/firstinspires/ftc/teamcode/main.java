package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Remote Operation")
public class main extends LinearOpMode {
    /** Back right drive motor */
    private DcMotor BackRight;

    /** Front right drive motor */
    private DcMotor FrontRight;

    /** Back left drive motor */
    private DcMotor BackLeft;

    /** Front left drive motor */
    private DcMotor FrontLeft;

    /** Main arm Motor */
    private DcMotor Arm;

    /** Airplane Launcher release servo */
    private Servo AirplaneLauncher;

    /** Intake brush servo */
    private CRServo Intake;

    /**
     * In order from front left to back right, specify doubles for each motor's power value.
     * @param frontLeftPower Power level for the Front Left motor
     * @param frontRightPower Power level for the Front Right motor
     * @param backLeftPower Power level for the Rear Left motor
     * @param backRightPower Power level for the Rear Right motor
     */
    private void unifiedSetPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        FrontLeft.setPower(frontLeftPower);
        FrontRight.setPower(frontRightPower);
        BackLeft.setPower(backLeftPower);
        BackRight.setPower(backRightPower);
    }

    /**
     * This function is executed when this OpMode is selected FrontRight the Driver Station.
     * Throws InterruptedException because I don't want an ugly try catch block when we use sleep.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        BackRight = hardwareMap.get(DcMotor.class, "b r");
        FrontRight = hardwareMap.get(DcMotor.class, "f r");
        BackLeft = hardwareMap.get(DcMotor.class, "b l");
        FrontLeft = hardwareMap.get(DcMotor.class, "f l");
        Arm = hardwareMap.get(DcMotor.class, "arm");

        AirplaneLauncher = hardwareMap.get(Servo.class, "airplane");
        Intake = hardwareMap.get(CRServo.class, "intake");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Get the drive motor sensitivity from config. If the slowmode button is pressed down, half speed.
                final double slowModeModifier = (gamepad1.right_trigger == 1) ? 0.5: 1;
                final double sensitivity = config.ControlSensitivity * slowModeModifier;

                // Inputs
                final double rotation = -gamepad1.left_stick_x;
                final double strafe = gamepad1.right_stick_x;
                final double drive = gamepad1.left_stick_y;

                double leftFrontPower = (rotation - strafe + drive) * sensitivity;
                double rightFrontPower = (rotation - strafe - drive) * sensitivity;
                double leftBackPower = (rotation + strafe + drive) * sensitivity;
                double rightBackPower = (rotation + strafe - drive) * sensitivity;

                if (gamepad1.a) { // Handbrake
                    FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    unifiedSetPower(0, 0, 0, 0);
                } else {
                    FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    unifiedSetPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
                }

                Arm.setTargetPosition(
                    Arm.getCurrentPosition()
                    + 1024
                    * (int) gamepad2.left_stick_y
                );

                // The ordering of the following 2 functions is significant.
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(config.ArmSensitivity);

                if (gamepad2.a) {
                    Intake.setPower(-1);
                } else if (gamepad2.b) {
                    Intake.setPower(1);
                } else {
                    Intake.setPower(0);
                }

                if (gamepad2.y) {
                    AirplaneLauncher.setPosition(-0.5);
                }

                telemetry.addData("Arm Position: ", Arm.getTargetPosition());
                telemetry.update();
            }

            // Ending program
            AirplaneLauncher.setPosition(0.0);
        }
    }
}
