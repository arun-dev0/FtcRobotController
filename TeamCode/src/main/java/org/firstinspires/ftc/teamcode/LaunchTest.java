package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "LaunchTest")
public final class LaunchTest extends LinearOpMode {
    public static double launchPower = 0.5;
    public static double intakePower = 0.5;
    public static double feedPower = 1;


    public DcMotorEx intake;
    public DcMotorEx launcher;
    public CRServo left_feeder;
    public CRServo right_feeder;


        @Override
        public void runOpMode() {
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            left_feeder = hardwareMap.get(CRServo.class, "left_feeder");
            right_feeder = hardwareMap.get(CRServo.class, "right_feeder");
            boolean onflag = true;

            waitForStart();

            while (opModeIsActive()) {
                if (gamepad1.left_bumper && onflag) {
                    intake.setPower(intakePower);
                    onflag = false;
                }
                if (!gamepad1.left_bumper && !onflag) {
                    intake.setPower(0);
                    onflag = true;
                }
                if (gamepad1.right_bumper) {
                    for (int i = 0; i < 4; i++) {
                        left_feeder.setPower(feedPower);
                        right_feeder.setPower(-feedPower);
                        sleep(200);
                        right_feeder.setPower(0);
                        left_feeder.setPower(0);
                        sleep(200);
                        }
                    }
                    left_feeder.setPower(0);
                    right_feeder.setPower(0);
                }
                if (gamepad1.left_trigger > 0) {
                    launcher.setPower(-launchPower);
                } else {
                    launcher.setPower(0);
                }
            }
    }
