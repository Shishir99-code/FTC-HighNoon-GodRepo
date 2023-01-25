package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp

public class FullTeleOP extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftBack;
    DcMotor leftFront;
    DcMotor rightBack;
    DcMotor rightFront;
    Servo claw;
    DcMotor slide;
//    OpticalDistanceSensor odsSensor;


    @Override
    public void runOpMode() {
        // mapping motors to their correct names
        // left back motor
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);

        // left front motor
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        // right back motor
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // right front motor
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int slideInitial = slide.getCurrentPosition();
        slide.setTargetPosition(slideInitial);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        claw = hardwareMap.get(Servo.class, "claw");

//        odsSensor = hardwareMap.get(OpticalDistanceSensor.class, "sensor_ods");

        int SL_LOW = 875;
        int SL_MEDIUM = 1575;
        int SL_HIGH = 2175;

        double power = 0.7;

        // Gamepad 2
        boolean releasedA2 = true, releasedB2 = true, releasedX2 = true, releasedY2 = true;
        boolean releasedDU2 = true, releasedDD2 = true;


        waitForStart();
        while (opModeIsActive()) {

            // Initiallizing vairables
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;



            if (gamepad1.left_bumper) {
                leftFront.setPower(frontLeftPower * 1);
                leftBack.setPower(backLeftPower * 1);
                rightFront.setPower(frontRightPower * 1);
                rightBack.setPower(backRightPower * 1);
            } else{
                leftFront.setPower(frontLeftPower * power);
                leftBack.setPower(backLeftPower * power);
                rightFront.setPower(frontRightPower * power);
                rightBack.setPower(backRightPower * power);
            }


//            double MIN_DISTANCE = 5;
//            double MAX_DISTANCE = 10;


            if (gamepad2.a) {
                if (releasedA2) {
                    slide.setTargetPosition(slideInitial);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    releasedA2 = false;
                }
            } else if (!releasedA2) {
                releasedA2 = true;
            }

            if (gamepad2.b) {
                if (releasedB2) {
                    slide.setTargetPosition(slideInitial + SL_LOW);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(-1);
                    releasedB2 = false;
                }
            } else if (!releasedB2) {
                releasedB2 = true;
            }

            if (gamepad2.x) {
                if (releasedX2) {
                    slide.setTargetPosition(slideInitial + SL_MEDIUM);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    releasedX2 = false;
                }
            } else if (!releasedX2) {
                releasedX2 = true;
            }

            if (gamepad2.y) {
                if (releasedY2) {
                    slide.setTargetPosition(slideInitial + SL_HIGH);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    releasedY2 = false;
                }
            } else if (!releasedY2) {
                releasedY2 = true;
            }

            if (gamepad2.dpad_up) {
                if (releasedDU2) {
                    slide.setTargetPosition(slide.getCurrentPosition() + 150);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    releasedDU2 = false;
                }
            } else if (!releasedDU2) {
                releasedDU2 = true;
            }

            if (gamepad2.dpad_down) {
                if (releasedDD2) {
                    slide.setTargetPosition(slide.getCurrentPosition() - 105);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    releasedDD2 = false;
                }
            } else if (!releasedDD2) {
                releasedDD2 = true;
            }

            if ((gamepad2.right_bumper)) {
                claw.setPosition(1);
            } else {
                claw.setPosition(0.5);
            }

//            if (odsSensor.getRawLightDetected() >= MIN_DISTANCE && odsSensor.getRawLightDetected() <= MAX_DISTANCE) {
//                telemetry.addLine("Good to go");
//            } else if (odsSensor.getRawLightDetected() <= MIN_DISTANCE) {
//                telemetry.addLine("Move Forward!");
//            } else if (odsSensor.getRawLightDetected() >= MAX_DISTANCE) {
//                telemetry.addLine("Move Backward");
//            }

        }

    }
}