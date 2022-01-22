package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


//TELEOP WITH TWO CONTROLLERS GP1 IS DRIVE GP2 IS OTHER
@TeleOp(name = "Tele w/ 2")
public class teleopOff2 extends LinearOpMode {
    public void runOpMode() {

        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor lift = hardwareMap.dcMotor.get("lift_dcMotor");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        //bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        CRServo spinServo = hardwareMap.crservo.get("crServo");
        Servo clawServo = hardwareMap.servo.get("clawServo");

        double driveSpeed = 1;
        double servoSpinSpeed = 0;
        double servoLiftSpeed = 0;

        double servoClawPosOpen = 90; //FAKE NUMBERS PLACEHOLDERS CHANGE
        double servoClawPosClose = 0;

        telemetry.addData("Status", "Initalized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            //speed control gamepad1
            if (gamepad1.a) {
                if (driveSpeed == 1) { //if the current increment is 1, it'll switch to 0.5
                    driveSpeed = 0.5;
                } else { //if the current increment is not 1, it'll switch to 1
                    driveSpeed = 1;
                }
            }

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            fl.setPower(v1);
            fr.setPower(v2);
            bl.setPower(v3);
            br.setPower(v4);

            //spin carousel servo gamepad2
            if (gamepad2.right_bumper)
            {
                servoSpinSpeed = 1;

            }
            else if (gamepad2.left_bumper)
            {
                servoSpinSpeed = -1;

            }
            else if (gamepad2.right_bumper && gamepad2.left_bumper)
            {
                servoSpinSpeed = 0;
            }
            else
            {
                servoSpinSpeed = 0;

            }
            spinServo.setPower(servoSpinSpeed);

            //lift servo gamepad2
            lift.setPower(gamepad2.right_trigger);
            lift.setPower(-gamepad2.left_trigger);

            //lift servo gamepad2
            if(gamepad2.x){
                clawServo.setPosition(5);
            }
            if(gamepad2.b){
                clawServo.setPosition(0);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("right joystick y value", gamepad1.right_stick_y);
            telemetry.addData("right joystick x value", gamepad1.right_stick_x);
            telemetry.addData("left joystick y value", gamepad1.left_stick_y);
            telemetry.addData("left joystick x value", gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}
