package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Tele w/ 1")

@Autonomous(name = "Tele w/ 1")

public class teleopOff extends LinearOpMode {
    public void runOpMode() {
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        CRServo spinServo = hardwareMap.crservo.get("crServo");
        CRServo liftServo = hardwareMap.crservo.get("liftServo");

        double driveSpeed = 1;
        double servoSpinSpeed = 0;
        double servoLiftSpeed = 0;
        boolean rightbumper;


        telemetry.addData("Status", "Initalized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            //if the "A" button is pressed, the speed will switch from 1 to 0.5 and vice versa
            if (gamepad1.a) {
                if (driveSpeed == 1) { //if the current increment is 1, it'll switch to 0.5
                    driveSpeed = 0.5;
                } else { //if the current increment is not 1, it'll switch to 1
                    driveSpeed = 1;
                }
            }

            fl.setPower(-gamepad1.left_stick_y * driveSpeed);
            fr.setPower(-gamepad1.right_stick_y * driveSpeed);
            bl.setPower(-gamepad1.left_stick_y * driveSpeed);
            br.setPower(-gamepad1.right_stick_y * driveSpeed);

            if (gamepad1.right_bumper) {
                servoSpinSpeed = 1;

                rightbumper = true;

            } else {
                servoSpinSpeed = 0;
                rightbumper = false;
            }

            if(gamepad1.left_bumper && !rightbumper){
                servoSpinSpeed = -1;
            } else {
                servoSpinSpeed = 0;
            }

            spinServo.setPower(servoSpinSpeed);


            liftServo.setPower(gamepad1.right_trigger);
            liftServo.setPower(-gamepad1.left_trigger);


            } else {
                servoSpinSpeed = 0;
            }
            spinServo.setPower(-gamepad1.right_trigger);
            spinServo.setPower(servoSpinSpeed);

            if(gamepad1.left_bumper){
                servoLiftSpeed = 1;
            } else {
                servoLiftSpeed = 0;
            }
            liftServo.setPower(-gamepad1.right_trigger);
            liftServo.setPower(servoLiftSpeed);


            telemetry.addData("Status", "Running");
            telemetry.addData("right joystick y value", gamepad1.right_stick_y);
            telemetry.addData("right joystick x value", gamepad1.right_stick_x);
            telemetry.addData("left joystick y value", gamepad1.left_stick_y);
            telemetry.addData("left joystick x value", gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}
