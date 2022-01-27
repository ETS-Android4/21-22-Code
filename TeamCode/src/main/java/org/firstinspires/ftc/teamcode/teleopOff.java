package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


//TELEOP WITH 1 CONTROLLER
@TeleOp(name = "Tele w/ 1")
public class teleopOff extends LinearOpMode {
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
        //CRServo liftServo = hardwareMap.crservo.get("liftServo");

        Servo clawServo = hardwareMap.servo.get("clawServo");

        double driveSpeed = 1;
        double servoSpinSpeed = 0;
        double liftSpeed =  0;


        telemetry.addData("Status", "Initalized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                if (driveSpeed == 1) { //if the current increment is 1, it'll switch to 0.5
                    driveSpeed = 0.5;
                } else { //if the current increment is not 1, it'll switch to 1
                    driveSpeed = 1;
                }
            }

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y); //finds hypotenuse (power of each motor)
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4; //finds angle of robot subtracted by pi/4 bc
                                                                                                         //it "shifts" the powers to each motor CW
            double rightX = -gamepad1.right_stick_x; //for rotating w/ right stick

            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;
            /*
                r is mulitplier for power
                math.cos is used for fl and br bc fl&br are used to go diagonal top right, if you want to go faster to the right apply more power to
                those motors so closer joystick is to x axis faster robot go to that direction
                math.sin is used for same reason as ^ but to go faster forward/backwards
             */

            fl.setPower(v1);
            fr.setPower(v2);
            bl.setPower(v3);
            br.setPower(v4);

            if (gamepad1.right_bumper)
            {
                servoSpinSpeed = 1;

            }
            else if (gamepad1.left_bumper)
            {
                servoSpinSpeed = -1;

            }
            else if (gamepad1.right_bumper && gamepad1.left_bumper)
            {
                servoSpinSpeed = 0;
            }
            else
            {
                servoSpinSpeed = 0;

            }

            //spin carousel servo gamepad
            spinServo.setPower(servoSpinSpeed);

            if((gamepad1.right_trigger > 0) && gamepad1.left_trigger == 0){
                lift.setPower(-gamepad1.right_trigger);
            }
            if((gamepad1.left_trigger > 0) && gamepad1.right_trigger == 0){
                lift.setPower(gamepad1.left_trigger);
            }
            if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
                lift.setPower(0);
            }


            if (gamepad1.x) {
                clawServo.setPosition(5);
            }
            if (gamepad1.b) {
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
