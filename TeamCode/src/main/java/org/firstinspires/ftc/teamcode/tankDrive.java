package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="tankdrive")
public class tankDrive extends OpMode {

    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor bl = null;
    DcMotor br = null;

    public void init(){
        fl = hardwareMap.get(DcMotor.class, "front_left_wheel");
        fr = hardwareMap.get(DcMotor.class, "front_right_wheel");
        bl = hardwareMap.get(DcMotor.class, "back_left_wheel");
        br = hardwareMap.get(DcMotor.class, "back_right_wheel");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
    }

    public void init_loop(){

    }

    public void loop(){

        fl.setPower(-gamepad1.left_stick_y);
        bl.setPower(-gamepad1.left_stick_y);

        bl.setPower(-gamepad1.right_stick_y);
        br.setPower(-gamepad1.right_stick_y);

    }

}
