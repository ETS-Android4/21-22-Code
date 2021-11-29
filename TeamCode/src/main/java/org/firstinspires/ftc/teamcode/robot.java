package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class robot {

    Orientation angles = null;
    HardwareMap hardwareMap;
    BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
   // parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

    public DcMotor fl = null;
    public DcMotor fr = null;
    public DcMotor bl = null;
    public DcMotor br = null;



    public void init(){
        fl = hardwareMap.dcMotor.get("front_left_motor");
        fr = hardwareMap.dcMotor.get("front_right_motor");
        bl = hardwareMap.dcMotor.get("back_left_motor");
        br = hardwareMap.dcMotor.get("back_right_motor");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void driveForward(double power){
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    public void driveBack(double power){
        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);
    }

    public void turn(double angle){

       // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, BNO055IMU.AngleUnit.DEGREES);
       // fl.setPower(power);
      //  fr.setPower(power);
      //  bl.setPower(power);
      //  br.setPower(power);
    }
}
