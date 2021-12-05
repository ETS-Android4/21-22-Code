package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.security.KeyStore;

@Autonomous(name="blueBasic4Auto") //telling robot it is autonoumous
public class blueBasic4Auto extends LinearOpMode {
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor bl = null;
    DcMotor br = null;
    CRServo crServo = null;

    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    Orientation angles;

    BNO055IMU imu;

    public void runOpMode(){
        fl = hardwareMap.dcMotor.get("front_left_motor");
        fr = hardwareMap.dcMotor.get("front_right_motor");
        bl = hardwareMap.dcMotor.get("back_left_motor");
        br = hardwareMap.dcMotor.get("back_right_motor");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        crServo = hardwareMap.crservo.get("crServo");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        waitForStart();
        if(opModeIsActive()){
            driveForwardDistance(.1, (int) (250/TICKSTOMMTRAVEL));
            rotate(90, false);
        }
    }

    private void driveForwardDistance(double power, int distance){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(distance);
        fr.setTargetPosition(distance);
        bl.setTargetPosition(distance);
        br.setTargetPosition(distance);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveForward(power);

        while(fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy()) {
        }

        stopDriving();
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    private void driveBackDistance(double power, int distance){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(-distance);
        fr.setTargetPosition(-distance);
        bl.setTargetPosition(-distance);
        br.setTargetPosition(-distance);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveBack(power);

        while(fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())
        {

        }
        stopDriving();
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void driveForward(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    private void driveBack(double power){
        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);
    }
    private void stopDriving() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    public void servo(double power, int time){
        crServo.setPower(power);
        sleep(time);
    }
    public void rotate(double wantedAngle, boolean turnRight){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startTime = System.nanoTime();
        double target = 0;
        if(turnRight) {
            target = 270;
        }
        if(!turnRight){
            target = wantedAngle;
        }


        double angle = 0;

        int totalTime = 0;

        double error = 90, P, I, D, integral = 0, derivative, correction, t, lastTime = 0, dt = 0.1, lastError = 90;
        double kp = .02;
        double ki = 0;
        double kd = .01;

        while(Math.abs(error) > 3 && opModeIsActive()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = angles.firstAngle;

            t = (double)System.nanoTime()/10;
            if (lastTime != 0){
                dt = t - lastTime;
            }

            error = target - angle;
            integral = ki * ((error - lastError) * dt);
            derivative = kd * ((error - lastError) / dt);

            P = kp * error;
            I = ki * integral;
            D = kd * derivative;

            correction = P + I + D;

            if(turnRight) {
                fl.setPower(correction);
                fr.setPower(-correction);
                bl.setPower(correction);
                br.setPower(-correction);
            }
            if(!turnRight){
                fl.setPower(-correction);
                fr.setPower(correction);
                bl.setPower(-correction);
                br.setPower(correction);
            }



            //System.out.println(P + " " + I + " " + D);
            System.out.println(error);
            System.out.println(angles);

            telemetry.addData("Dt", dt);
            telemetry.addData("Error", error);
            telemetry.addData("correction:", correction);
            telemetry.addData("top Left Power", fl.getPower());
            telemetry.addData("top Right Power", fr.getPower());
            telemetry.addData("bottom Left Power", bl.getPower());
            telemetry.addData("bottom Right Power", br.getPower());
            telemetry.addData("Angle", angle);
            telemetry.update();

            lastError = error;
            lastTime = t;
            totalTime += t;
        }
        telemetry.addData("Angle", angle);
        telemetry.addData("totalTime: ", totalTime * 10E-9);
        telemetry.update();

        System.out.println("angle: " + angle);
        System.out.println("totalTime: " + (totalTime * 10E-9));

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
