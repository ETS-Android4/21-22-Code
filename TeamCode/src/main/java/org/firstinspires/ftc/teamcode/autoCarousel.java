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


@Autonomous(name="auto Carousel") //telling robot it is autonoumous
public class autoCarousel extends LinearOpMode {

    double CIRCUMFERENCEOFWHEEL = 603.25; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    Robot Robot = new Robot(hardwareMap);
    BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
    public void runOpMode(){

        Robot.init();

        waitForStart();
        if(opModeIsActive()){
            Robot.driveForward(.1);
        }
    }

}
