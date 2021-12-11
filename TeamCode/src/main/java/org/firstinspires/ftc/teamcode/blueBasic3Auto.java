//THIS ONE STARTS FURTHER FROM TEAM WAREHOUSE
package org.firstinspires.ftc.teamcode;

import android.provider.DocumentsContract;

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

@Autonomous(name="blueBasic3Auto") //telling robot it is autonoumous
public class blueBasic3Auto extends LinearOpMode {

    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;


    Robot robot = new Robot();

    public void runOpMode(){

        robot.init(hardwareMap, telemetry);

        waitForStart();
        if(opModeIsActive()){
            robot.driveForwardDistance(.5, (int) (-180/TICKSTOMMTRAVEL)); //away from wall
            robot.rotate(93); //face warehouse
            robot.driveForwardDistance(1, (int) (-600/TICKSTOMMTRAVEL)); //into warehouse
        }
    }
}


