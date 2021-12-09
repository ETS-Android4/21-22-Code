//THIS ONE STARTS CLOSER TO TEAM WAREHOUSE
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

@Autonomous(name="autoRed2") //telling robot it is autonoumous
public class redBasic2Auto extends LinearOpMode {
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

    Robot robot = new Robot();

    public void runOpMode(){

        robot.init(hardwareMap, telemetry);

        waitForStart();
        if(opModeIsActive()){
            robot.driveForwardDistance(.5, (int) (205/TICKSTOMMTRAVEL)); //get away from wall
            robot.rotate(90);
            robot.driveForwardDistance(.5, (int) (762/TICKSTOMMTRAVEL)); //get away from wall
            robot.servo(1, 5000);
            robot.rotate(90);
            robot.driveForwardDistance(.5, (int) (914.4/TICKSTOMMTRAVEL));
        }
    }
}
