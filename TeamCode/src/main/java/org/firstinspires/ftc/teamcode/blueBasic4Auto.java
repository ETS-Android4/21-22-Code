//THIS ONE STARTS CLOSER TO TEAM STATION (DUCK)
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

@Autonomous(name="autoBlue4") //telling robot it is autonoumous
public class blueBasic4Auto extends LinearOpMode {

    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL / ENCODERTICKS) * GEARRATIO;


    Orientation angles;

    BNO055IMU imu;
    Robot robot = new Robot();

    public void runOpMode() {

        robot.init(hardwareMap, telemetry);


        waitForStart();
            if (opModeIsActive()) {
                robot.driveForwardDistance(.5, (int) (90 / TICKSTOMMTRAVEL)); //get away from wall
                robot.driveForwardDistance(.25, (int) (556 / TICKSTOMMTRAVEL)); //get away from wall

                robot.servo(-1, 4400);
                robot.driveBackDistance(.5, (int) (-120 / TICKSTOMMTRAVEL)); //go backwards
                robot.rotate(0); //aim for storage unit
                robot.driveForwardDistance(.5, (int) (-450 / TICKSTOMMTRAVEL)); //go to storage unit
                robot.rotate(-90);

                robot.servo(1, 4400);
                robot.driveBackDistance(.5, (int) (-120 / TICKSTOMMTRAVEL)); //go backwards
                robot.rotate(0); //aim for storage unit
                robot.driveForwardDistance(.5, (int) (-450 / TICKSTOMMTRAVEL)); //go to storage unit
                robot.rotate(90);

                robot.driveForwardDistance(.25, (int) (120 / TICKSTOMMTRAVEL));
            }
        }
    }