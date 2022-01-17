package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Blue top w/ white wh")
public class cvAuto3Blue extends LinearOpMode {

    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    Robot robot = new Robot();

    public void runOpMode() {
        robot.init(hardwareMap,telemetry);
        OpenCvWebcam webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera pipeline;

        pipeline = new camera();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("img analysis", pipeline.getAnalysis());
            telemetry.update();

            if (pipeline.getAnalysis() == camera.SkystonePosition.RIGHT) {
                telemetry.addData("right", "found item in right box");
                telemetry.update();
                robot.driveForwardDistance(.25, (int) (610/TICKSTOMMTRAVEL));
                robot.rotate(-45);
                //robot.lift(5000);
                robot.driveForwardDistance(.25, (int) (220/TICKSTOMMTRAVEL)); //random number
                //robot.clawOpen();
                robot.driveBackDistance(.25, (int) (150/TICKSTOMMTRAVEL));    //away from hub and to warehouse
                //robot.liftDown(5000);
                robot.rotate(90);
                robot.driveForwardDistance(1, (int) (2000/TICKSTOMMTRAVEL)); //go to white warehouse //go to white warehouse
            }
            else if (pipeline.getAnalysis() == camera.SkystonePosition.CENTER) {
                telemetry.addData("center", "found item in center box");
                telemetry.update();
                robot.driveForwardDistance(.25, (int) (610/TICKSTOMMTRAVEL));
                robot.rotate(-45);
                //robot.lift(5000);
                robot.driveForwardDistance(.25, (int) (220/TICKSTOMMTRAVEL)); //random number
                //robot.clawOpen();
                robot.driveBackDistance(.25, (int) (150/TICKSTOMMTRAVEL));    //away from hub and to warehouse
                //robot.liftDown(5000);
                robot.rotate(90);
                robot.driveForwardDistance(1, (int) (2000/TICKSTOMMTRAVEL)); //go to white warehouse //go to white warehouse
            }
            else if (pipeline.getAnalysis() == camera.SkystonePosition.LEFT) {
                telemetry.addData("left", "found item in left box");
                telemetry.update();
                robot.driveForwardDistance(.25, (int) (610/TICKSTOMMTRAVEL));
                robot.rotate(-45);
                //robot.lift(5000);
                robot.driveForwardDistance(.25, (int) (220/TICKSTOMMTRAVEL)); //random number
                //robot.clawOpen();
                robot.driveBackDistance(.25, (int) (150/TICKSTOMMTRAVEL));    //away from hub and to warehouse
                //robot.liftDown(5000);
                robot.rotate(90);
                robot.driveForwardDistance(1, (int) (2000/TICKSTOMMTRAVEL)); //go to white warehouse //go to white warehouse
            }
        }
    }
}
