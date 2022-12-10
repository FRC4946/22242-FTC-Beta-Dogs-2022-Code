/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;

//@TeleOp
@Disabled
public class TestAutoWithTags3and4 extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private CRServo clawLeft = null;
    private CRServo clawRight = null;
    private CRServo stage2 = null;
    private CRServo stage3 = null;
    private CRServo stage1 = null;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C270 at default res
    // You will need to do your own calibration for other configurations!
    double fx = 964.9720651044612;
    double fy = 961.9626539642991;
    double cx = 302.93306542423454;
    double cy = 237.66599433445498;

    // UNITS ARE METERS
    double tagsize = 0.04;

    int ID_TAG_OF_INTEREST = 3; // Tag ID 18 from the 36h11 family
    ArrayList<Integer> TARGET_TAGS = new ArrayList<Integer>(2);

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        leftDrive  = hardwareMap.get(DcMotor .class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        clawLeft = hardwareMap.get(CRServo .class, "clawLeft");
        clawRight = hardwareMap.get(CRServo.class, "clawRight");
        stage1 = hardwareMap.get(CRServo.class, "stage1");
        stage2 = hardwareMap.get(CRServo.class, "stage2");
        stage3 = hardwareMap.get(CRServo.class, "stage3");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        clawLeft.setDirection(CRServo.Direction.FORWARD);
        clawRight.setDirection(CRServo.Direction.REVERSE);
        stage2.setDirection(CRServo.Direction.REVERSE);
        stage1.setDirection(CRServo.Direction.REVERSE);
        stage3.setDirection(CRServo.Direction.FORWARD);

        TARGET_TAGS.add(3);
        TARGET_TAGS.add(4);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            boolean tagFound = false;
            for (AprilTagDetection tag : currentDetections) {
                if (TARGET_TAGS.contains(tag.id)) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }
            if (tagFound) {
                telemetry.addLine("Tag Found!");
                tagToTelemetry(tagOfInterest);
            } else {
                telemetry.addLine("No Tags Found!");
            }
            telemetry.update();
            sleep(20);
        }

        telemetry.addLine("press start to start!");
        telemetry.update();
        while (!gamepad1.start) {
            sleep(20);
        }
        telemetry.clearAll();
        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        // Does the code
        if(tagOfInterest == null)
        {
            telemetry.addLine("No tags!");
        }
        else
        {
            if(tagOfInterest.id == TARGET_TAGS.get(0))
            {
                stage1.setPower(1);
                stage2.setPower(1);
                stage3.setPower(1);
                sleep(1000);
                stage1.setPower(-1);
                stage2.setPower(-1);
                stage3.setPower(-1);
                sleep(1000);
            }
            else if(tagOfInterest.id == TARGET_TAGS.get(1))
            {
                leftDrive.setPower(1);
                rightDrive.setPower(1);
                sleep(1000);
                leftDrive.setPower(-1);
                rightDrive.setPower(-1);
                sleep(1000);
            }
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}