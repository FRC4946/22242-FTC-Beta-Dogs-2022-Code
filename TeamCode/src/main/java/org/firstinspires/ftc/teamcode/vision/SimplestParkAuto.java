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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDTuning;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.PIDController;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name = "Simple Park Auto (STRIPPED NO CONE)", group = "!Beta Dogs Auto")
public class SimplestParkAuto extends LinearOpMode
{
    //TODO: EACH ROTATION OF THE WHEELS IS 49CM
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    PIDController pid;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor = null;
    private Servo claw = null;
    //private Servo clawSpinner = null;
    //private CRServo clawLeft = null;
    //private CRServo clawRight = null;
    private BNO055IMU imu = null;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C270 at default res
    // You will need to do your own calibration for other configurations!
    double fx = 985.4318958505312;
    double fy = 982.8510144217059;
    double cx = 353.64477664507393;
    double cy = 263.136868520393;

    double p = 0.1175;
    double i = 0.0;
    double d = 0.0;

    double robotHeading = 0;
    Orientation lastAngles = new Orientation();

    // UNITS ARE METERS
    double tagsize = 0.04;

    int phase = 0;

    int c = 0;

    int squareDist = 2500; //this is probably entirely incorrevt but its been adjusted for in other places

    int ID_TAG_OF_INTEREST = 3; // Tag ID 18 from the 36h11 family
    ArrayList<Integer> TARGET_TAGS = new ArrayList<Integer>(3);

    AprilTagDetection tagOfInterest = null;

    public enum Direction {
        FORWARDS, BACKWARDS, LEFT, RIGHT, ROTATE_LEFT, ROTATE_RIGHT, ROTATE_PID
    }
    @Override
    public void runOpMode()
    {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front left drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front right drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "back left drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back right drive");
        liftMotor = hardwareMap.get(DcMotor.class, "lift motor");
        claw = hardwareMap.get(Servo.class, "claw"); // needs to be looked into, acting up, likely hardware.

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        TARGET_TAGS.add(1);
        TARGET_TAGS.add(2);
        TARGET_TAGS.add(3);

        pid = new PIDController(p, i, d);

        pid.setTolerance(0.35);

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

        telemetry.addLine("Init. Waiting for start.");
        telemetry.update();
        waitForStart();

        telemetry.addLine("Getting latest detections");
        telemetry.update();
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        telemetry.addLine("Got latest detections");
        telemetry.update();

        sleep(1000);

        int tagIdOfInterest = -1;


        while (currentDetections.size() == 0 && c == 0 && opModeIsActive()) {
            currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (TARGET_TAGS.contains(tag.id)) {
                        tagOfInterest = tag;
                        tagFound = true;
                        telemetry.addLine("FOUND TAG!!!");
                        telemetry.update();
                        sleep(500);
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Tag Found! ID: " + tagOfInterest.id);
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("No Tags Found!");
                }
                telemetry.update();
                sleep(20);
            }
        }

        sleep(500);
        //telemetry.addLine("TAG ID IS " + tagOfInterest.id);
        //telemetry.update();

        sleep(500);

        // Does the code
        if(tagOfInterest == null)
        {
            telemetry.addLine("No tags!");
            telemetry.update();
        }
        else
        {

            tagToTelemetry(tagOfInterest);

            telemetry.update();

            // get away from the god damn wall (makes turning awkward);
            driveToPosition((int)Math.floor(-squareDist * 0.2), 1.0, Direction.FORWARDS);
            while (isAnyActive() && opModeIsActive())
            {
                sleep(5);
            }

            c++;
            stopAndResetEncoders();
            switch (tagOfInterest.id) {
                case 1:
                    //turn 90 to left
                    pid.setSetpoint(90);
                    PIDToTelemetry(pid);
                    telemetry.update();
                    while (!pid.atSetpoint() && opModeIsActive()) {
                        PIDToTelemetry(pid);
                        telemetry.update();
                        mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()), false);
                        sleep(10);
                    }
                    //drive to next square
                    driveToPosition((int)Math.floor(-squareDist * 1.1), 1.0, Direction.FORWARDS);
                    while (isAnyActive()) {
                        sleep(10);
                    }
                    stopAndResetEncoders();
                    //turn 90 ot right
                    pid.setSetpoint(0);
                    PIDToTelemetry(pid);
                    telemetry.update();
                    while (!pid.atSetpoint() && opModeIsActive()) {
                        mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()), false);
                        sleep(10);
                    }
                    //move to position
                    driveToPosition((int)Math.floor(-squareDist * 1.65), 1.0, Direction.FORWARDS);
                    while (isAnyActive()) {
                        sleep(10);
                    }
                    stopAndResetEncoders();
                    phase = 69;
                    break;
                case 2:
                    //drive forwards to position
                    driveToPosition((int)Math.floor(-squareDist * 1.65), 1.0, Direction.FORWARDS);
                    while (isAnyActive()) {
                        sleep(10);
                    }
                    stopAndResetEncoders();
                    phase = 69;
                    break;
                case 3:
                    //turn 90 to right
                    pid.setSetpoint(-90);
                    PIDToTelemetry(pid);
                    telemetry.update();
                    while (!pid.atSetpoint() && opModeIsActive()) {
                        mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()), false);
                        sleep(10);
                    }
                    stopAndResetEncoders();
                    //drive to next square
                    driveToPosition((int)Math.floor(-squareDist * 1.1), 1.0, Direction.FORWARDS);
                    while (isAnyActive()) {
                        sleep(10);
                    }
                    stopAndResetEncoders();
                    //turn 90 to left
                    pid.setSetpoint(0);
                    PIDToTelemetry(pid);
                    telemetry.update();
                    while (!pid.atSetpoint() && opModeIsActive()) {
                        mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()), false);
                        sleep(10);
                    }
                    //move to position
                    driveToPosition((int)Math.floor(-squareDist * 1.65), 1.0, Direction.FORWARDS);
                    while (isAnyActive()) {
                        sleep(10);
                    }
                    stopAndResetEncoders();
                    phase = 69;
                    break;
            }
        }
        stopMotors();
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

    void PIDToTelemetry(PIDController pid) {
        telemetry.addLine("PID Values")
                .addData("P", pid.getP())
                .addData("At Setpoint?", pid.atSetpoint())
                .addData("setpoint", pid.getSetpoint())
                .addData("getAngle()", getAngle());
    }


    void mecanumDrive(Direction direction, double power, boolean pos) {
        // leftFrontDrive.setPower();
        // rightFrontDrive.setPower();
        // leftBackDrive.setPower();
        // rightBackDrive.setPower();
        if (!pos) {
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        switch (direction) {
            case FORWARDS:
                leftFrontDrive.setPower(Math.abs(power));
                rightFrontDrive.setPower(Math.abs(power));
                leftBackDrive.setPower(Math.abs(power));
                rightBackDrive.setPower(Math.abs(power));
                break;
            case BACKWARDS:
                leftFrontDrive.setPower(Math.copySign(power, -1));
                rightFrontDrive.setPower(Math.copySign(power, -1));
                leftBackDrive.setPower(Math.copySign(power, -1));
                rightBackDrive.setPower(Math.copySign(power, -1));
                break;
            case LEFT:
                leftFrontDrive.setPower(Math.copySign(power, -1));
                rightFrontDrive.setPower(Math.copySign(power, 1));
                leftBackDrive.setPower(Math.copySign(power, 1));
                rightBackDrive.setPower(Math.copySign(power, -1));
                break;
            case RIGHT:
                leftFrontDrive.setPower(Math.copySign(power, 1));
                rightFrontDrive.setPower(Math.copySign(power, -1));
                leftBackDrive.setPower(Math.copySign(power, -1));
                rightBackDrive.setPower(Math.copySign(power, 1));
                break;
            case ROTATE_LEFT:
                leftFrontDrive.setPower(Math.copySign(power, -1));
                rightFrontDrive.setPower(Math.copySign(power, 1));
                leftBackDrive.setPower(Math.copySign(power, -1));
                rightBackDrive.setPower(Math.copySign(power, 1));
                break;
            case ROTATE_RIGHT:
                leftFrontDrive.setPower(Math.copySign(power, 1));
                rightFrontDrive.setPower(Math.copySign(power, -1));
                leftBackDrive.setPower(Math.copySign(power, 1));
                rightBackDrive.setPower(Math.copySign(power, -1));
                break;
            case ROTATE_PID:
                if (power > 0) {
                    leftFrontDrive.setPower(Math.copySign(power, 1)); // -1
                    rightFrontDrive.setPower(Math.copySign(power, -1)); // 1
                    leftBackDrive.setPower(Math.copySign(power, 1)); // -1
                    rightBackDrive.setPower(Math.copySign(power, -1)); // 1
                } else if (power < 0) {
                    leftFrontDrive.setPower(Math.copySign(power, -1)); // 1
                    rightFrontDrive.setPower(Math.copySign(power, 1)); // -1
                    leftBackDrive.setPower(Math.copySign(power, -1)); // 1
                    rightBackDrive.setPower(Math.copySign(power, 1)); // -1
                }
                break;
            default:
                telemetry.addLine("L + Ratio + You fell off + don't care + didn't ask + you're british + anime pfp");
                break;
        }
    }

    void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotHeading = 0;
    }
    // because I can't find any docs on the gyro and how it works, im ctrl + c and ctrl + v ing code
    // and so the ppl i copied from have angles set to + and - 180
    // not 360
    // because imu idfk
    double getAngle() {
        Orientation currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = currentAngle.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        robotHeading += deltaAngle;

        lastAngles = currentAngle;
        return robotHeading;
    }
    void stopAndResetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(0);
        rightFrontDrive.setTargetPosition(0);
        rightBackDrive.setTargetPosition(0);
        leftBackDrive.setTargetPosition(0);
    }
    void driveToPosition(int target, double power, Direction direction) {
        if (leftBackDrive.getTargetPosition() != target) {
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setTargetPosition(target);
            rightFrontDrive.setTargetPosition(target);
            rightBackDrive.setTargetPosition(target);
            leftBackDrive.setTargetPosition(target);
            if (    !(leftFrontDrive.getMode() == DcMotor.RunMode.RUN_TO_POSITION) ||
                    !(rightFrontDrive.getMode() == DcMotor.RunMode.RUN_TO_POSITION) ||
                    !(leftBackDrive.getMode() == DcMotor.RunMode.RUN_TO_POSITION) ||
                    !(rightBackDrive.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
            ) {
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        mecanumDrive(direction, power, true);
    }

    boolean isAnyActive() {
        return (leftBackDrive.isBusy() || leftFrontDrive.isBusy() || rightBackDrive.isBusy() || rightFrontDrive.isBusy()) && opModeIsActive();
    }

    void stopMotors() {
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}