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
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.PIDController;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name="Simple Park Auto")
public class SimpleParkAuto extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    PIDController pid;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
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

    double p = 0.055;
    double i = 0.0;
    double d = 0.0;

    double robotHeading = 0;
    Orientation lastAngles = new Orientation();

    // UNITS ARE METERS
    double tagsize = 0.04;

    int squareDist = 1863; //60cm / 49 * 360 * 4

    int phase = 0;

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
        ////clawLeft = hardwareMap.get(CRServo.class, "left claw");
        ////clawRight = hardwareMap.get(CRServo.class, "right claw");
        ////clawSpinner = hardwareMap.get(Servo.class, "claw spinner");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

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
        pid.setTolerance(2);

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

        if (currentDetections.size() != 0 && (pid.getSetpoint() == 0 || phase == 0)) {
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

        telemetry.clearAll();




        // Does the code
        if(tagOfInterest == null)
        {
            telemetry.addLine("No tags!");
            telemetry.update();
            sleep(5000);
        }
        else
        {
            switch (tagOfInterest.id) {
                case 1:
                    //turn 90 to left
                    pid.setSetpoint(-90);
                    while (!pid.atSetpoint()) {
                        telemetry.addLine("rotate pid target -90\nPID Power: " + pid.calculate(getAngle()) + "\nAngle: " + getAngle());
                        telemetry.addLine("At set point? " + pid.atSetpoint());
                        telemetry.update();
                        mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()));
                        opModeIsActive();
                    }
                    //drive to next square
                    //driveToPosition(squareDist, 0.5, Direction.FORWARDS); //TODO: get target, 1440 = 360 deg
                    // the above doesn't want to work

                    leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + squareDist);
                    leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + squareDist);
                    rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + squareDist);
                    rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + squareDist);

                    leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftBackDrive.setPower(0.4);
                    leftFrontDrive.setPower(0.4);
                    rightBackDrive.setPower(0.4);
                    rightFrontDrive.setPower(0.4);

                    sleep(1000);

                    while (!leftBackDrive.isBusy())
                    {
                        telemetry.addLine("DRIVING TO TARGET");
                        telemetry.update();
                        sleep(10);
                    }

                    // stop all motor

                    leftBackDrive.setPower(0.0);
                    leftFrontDrive.setPower(0.0);
                    rightBackDrive.setPower(0.0);
                    rightFrontDrive.setPower(0.0);


                    sleep(15000); // temp

//                    while (leftBackDrive.isBusy()) {
//                        telemetry.addLine("Waiting for leftBackDrive?");
//                        telemetry.update();
//                        opModeIsActive();
//                        sleep(10);
//                    }
                    //turn 90 ot right
                    pid.setSetpoint(0);
                    while (!pid.atSetpoint()) {
                        mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()));
                        telemetry.addLine("Rotating to second target");
                        telemetry.update();
                        opModeIsActive();
                    }
                    //move to position
                    driveToPosition(squareDist, 0.5, Direction.FORWARDS); //TODO: get target, 1440 = 360 deg
                    while (leftBackDrive.isBusy()) {
                        sleep(10);
                        opModeIsActive();
                    }
                    phase = 69;
                    break;
                case 2:
                    //drive forwards to position
                    driveToPosition(squareDist, 0.5, Direction.FORWARDS); //TODO: get target, 1440 = 360 deg
                    while (leftBackDrive.isBusy()) {
                        sleep(10);
                        opModeIsActive();
                    }
                    phase = 69;
                    break;
                case 3:
                    //turn 90 to right
                    pid.setSetpoint(90);
                    while (!pid.atSetpoint()) {
                        mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()));
                        opModeIsActive();
                    }
                    //drive to next square
                    driveToPosition(squareDist, 0.5, Direction.FORWARDS);
                    while (leftBackDrive.isBusy()) {
                        sleep(10);
                        opModeIsActive();
                    }
                    //turn 90 to left
                    pid.setSetpoint(0);
                    while (!pid.atSetpoint()) {
                        mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()));
                        opModeIsActive();
                    }
                    //move to position
                    driveToPosition(squareDist, 0.5, Direction.FORWARDS);
                    while (leftBackDrive.isBusy()) {
                        sleep(10);
                        opModeIsActive();
                    }
                    phase = 69;
                    break;
            }
        }


        /* nice */
        while (phase != 69) { opModeIsActive(); };
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


    void mecanumDrive(Direction direction, double power) {
        // leftFrontDrive.setPower();
        // rightFrontDrive.setPower();
        // leftBackDrive.setPower();
        // rightBackDrive.setPower();
        switch (direction) {
            case FORWARDS:
                leftFrontDrive.setPower(Math.abs(power));
                rightFrontDrive.setPower(-Math.abs(power));
                leftBackDrive.setPower(Math.abs(power));
                rightBackDrive.setPower(-Math.abs(power));
                telemetry.addLine("SET LEFT BACK POWER TO: " + (Math.abs(power)));
                telemetry.update();
                sleep(5000);
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
                    leftFrontDrive.setPower(Math.copySign(power, -1));
                    rightFrontDrive.setPower(Math.copySign(power, 1));
                    leftBackDrive.setPower(Math.copySign(power, -1));
                    rightBackDrive.setPower(Math.copySign(power, 1));
                } else if (power < 0) {
                    leftFrontDrive.setPower(Math.copySign(power, 1));
                    rightFrontDrive.setPower(Math.copySign(power, -1));
                    leftBackDrive.setPower(Math.copySign(power, 1));
                    rightBackDrive.setPower(Math.copySign(power, -1));
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
    }
    void driveToPosition(int target, double power, Direction direction) {
        leftFrontDrive.setTargetPosition(target);
        rightFrontDrive.setTargetPosition(-target);
        rightBackDrive.setTargetPosition(-target);
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
        mecanumDrive(direction, power);
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