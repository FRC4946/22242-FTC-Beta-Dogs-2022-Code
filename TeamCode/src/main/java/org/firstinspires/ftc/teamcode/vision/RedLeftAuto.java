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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name="Red Left Auto")
@Disabled
public class RedLeftAuto extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    PIDController pid;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor liftMotor = null;
    private Servo liftServo = null;

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

    //int ID_TAG_OF_INTEREST = 3; // Tag ID 18 from the 36h11 family
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
        liftServo = hardwareMap.get(Servo.class, "claw");

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

        //TODO: make a
        TARGET_TAGS.add(1);
        TARGET_TAGS.add(2);
        TARGET_TAGS.add(3);

        pid = new PIDController(p, i, d);

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
                telemetry.addLine("Initialized Camera");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        telemetry.addLine("init");
        telemetry.update();

        waitForStart();

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            if (currentDetections.size() > 1)
            {
                telemetry.addLine("I see multiple tags, this might not be ideal");
            }
            boolean tagFound = false;
            for (AprilTagDetection tag : currentDetections) {
                if (TARGET_TAGS.contains(tag.id)) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }
            if (tagFound) {
                telemetry.addLine("Tag Found! ID:" + tagOfInterest.id);
                tagToTelemetry(tagOfInterest);
            } else {
                telemetry.addLine("No Tags Found!");
            }
            telemetry.update();
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
                int i = -1;
                int phase = 0;
                switch (phase) {
                    case 0:
                        pid.setSetpoint(-62.5);
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()));
                        } else {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 1:
                        //TODO: convert encoder moves to cm and back to rotation
                        //164cm from front of claw
                        //should be 154 to allow for turn to cone stack
                        driveToPosition(164, 0.5, Direction.FORWARDS);
                        if (!leftFrontDrive.isBusy()) {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 2:
                        //TODO: code the grabbing of the cone
                        //move up slightly
                        //grab cone
                        phase++;
                        break;
                    case 3:
                        //square up
                        pid.setSetpoint(90);
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate((getAngle())));
                        } else {
                            mecanumDrive(Direction.BACKWARDS, 1.0);
                            sleep(500);
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 4:
                        //rotate to pole
                        pid.setSetpoint(90); //TODO: get angle
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()));
                        } else {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 5:
                        //go to pole
                        driveToPosition(0.0, 0.75, Direction.FORWARDS); //TODO: get values
                        if (!leftFrontDrive.isBusy()) {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 6:
                        //TODO: get how to do claw
                        phase++;
                        break;
                    case 7:
                        pid.setSetpoint(-90);
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()));
                        } else {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 8:
                        driveToPosition(10, 0.75, Direction.FORWARDS); //TODO: get values
                        if (!leftFrontDrive.isBusy()) {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 9:
                        //turn to cone pile again
                        pid.setSetpoint(20); //TODO: get angle
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()));
                        } else {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 10:
                        if (TARGET_TAGS.get(2) == tagOfInterest.id && i == 3) {
                            phase++;
                        } else if (TARGET_TAGS.get(1) == tagOfInterest.id && i == 4) {
                            phase = 15;
                        } else if (TARGET_TAGS.get(0) == tagOfInterest.id && i == 5) {
                            phase = 19;
                        } else {
                            phase = 2;
                            i++;
                        }
                        break;
                    case 11:
                        //turn on angle so we fit
                        pid.setSetpoint(-45);
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()));
                        } else {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 12:
                        //go to center of patch
                        driveToPosition(33.0, 0.75, Direction.FORWARDS); //TODO: get values
                        if (!leftFrontDrive.isBusy()) {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 13:
                        //turn to location 3
                        pid.setSetpoint(90.0);
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()));
                        } else {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 14:
                        //go to location 3
                        driveToPosition(33.0, 0.75, Direction.FORWARDS); //TODO: get values
                        if (!leftFrontDrive.isBusy()) {
                            stopAndResetEncoders();
                            phase = 69;
                        }
                        break;
                    case 15:
                        //turn on angle so we fit
                        pid.setSetpoint(-45.0);
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()));
                        } else {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 16:
                        //go to center of patch
                        driveToPosition(013.0, 0.75, Direction.FORWARDS); //TODO: get values
                        if (!leftFrontDrive.isBusy()) {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 17:
                        //turn to location 2
                        pid.setSetpoint(90.00);
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()));
                        } else {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 18:
                        //go to location 2
                        driveToPosition(22.0, 0.75, Direction.FORWARDS); //TODO: get values
                        if (!leftFrontDrive.isBusy()) {
                            stopAndResetEncoders();
                            phase = 69;
                        }
                        break;
                    case 19:
                        //turn on angle so we fit
                        pid.setSetpoint(-45.00);
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()));
                        } else {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 20:
                        //go to center of patch
                        driveToPosition(11.0, 0.75, Direction.FORWARDS); //TODO: get values
                        if (!leftFrontDrive.isBusy()) {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 21:
                        //turn to location 3
                        pid.setSetpoint(90.000);
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()));
                        } else {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 22:
                        //go to location 3
                        driveToPosition(11.0, 0.75, Direction.FORWARDS); //TODO: get values
                        if (!leftFrontDrive.isBusy()) {
                            stopAndResetEncoders();
                            phase = 69;
                        }
                        break;
                    default:
                        telemetry.addLine("All done!");
                        break;
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

    //TODO: CONVERT ALL THIS SHIT TO FUCKIGN MECHANUM KILL ME AAAAAAA (HIGH PRIORITY)
    // Vouch
    void mecanumDrive(Direction direction, double power) {
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
    void driveToPosition(double target, double power, Direction direction) {
        int targetInt = (int)target;
        leftFrontDrive.setTargetPosition(targetInt);
        rightFrontDrive.setTargetPosition(targetInt);
        rightBackDrive.setTargetPosition(targetInt);
        leftBackDrive.setTargetPosition(targetInt);
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

    /**
     * what the fuck does this thing do
     */
    void stopMotors() {
        //Arms the battery (it's a bomb (fr))
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