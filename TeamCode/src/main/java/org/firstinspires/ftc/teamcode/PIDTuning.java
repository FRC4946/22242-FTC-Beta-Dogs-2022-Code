/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.  ftc.teamcode.PIDController;

// function just made to quickly tune the PID for rotation

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear Mecanum OpMode PID TUNING - BETA DOGS", group="Linear Opmode")
//@Disabled
public class PIDTuning extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor = null;
    private Servo claw = null;
    private PIDController pid = null;
    private BNO055IMU imu = null;

    private Orientation lastAngles = new Orientation();
    private double robotHeading = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front left drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front right drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "back left drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back right drive");
        liftMotor = hardwareMap.get(DcMotor.class, "lift motor");
        claw = hardwareMap.get(Servo.class, "claw"); // needs to be looked into, acting up, likely hardware.
        pid = new PIDController(0.055, 0.0, 0.0);



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);


            // bull (relating to claw servo)
            if (gamepad1.a)
            {
                pid.setSetpoint(90);
                double power = pid.calculate(getAngle());
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
            }
            else if (gamepad1.b)
            {
                pid.setSetpoint(0);
                double power = pid.calculate(getAngle());
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
            }
            else {
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
            }

            if (gamepad1.y)
            {
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftFrontDrive.setTargetPosition(1440);
                rightFrontDrive.setTargetPosition(1440);
                leftBackDrive.setTargetPosition(1440);
                rightBackDrive.setTargetPosition(1440);

                leftFrontDrive.setPower(0.5);
                rightFrontDrive.setPower(0.5);
                leftBackDrive.setPower(0.5);
                rightBackDrive.setPower(0.5);
                sleep(1000);
            }

            // bull (relating to elevator)
            if (gamepad1.dpad_up)
            {
                pid.setP(pid.getP() + 0.01);
                sleep(500);
            }
            else if (gamepad1.dpad_down)
            {
                pid.setP(pid.getP() - 0.01);
                sleep(500);
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addLine("Drive Motors");
            telemetry.addLine("Front Left: " + frontLeftPower);
            telemetry.addLine("Front Right: " + frontRightPower);
            telemetry.addLine("Back Left: " + backLeftPower);
            telemetry.addLine("Back Right: " + backRightPower);
            telemetry.addLine();
            telemetry.addLine("Elevator");
            telemetry.addLine("claw servo pos: " + claw.getPosition());
            telemetry.addLine("Lift Power: " + liftMotor.getPower());

            telemetry.addLine("PID P: " + pid.getP());

            telemetry.update();
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

}