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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Start Left Park", group="Linear Opmode")
//@Disabled
public class StartL extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotorSimple intakeMotors = null;
    private DcMotorSimple liftMotor = null;
    private Servo armServo = null;
    private Servo clawServo = null;
    private Servo foundationMoverL;
    private Servo foundationMoverR;
    private BNO055IMU               imu;
    private Orientation lastAngles = new Orientation ();
    private double heading;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        leftRear = hardwareMap.get(DcMotor.class, "BL");
        rightRear = hardwareMap.get(DcMotor.class, "BR");
        intakeMotors = hardwareMap.get(DcMotorSimple.class, "im");
        liftMotor = hardwareMap.get(DcMotorSimple.class, "lm");
        //armServo = hardwareMap.get(Servo.class, "as");
        //clawServo = hardwareMap.get(Servo.class,"cs");
        foundationMoverL = hardwareMap.get(Servo.class, "foundationmoverL");
        foundationMoverR = hardwareMap.get (Servo.class, "foundationmoverR");

        foundationMoverL.setDirection (Servo.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        // Wait for the game to start (driver presses PLAY)


        waitForStart();
        runtime.reset();

        //Auto Starts
        while (opModeIsActive()) {
            moveRobot(0.5, 180);
            sleep(3000);
            stopRobot();

         heading = getAngle();

         headingOffsetHolder.setOffset(heading);
         break;
            }
        }

    public void moveRobot(double speed, double angle) {
        speed = -speed;
        double robotAngle = Math.toRadians(angle) - Math.PI / 4;
        double Rotation = gamepad1.right_stick_x;
        final double lf = speed * Math.cos(robotAngle) + Rotation;
        final double rf = speed * Math.sin(robotAngle) - Rotation;
        final double lr = speed * Math.sin(robotAngle) + Rotation;
        final double rr = speed * Math.cos(robotAngle) - Rotation;
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
    }


    public void stopRobot() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

    }
    public void foundationMover(boolean Up){
        double downPosition = 0.5;
        double upPosition = 0;
        if (Up == false ) {
            foundationMoverL.setPosition(downPosition);
            foundationMoverR.setPosition(downPosition + 0.05);
        }else {
            foundationMoverR.setPosition(upPosition);
            foundationMoverL.setPosition(upPosition);
        }
    }

    public double getAngle() {
        // Z axis is returned as 0 to +180 or 0 to -180 rolling to -179 or +179 when passing 180
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        return heading;
    }


}