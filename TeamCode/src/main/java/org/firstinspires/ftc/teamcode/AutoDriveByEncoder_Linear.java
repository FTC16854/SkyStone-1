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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Drive By Encoder", group="ThrowawayCode")
//@Disabled
public class AutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 723.24 ;    // goBilda 5201

    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     INCHES_PER_FULL_WHEEL_ROTATION = WHEEL_DIAMETER_INCHES * Math.PI; //12.566"
    static final double     COUNTS_PER_INCH         = COUNTS_PER_MOTOR_REV / INCHES_PER_FULL_WHEEL_ROTATION;
    static final double     TURNING_RADIUS =  17.5; // MEASURE THE DISTANCE BETWEEN ROBOT TIRES FROM ONE SIDE TO THE OTHER

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.leftDrive.getCurrentPosition(),
                          robot.rightDrive.getCurrentPosition());
        telemetry.update();

        sleep(2000);

        telemetry.addData("INCHES PER FULL WHEEL ROTATION", "%7f", INCHES_PER_FULL_WHEEL_ROTATION);
        telemetry.addData("PULSES PER INCH", "%7f", COUNTS_PER_INCH);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderDriveStraight(DRIVE_SPEED, 36, false);
        encoderDriveStraight(DRIVE_SPEED, 12, true);
        encoderTurnRobot("left", 90);
        encoderDriveStraight(DRIVE_SPEED, 12, false);
        encoderTurnRobot("right", 180);
        encoderDriveStraight(DRIVE_SPEED, 12, false);

        robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void encoderDriveStraight(double speed, double inches, boolean moveInReverse) {
        telemetry.addData("encoderDriveStraight", inches);
        telemetry.addData("encoderDriveStraight", moveInReverse);
        telemetry.update();
        sleep(2000);   // optional pause after each move

        int countsToRotate = (int)(inches * COUNTS_PER_INCH);

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + countsToRotate;
            newRightTarget = robot.rightDrive.getCurrentPosition() + countsToRotate;
            if(!moveInReverse) {
                newLeftTarget = robot.leftDrive.getCurrentPosition() - countsToRotate;
                newRightTarget = robot.rightDrive.getCurrentPosition() - countsToRotate;
            }

            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            if(!moveInReverse) {
                telemetry.addData("Moving Forward", inches);
                telemetry.update();
               // sleep(2000);   // optional pause after each move


                while (opModeIsActive() &&
                        (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())
                        && (Math.abs(robot.leftDrive.getCurrentPosition()) <= Math.abs(newLeftTarget)) && (Math.abs(robot.rightDrive.getCurrentPosition()) <= Math.abs(newRightTarget))
                ) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.leftDrive.getCurrentPosition(),
                            robot.rightDrive.getCurrentPosition());
                    telemetry.update();
                }
            } else {
                telemetry.addData("Moving Backwards", inches);
                telemetry.update();
               // sleep(2000);   // optional pause after each move

                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();

                while (opModeIsActive() &&
                        (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())
                        && (robot.leftDrive.getCurrentPosition() >= (-1 * newLeftTarget)) && (robot.rightDrive.getCurrentPosition() >= (-1 * newRightTarget))
                ) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.leftDrive.getCurrentPosition(),
                            robot.rightDrive.getCurrentPosition());
                    telemetry.update();
                }
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
                //sleep(2000);   // optional pause after each move
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(3000);   // optional pause after each move
        }
    }

    public void encoderTurnRobot(String direction, double degrees) {
        double revolution = degrees / 360; //change degrees to revolutions 90=.25, 180=.5 etc...
        double inchesToTurn = (revolution * (TURNING_RADIUS * 2)); //multiply revolutions by diameter to find how many inches to turn
        int newTarget = (int)(inchesToTurn * COUNTS_PER_INCH); // convert inches to turn to pulses of encoder

        if(opModeIsActive()) {
            if (direction.toLowerCase() == "right") {
                robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + newTarget);
                // Turn On RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (opModeIsActive() &&
                        (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())
                        && (robot.leftDrive.getCurrentPosition() <= newTarget)
                ) {
                    telemetry.addData("Robot turning: ", direction);
                    telemetry.update();
                }

                robot.leftDrive.setPower(TURN_SPEED);
            } else {
                robot.rightDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + newTarget);
                // Turn On RUN_TO_POSITION
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.rightDrive.setPower(TURN_SPEED);
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newTarget , newTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
                //leep(2000);
                while (opModeIsActive() &&
                        (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())
                        && (robot.rightDrive.getCurrentPosition() <= newTarget)
                ) {
                    telemetry.addData("Robot turning: ", direction);
                    telemetry.update();
                }

                telemetry.addData("Path1", "Running to %7d :%7d", newTarget , newTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
                //sleep(2000);

            }

            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
