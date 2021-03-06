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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
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

@Autonomous(name="Pushbot: Auto Drive By Encoder Red", group="Pushbot")

public class PushbotAutoDriveByEncoder_Linear_Red_New extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    Gyroscope gyro;
    ColorSensor los_colores;
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1514 ;    // eg: MATRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.56 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    int i = 0;

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

        front_left = hardwareMap.dcMotor.get("front_left");
        front_right = hardwareMap.dcMotor.get("front_right");
        back_left = hardwareMap.dcMotor.get("back_left");
        back_right = hardwareMap.dcMotor.get("back_right");
        gyro = hardwareMap.get(Gyroscope.class, "gyro");

        los_colores = hardwareMap.colorSensor.get("colorSensor");
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          front_left.getCurrentPosition(),
                          back_left.getCurrentPosition(),
                          back_right.getCurrentPosition(),
                          front_right.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // Order of wheel params: back left, back right, front left, front right
        // To go forward: (bl- br0 fl+ fr0)
        // To go backward: (bl+ br0 fl- fr0)
        // To go right: (bl0 br- fl0 fr+)
        // To go left: (bl0 br+ fl0 fr-)
        // To turn left: (bl- br- fl- fr-)
        // To turn right: (bl+ br+ fl+ fr+)
        // Direction Options: forward, backward, left, right, turn

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        telemetry.addData("Path 1 Running", "");

          encoderDrive(DRIVE_SPEED,  60,  -60, -60, 60, 1000, "right");  // S1: Forward 47 Inches with 5 Sec timeout
          encoderDrive(DRIVE_SPEED,  12,  12, -12, -12, 1000, "backward");
//  encoderDrive(DRIVE_SPEED, 0, 15, 0, 15, 1000);
//        telemetry.addData("Path 1 ", "Complete");
//        telemetry.addData("Path 2 Running", "");
//        encoderDrive(TURN_SPEED,   -12, -12, -12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        telemetry.addData("Path 2 ", "Complete");
//        telemetry.addData("Path 3 Running", "");
//        encoderDrive(DRIVE_SPEED, 0, 10, 0, -10, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
//        telemetry.addData("Path 3 ", "Complete");

        sleep(10000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double backLeftInches, double backRightInches, double frontLeftInches,
                             double frontRightInches, double timeoutS, String direction) {
        int newLeftBackTarget;
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;



        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftBackTarget = back_left.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newRightBackTarget = back_right.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);
            newLeftFrontTarget = front_left.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newRightFrontTarget = front_right.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);

            back_left.setTargetPosition(newLeftBackTarget);
            back_right.setTargetPosition(newRightBackTarget);
            front_left.setTargetPosition(newLeftFrontTarget);
            front_right.setTargetPosition(newRightFrontTarget);

            if (direction == "forward" || direction == "backward") {
                back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                back_left.setPower(Math.abs(speed));
                front_left.setPower(Math.abs(speed));
            }
            else if (direction == "left" || direction == "right") {
                back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                back_right.setPower(Math.abs(speed));
                front_right.setPower(Math.abs(speed));
            }
            else if (direction == "turn") {
                back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                back_right.setPower(Math.abs(speed));
                front_right.setPower(Math.abs(speed));
                back_left.setPower(Math.abs(speed));
                front_left.setPower(Math.abs(speed));
            }
            // reset the timeout time and start motion.
            runtime.reset();

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (front_left.isBusy() && front_right.isBusy() && back_right.isBusy() && back_left.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftBackTarget,  newRightBackTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                                            back_right.getCurrentPosition(),
//                                            back_left.getCurrentPosition(),
//                                            front_right.getCurrentPosition(),
//                                            front_left.getCurrentPosition());
//                telemetry.update();
            }

            // Stop all motion;
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);


            // Turn off RUN_TO_POSITION
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}
