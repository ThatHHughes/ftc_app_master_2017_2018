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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwarePushbot;
/**
 *
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop v0.9", group="Pushbot")
public class PushbotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
  	DcMotor front_left;
  	DcMotor front_right;
  	DcMotor back_left;
  	DcMotor back_right;

    double maxSpeed = 0.5;

    public void init() {

      
        robot.init(hardwareMap);
		front_left = hardwareMap.dcMotor.get("front_left");
      	front_right = hardwareMap.dcMotor.get("front_right");
      	back_left = hardwareMap.dcMotor.get("back_left");
      	back_right = hardwareMap.dcMotor.get("back_right");
//        test_1 = hardwareMap.dcMotor.get("test_1");
//        test_2 = hardwareMap.dcMotor.get("test_2");
        telemetry.addData("Status:", "hwMap is good.");

    }



    public void init_loop() {
    }


    public void start() {

        telemetry.addData("Status:", "Started.");
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }



    public void loop() {

//      	front_left.setPower(-gamepad1.right_stick_y*maxSpeed);
//      	front_right.setPower(-gamepad1.right_stick_x*maxSpeed);
//      	back_right.setPower(-gamepad1.left_stick_x*maxSpeed);
//      	back_left.setPower(-gamepad1.left_stick_y*maxSpeed);

        double yspeed = gamepad1.left_stick_y;
        telemetry.addData("YSpeed: ", yspeed);
        double xspeed = gamepad1.left_stick_x;
        telemetry.addData("XSpeed: ", xspeed);
        double turnspeed = gamepad1.right_stick_x;

        if (gamepad1.left_stick_y > 0)
        {
            front_left.setPower(-yspeed*maxSpeed);
            back_left.setPower(yspeed*maxSpeed);
        }
        else if (gamepad1.left_stick_y < 0)
        {
            front_left.setPower(-yspeed*maxSpeed);
            back_left.setPower(yspeed*maxSpeed);
        }
        else if (gamepad1.left_stick_x > 0)
        {
            back_right.setPower(-xspeed*maxSpeed);
            front_right.setPower(xspeed*maxSpeed);
        }
        else if (gamepad1.left_stick_x < 0)
        {
            back_right.setPower(-xspeed*maxSpeed);
            front_right.setPower(xspeed*maxSpeed);
        }
        else if (gamepad1.right_stick_x < 0)
        {
            front_left.setPower(turnspeed*maxSpeed);
            back_left.setPower(turnspeed*maxSpeed);
            back_right.setPower(turnspeed*maxSpeed);
            front_right.setPower(turnspeed*maxSpeed);
        }
        else if (gamepad1.right_stick_x > 0)
        {
            front_left.setPower(turnspeed*maxSpeed);
            back_left.setPower(turnspeed*maxSpeed);
            back_right.setPower(turnspeed*maxSpeed);
            front_right.setPower(turnspeed*maxSpeed);
        }
        else {
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);
        }

//        if (gamepad1.left_stick_y > 0)
//        {
//            front_left.setPower(gamepad1.left_stick_y);
//
//        }
//        if (gamepad1.left_stick_y < 0)
//        {
//            front_right.setPower(gamepad1.left_stick_y);
//
//        }
//        if (gamepad1.left_stick_x > 0)
//        {
//
//            back_right.setPower(gamepad1.left_stick_x);
//        }
//        if (gamepad1.left_stick_x < 0)
//        {
//
//            back_left.setPower(gamepad1.left_stick_x);
//        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
      telemetry.addData("Status:", "Stopped.");
    }
}
