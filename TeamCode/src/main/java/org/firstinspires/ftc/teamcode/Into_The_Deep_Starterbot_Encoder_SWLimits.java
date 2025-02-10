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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Into_The_Deep_Starterbot_EncoderSWLimits", group = "Linear Opmode")
public class Into_The_Deep_Starterbot_Encoder_SWLimits extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //       private DcMotor motorLeft; //left drive motor
//       private DcMotor motorRight; //right drive motor
    private DcMotor motorArm; //arm motor
//       private Servo servoClaw; //claw servo
//       private Servo servoExtend; //extension servo

    //variables for drive
    int drive = 0;//variable for drive style. 0=Split arcade, 1=Arcade, 2=Tank **ADJUST THIS NUMBER AS NEEDED!**
    double LeftPower = 0;//Do not adjust!
    double RightPower = 0;//Do not adjust!

    //variables for arm position; more negative is arm up
    int height = 0; //current arm target position
    //       int sub = -180; //arm sub position **ADJUST THIS NUMBER AS NEEDED!**
    int sub = -10; //arm sub position **ADJUST THIS NUMBER AS NEEDED!**
    int clip = -300; //arm wall clip position **ADJUST THIS NUMBER AS NEEDED!**
    int low = -585; //arm low position **ADJUST THIS NUMBER AS NEEDED!**
    int high = -1000; //arm high position **ADJUST THIS NUMBER AS NEEDED!**
    int ascend = -1420; //arm climb prep position **ADJUST THIS NUMBER AS NEEDED!**
    int state = 0; //variable for position control telemetry feedback

    //variables for extension; larger number is extended
    double extensionTarget = 1.0;//extension target position; also the position at startup
    double stow = 1.0; //extension position (fully retracted) **ADJUST THIS NUMBER AS NEEDED!**
    double reachF = 0.0; //extension position (fully extended) **ADJUST THIS NUMBER AS NEEDED!**
    double reachM = 0.4; //extension position (mostly extended) **ADJUST THIS NUMBER AS NEEDED!**
    double reachH = 0.6; //extension position (half extended) **ADJUST THIS NUMBER AS NEEDED!**
    double pickup = 0.9; //extension position (nearby floor pickup) **ADJUST THIS NUMBER AS NEEDED!**
    double wall = 0.63; //extension position (wall pickup) **ADJUST THIS NUMBER AS NEEDED!**
    double manualExtend = 0.0015;//this is the manual extension factor bigger moves faster**ADJUST THIS NUMBER AS NEEDED!**

    //variables for claw; larger number is open
    double open = 0.6; //claw position (fully open) **ADJUST THIS NUMBER AS NEEDED!**
    double close = 0.8; //claw position (fully closed) **ADJUST THIS NUMBER AS NEEDED!**

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
//        motorLeft = hardwareMap.dcMotor.get("Left");
//        motorRight = hardwareMap.dcMotor.get("Right");
        motorArm = hardwareMap.dcMotor.get("arm");
//        servoClaw = hardwareMap.servo.get("Claw");
//        servoExtend = hardwareMap.servo.get("Extend");

        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resets the arm counts to zero

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
//        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD); //if the robot is driving backwards, change this to "REVERSE"
//        motorRight.setDirection(DcMotorSimple.Direction.REVERSE); //if the robot is driving backwards, change this to "FORWARD"
        motorArm.setDirection(DcMotorSimple.Direction.REVERSE); //if the arm goes backwards, change this to "REVERSE"

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            motorArm.setTargetPosition(height); //sets the target position of the arm
            if (height > motorArm.getCurrentPosition()) {//Arm is going down
                motorArm.setPower(.15); //sets the arm power **ADJUST THIS NUMBER AS NEEDED!**
            } else if ((height < motorArm.getCurrentPosition()) && (height < -850)) {
                motorArm.setPower(.25); //sets the arm power **ADJUST THIS NUMBER AS NEEDED!**
            } else {//arm is going up (front side)
                motorArm.setPower(0.25); //sets the arm power **ADJUST THIS NUMBER AS NEEDED!**
            }
            motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //sets the arm to brake mode to prevent sagging while raised
            motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION); //sets the arm to run using the built in encoder

            //Drive Control
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            //split arcade drive
            if (drive == 0) {//drive on the left stick and rotate on the right
                LeftPower = (y + rx) / denominator;
                RightPower = (y - rx) / denominator;
            }

            //arcade drive
            else if (drive == 1) {//all driving on the left stick
                LeftPower = (y + x) / denominator;
                RightPower = (y - x) / denominator;
            }

            //tank drive
            else if (drive == 2) {//left stick = left drive and right stick = right drive
                LeftPower = gamepad1.left_stick_y;
                RightPower = gamepad1.right_stick_y;
            }

//            motorLeft.setPower(LeftPower);
//            motorRight.setPower(RightPower);

            //Lift/Extension Control
//            servoExtend.setPosition(extensionTarget);//sets the extension servo to the target position
            if (gamepad1.right_trigger > 0) {//manual extension in
                if (extensionTarget < 1) {
                    extensionTarget = extensionTarget + manualExtend;
                } else if (extensionTarget > 1) {
                    extensionTarget = 1;
                }
            } else if (gamepad1.right_bumper) {//manual extension out
                if (extensionTarget > 0) {
                    extensionTarget = extensionTarget - manualExtend;
                } else if (extensionTarget < 0) {
                    extensionTarget = 0;
                }
            } else if (gamepad1.left_bumper) {//wall game piece position
                height = clip;
                extensionTarget = stow;
                state = 4;
            } else if (gamepad1.dpad_down) {//floor pickup
                height = 0;
                extensionTarget = pickup;
                state = 0;
            } else if (gamepad1.dpad_left) {//sub pickup **Press and Hold**
                height = sub;
                if (motorArm.getCurrentPosition() < (sub + 25)) {//verifies the arm is in the correct area before extending
                    extensionTarget = reachM;
                } else {//keeps the extension in until the arm is in the correct area
                    extensionTarget = stow;
                }
                state = 1;
            } else if (gamepad1.dpad_right) {//lower basket **Press and Hold**
                height = low;
                if (motorArm.getCurrentPosition() < (low + 25)) {//verifies the arm is in the correct area before extending
                    extensionTarget = reachM;
                } else {//keeps the extension in until the arm is in the correct area
                    extensionTarget = stow;
                }
                state = 2;
            } else if (gamepad1.dpad_up) {//upper basket **Press and Hold**
                height = high;
                if (motorArm.getCurrentPosition() < (high + 900)) {//verifies the arm is in the correct area before extending
                    extensionTarget = reachF;
                } else {//keeps the extension in until the arm is in the correct area
                    extensionTarget = stow;
                }
                state = 3;
            } else if (((gamepad1.a) && (height < 0))) {//manual arm down
                height = height + 1;
            } else if (((gamepad1.y) && (height > -1550))) {//manual arm up
                height = height - 1;
            } else if (gamepad1.back) {//ascend position
//                servoClaw.setPosition(open);
                height = ascend;
                extensionTarget = stow;

                state = 5;
            }

            //Claw Control
            if (motorArm.getCurrentPosition() < -1075) {//auto opens the claw when the arm is facing backwards
//                servoClaw.setPosition(open);
            } else if (gamepad1.left_trigger > 0) {//claw open on button
//                servoClaw.setPosition(open);
            } else {//claw defaults closed
//                servoClaw.setPosition(close);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            //telemetry data for drive type
            if (drive == 0) {
                telemetry.addData("Style", "SplitArcade");
            }
            if (drive == 1) {
                telemetry.addData("Style", "Arcade");
            }
            if (drive == 2) {
                telemetry.addData("Style", "Tank");
            }

            //telemetry data for claw position diagnostics
//            if ((servoClaw.getPosition()) == open) {
//                telemetry.addData("Claw", "Open");
//            } else {
//                telemetry.addData("Claw", "Closed");
//            }

            //telemetry data for arm position diagnostics
            if (state == 0) {
                telemetry.addData("Position", "Floor");
            }
            if (state == 1) {
                telemetry.addData("Position", "Sub");
            }
            if (state == 2) {
                telemetry.addData("Position", "Lower");
            }
            if (state == 3) {
                telemetry.addData("Position", "Upper");
            }
            if (state == 4) {
                telemetry.addData("Position", "Wall");
            }
            if (state == 5) {
                telemetry.addData("Position", "Ascend");
            }
            telemetry.addData("Arm Current", motorArm.getCurrentPosition());
            telemetry.addData("Arm Target", height);
            telemetry.addData("Extension Target", extensionTarget);
            telemetry.update();
        }
    }
}
