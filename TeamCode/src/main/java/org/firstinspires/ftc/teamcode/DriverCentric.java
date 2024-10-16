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

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "DriverCentric", group = "Linear OpMode")
//@Disabled
@Config
public class DriverCentric extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;

    private DcMotor slideRight = null;
    private DcMotor slideLeft = null;

    //variables for the claw
    public static double clawOpen = .4;
    public static double clawClosed = .9;
    public static double armUp = .4;
    public static double armDown = .9;
    boolean isClawOpen = true;
    boolean isArmUp = false;
    boolean isAlreadyPressingX = false; //the state of gamepad1.x the previous frame
    boolean isAlreadyPressingY = false;

    //Variables for the slider
    public static int slideHeight = 0;
    boolean isAlreadyPressingUp = false; //the state of gamepad1.dpad_up the previous frame
    boolean isAlreadyPressingDown = false; //the state of gamepad1.dpad_down the previous frame
    //    static int[] slideHeights = new int[]{0,0,0,0,0};

    //drive direction variables & map orientation variables
    double drive;
    double strafe;
    double turn;
    double squircleMapX;
    double squircleMapY;
    double robotAngleOfMovement;
    double joystickAngle;
    double robotAngle;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        backRightMotor = hardwareMap.get(DcMotor.class, "br");

        slideRight = hardwareMap.get(DcMotor.class, "sr");
        slideLeft = hardwareMap.get(DcMotor.class, "sl");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        slideLeft.setDirection(DcMotor.Direction.FORWARD);
        slideRight.setDirection(DcMotor.Direction.REVERSE);
//
//        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (driver presses START)

        //Creating the Claw in the code
//        Servo clawServo;
////      Setting a definition for claw servo from the map
//        clawServo = hardwareMap.get(Servo.class, "claw");
//        //Sets the claw position to be either open or closed on initialization
//        //True=closed, false equals open
//        clawServo.setPosition(isClawOpen ? clawClosed : clawOpen);
//        //Creating the Claw in the code
//        Servo armServo;
////      Setting a definition for claw servo from the map
//        armServo = hardwareMap.get(Servo.class, "arm");
//        //Sets the claw position to be either open or closed on initialization
//        //True=closed, false equals open
//        clawServo.setPosition(isArmUp ? armUp : armUp);

        waitForStart();
        runtime.reset();


//          Runs until we tell it to stop
        while (opModeIsActive()) {

//            Open and close claw
//            if (!isAlreadyPressingX && gamepad2.x) {
//                isClawOpen = !isClawOpen;
//                clawServo.setPosition(isClawOpen ? clawClosed : clawOpen);
//            }
//            isAlreadyPressingX = gamepad2.x;
//
//            if (!isAlreadyPressingY && gamepad2.y) {
//                isArmUp = !isArmUp;
//                clawServo.setPosition(isArmUp ? armUp : armDown);
//            }
//            isAlreadyPressingY = gamepad2.y;

            //Slider code
//            if(!isAlreadyPressingUp && gamepad2.dpad_up){
//                if(slideHeight == 4){
//                  slideHeight=0;
//                } else {
//                    slideHeight++;
//                }
//                slideRight.setTargetPosition(slideHeights[slideHeight]);
//                slideLeft.setTargetPosition(slideHeights[slideHeight]);
//            }
//
//            if(!isAlreadyPressingUp && gamepad2.dpad_down){
//                if(slideHeight == 0){
//                    slideHeight=4;
//                } else {
//                    slideHeight--;
//                }
//                slideRight.setTargetPosition(slideHeights[slideHeight]);
//                slideLeft.setTargetPosition(slideHeights[slideHeight]);
//            }
//
//
//            isAlreadyPressingDown = gamepad2.dpad_up;
//            isAlreadyPressingUp = gamepad2.dpad_down;
            if (gamepad2.dpad_up) {
                slideRight.setPower(.5);
                slideLeft.setPower(.5);
            } else if (gamepad2.dpad_down) {
                slideRight.setPower(-.3);
                slideLeft.setPower(-.3);
            } else {
                slideRight.setPower(0.2);
                slideLeft.setPower(0.2);
            }
            // Setup a variable for each drive wheel to save power level for telemetry
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.


            //sets drive to the square to circle map variables
            squircleMapX = gamepad1.left_stick_x*Math.sqrt(1-((gamepad1.left_stick_y*gamepad1.left_stick_y)/2));
            squircleMapY = gamepad1.left_stick_y*Math.sqrt(1-((gamepad1.left_stick_x*gamepad1.left_stick_x)/2));

            //the angle of the joystick based on x and y
            if(squircleMapX==0){
                joystickAngle=squircleMapY;
            }else if(squircleMapY>=0){
                joystickAngle=Math.atan(squircleMapY/squircleMapX);
            }else {
                joystickAngle=Math.atan(squircleMapY/squircleMapX)+180;
            }

            //the angle the robot needs to go based on the joystick angle and in reference to the map
            robotAngleOfMovement=joystickAngle-robotAngle;

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            drive = Math.sin(robotAngleOfMovement);
            strafe = Math.cos(robotAngleOfMovement);
            turn = gamepad1.right_stick_x;


            backLeftPower = drive + turn - strafe;
            backRightPower = drive - turn + strafe;
            frontLeftPower = drive + turn + strafe;
            frontRightPower = drive - turn - strafe;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);
//            if (gamepad1.x){
//                frontLeftMotor.setPower(1);
//            }
//            if (gamepad1.y){
//                frontRightMotor.setPower(1);
//            }
//            if (gamepad1.b){
//                backRightMotor.setPower(1);
//            }
//            if (gamepad1.a){
//                backLeftMotor.setPower(1);
//            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "fl (%.2f), fr (%.2f),bl (%.2f), br (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}
