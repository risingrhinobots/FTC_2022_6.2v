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

import androidx.renderscript.ScriptIntrinsicBLAS;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop_Mecanum_Test2020", group="Test2020")
//@Disabled
public class Teleop_Mecanum_Test2020 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private DcMotor CarouselDrive = null;
    private DcMotor SidePower = null;
    private Servo ClawMotor = null;




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FrontLeftDrive  = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeftDrive = hardwareMap.get(DcMotor.class,"BackLeft");
        BackRightDrive = hardwareMap.get(DcMotor.class,"BackRight");
        CarouselDrive = hardwareMap.get(DcMotor.class, "CarouselDrive");
        SidePower = hardwareMap.get(DcMotor.class, "SidePower");
        ClawMotor = hardwareMap.get(Servo.class,"ClawMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BackLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);
        CarouselDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        SidePower.setDirection(DcMotorSimple.Direction.REVERSE);
        ClawMotor.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double FrontLeftPower;
            double FrontRightPower;
            double BackLeftPower;
            double BackRightPower;
            double sidePower;
            double carouselPower;
            double clawPower;

            double drive = 0.9*(-gamepad1.left_stick_y);
            double turn  =  0.7 * (gamepad1.right_stick_x);
            double strafe = 0.9*(gamepad1.left_stick_x);


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.




            FrontLeftPower    = Range.clip(-drive - turn - strafe, -1, 1) ;
            FrontRightPower   = Range.clip(-drive + turn + strafe, -1, 1) ;
            BackLeftPower    = Range.clip(-drive - turn + strafe, -1, 1) ;
            BackRightPower   = Range.clip(-drive + turn - strafe, -1, 1) ;
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            FrontLeftDrive.setPower(FrontLeftPower);
            FrontRightDrive.setPower(FrontRightPower);
            BackLeftDrive.setPower(BackLeftPower);
            BackRightDrive.setPower(BackRightPower);

            if(gamepad1.right_bumper){
                CarouselDrive.setPower(-0.2);
            }
            if(gamepad1.left_bumper){
                CarouselDrive.setPower(0.2);
            }
            if(gamepad1.y){
                CarouselDrive.setPower(0.0);
            }
            if(gamepad1.a){
                SidePower.setPower(-0.9);
            }
            if(gamepad1.b){
                SidePower.setPower(0.6);
            }
            if(gamepad1.dpad_down){
                SidePower.setPower(0.0);
            }
            //creates a thread class that starts the motor and starts the timer at the same time
            class shootAndStart implements Runnable{
                public void run(){
                    ElapsedTime motorRunTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
                    FrontLeftDrive.setPower(0.6);
                    motorRunTime.startTime();
                    if(motorRunTime.time() == 120){
                        FrontLeftDrive.setPower(0.0);
                        motorRunTime.reset();
                    }
                }
            }

            //running the thread
            Thread thread1 = new Thread(new shootAndStart());
            thread1.start();

            if(gamepad2.dpad_up){
                ClawMotor.setPosition(1.0);
            }
            if(gamepad2.dpad_left) {
                ClawMotor.setPosition(0.5);
            }
            if(gamepad2.dpad_down){
                ClawMotor.setPosition(0.0);
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Frontleft (%.2f), Frontright (%.2f), Backleft (%.2f), Backright (%.2f)", FrontLeftPower, FrontRightPower, BackLeftPower,BackRightPower);
            telemetry.update();
        }
    }
}

