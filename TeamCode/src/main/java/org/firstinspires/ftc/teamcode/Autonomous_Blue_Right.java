package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;



/* move 9 inches forward
* turn counterclockwise 90 degrees
* go in reverse at slow speed for 40 inches
* turn on carousel motor at a speed of 0.2
* */
@Autonomous(name = "Pushbot: AutoTest1",group ="FreightFrenzy")
public class Autonomous_Blue_Right extends LinearOpMode{
    HardwarePushbot_TC robot = new HardwarePushbot_TC();
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 0.78;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.3;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/ (WHEEL_DIAMETER_INCHES * 3.1415);
    @Override
    public void runOpMode() {

        double clawPosition = 0.1;
        robot.init(hardwareMap);
        telemetry.addData("Status","Resetting Encoders");
        telemetry.update();
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0","Starting at %7d :%7d",
                robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition());

        telemetry.update();

        waitForStart();

    }



}
