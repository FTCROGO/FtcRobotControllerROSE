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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 */

@Autonomous(name="Robot: RedGreenAuto", group="Robot")
//@Disabled
public class RedGreenAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         mFL    = null;
    private DcMotor         mFR    = null;
    private DcMotor         mBL    = null;
    private DcMotor         mBR    = null;
    private DcMotor         mArm   = null;
    private DcMotor         mLS    = null;
    private Servo           Intake = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 112 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        mFL  = hardwareMap.get(DcMotor.class, "mFL");
        mFR  = hardwareMap.get(DcMotor.class, "mFR");
        mBL  = hardwareMap.get(DcMotor.class, "mFL");
        mBR  = hardwareMap.get(DcMotor.class, "mFR");
        mArm = hardwareMap.get(DcMotor.class, "mArm");
        mLS  = hardwareMap.get(DcMotor.class, "mLS");

        mFL.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.FORWARD);
        mBR.setDirection(DcMotor.Direction.REVERSE);
        mArm.setDirection(DcMotor.Direction.REVERSE);
        mLS.setDirection(DcMotor.Direction.REVERSE);

        mFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mLS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Starting at",  "%7d :%7d",
                          mFL.getCurrentPosition(),
                          mFR.getCurrentPosition(),
                          mBL.getCurrentPosition(),
                          mBR.getCurrentPosition(),
                          mArm.getCurrentPosition(),
                          mLS.getCurrentPosition());
        telemetry.update();

        waitForStart();

// Drive to basket
        encoderDrive(DRIVE_SPEED,  19, 19, 5.0);
        encoderDrive(TURN_SPEED,   -12, 24, 4.0);
        encoderDrive(DRIVE_SPEED, 15, 15, 4.0);

//Position arm and linear slide
//        mArm.setTargetPosition([]); //FIND 90 DEG POS
        mArm.setPower(0.6);
        sleep(100);
//        mLS.setTargetPosition([]); //FIND LS HEIGHT TOP BASKET
        mLS.setPower(0.5);
        sleep(100);
//        mArm.setTargetPosition([]); //FIND BASKET ANGLE
        mArm.setPower(0.6);
        sleep(100);

// Drop sample
        Intake.setPosition(1);
        sleep(100);

 // Lower arm and linear slide
//        mArm.setTargetPosition([]); //90 DEGREE
        mArm.setPower(0.6);
        sleep(100);
//        mLS.setTargetPosition(100);  //IS THIS CLOSE TO BASE LEVEL?
        mLS.setPower(1);
        sleep(100);

//Strafe and go to submersible
        //do we need to turn around to touch with back of robot?
        encoderDrive(TURN_SPEED,   24, -12, 4.0);
        mFL.setPower(1);
        mFR.setPower(-1);
        mBL.setPower(-1);
        mBR.setPower(1);
        sleep(1000);
        encoderDrive(DRIVE_SPEED, 12, 12, 4.0);





        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = mFL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = mFR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = mBL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = mBR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            mFL.setTargetPosition(newFrontLeftTarget);
            mFR.setTargetPosition(newFrontRightTarget);
            mBL.setTargetPosition(newBackLeftTarget);
            mBR.setTargetPosition(newBackRightTarget);

            mFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            mFL.setPower(Math.abs(speed));
            mFR.setPower(Math.abs(speed));
            mBL.setPower(Math.abs(speed));
            mBR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (mFL.isBusy() && mFR.isBusy() && mBL.isBusy() && mBR.isBusy())) {

                telemetry.addData("Running to",  " %7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            mFL.getCurrentPosition(), mFR.getCurrentPosition(),
                                            mBL.getCurrentPosition(), mBR.getCurrentPosition());
                telemetry.update();
            }

            mFL.setPower(0);
            mFR.setPower(0);
            mBL.setPower(0);
            mBR.setPower(0);

            mFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}