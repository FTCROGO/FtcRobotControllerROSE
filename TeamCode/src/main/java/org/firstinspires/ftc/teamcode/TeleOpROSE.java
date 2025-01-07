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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Robot: TeleOpROSE", group="Robot")
//@Disabled
public class TeleOpROSE extends LinearOpMode {

    public DcMotor  mFL  = null;
    public DcMotor  mFR  = null;
    public DcMotor  mBL  = null;
    public DcMotor  mBR  = null;
    public DcMotor  mArm = null;
    public DcMotor  mLS  = null;
    public Servo Intake  = null;

    double intakeOffset = 0;


    @Override
    public void runOpMode() {
        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;
        double drive;
        double strafe;
        double turn;
        double maxBack;
        double maxFront;
        double mArmPower;
        double mLSPower;

        mFL  = hardwareMap.get(DcMotor.class, "mFL");
        mFR  = hardwareMap.get(DcMotor.class, "mFR");
        mBL  = hardwareMap.get(DcMotor.class, "mBL");
        mBR  = hardwareMap.get(DcMotor.class, "mBR");
        mArm = hardwareMap.get(DcMotor.class, "mArm");
        mLS  = hardwareMap.get(DcMotor.class, "mLS");

        mFL.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.FORWARD);
        mBR.setDirection(DcMotor.Direction.REVERSE);
        mArm.setDirection(DcMotor.Direction.REVERSE);
        mLS.setDirection(DcMotor.Direction.REVERSE);

        mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mLS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Intake  = hardwareMap.get(Servo.class, "Intake");
        Intake.setPosition(0.5);

        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();



        waitForStart();

        while (opModeIsActive()) {

// Drivetrain -- forward/backward (y-axis joystick 1), left/right (x-axis joystick 1), turn (x-axis joystick 2)
            drive  = -gamepad1.left_stick_y;
            turn   =  gamepad1.right_stick_x;
            strafe =  gamepad1.left_stick_x;

            frontLeft  = drive + strafe + turn;
            frontRight = drive - strafe - turn;
            backLeft   = drive - strafe + turn;
            backRight  = drive + strafe - turn;

            maxBack = Math.max(Math.abs(backLeft), Math.abs(backRight));
            if (maxBack > 1.0)
            {
                backLeft   /= maxBack;
                backRight  /= maxBack;
            }

            maxFront = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
            if (maxFront > 1.0)
            {
                frontLeft  /= maxFront;
                frontRight /= maxFront;
            }

            mFL.setPower(frontLeft);
            mFR.setPower(frontRight);
            mBL.setPower(backLeft);
            mBR.setPower(backRight);


// Arm - up (Y), down (A)
            mArmPower = gamepad2.left_stick_y/1.5;

            if (mArmPower > 1.0)
                mArmPower   /= mArmPower;

            mArm.setPower(mArmPower);


    // Arm lower limit -------------------------------------------
//            if (mArm.getCurrentPosition() < 10) {
//                mArm.setTargetPosition(10);
//                mArm.setPower(0.3);
//            }
//            else if (mArm.getCurrentPosition() < 5) {
//                mArm.setPower(0);
//            }

    // Arm upper limit --------------------------------------------
//            if (mArm.getCurrentPosition() > 7130) {
//                mArm.setTargetPosition(7130);
//                mArm.setPower(-0.3);
//            }
//            else if (mArm.getCurrentPosition() > 7140) {
//                mArm.setPower(0);
//            }


// Linear Slide - up (dpad up), down (dpad down), zero position (x)
            mLSPower = -gamepad2.right_stick_y;

            if (mLSPower > 1.0)
                mLSPower   /= mLSPower;

            mLS.setPower(mLSPower);

    // Linear slide power decreases as position gets higher -------------------------
//            if (mLS.getCurrentPosition() > a && mLSPower > 0)
//                mLSPower   /= 1.5;
//            else if (mLS.getCurrentPosition() > b && mLSPower > 0)
//                mLSPower   /= 1.75;
//            else if (mLS.getCurrentPosition() > c && mLSPower > 0)
//                mLSPower   /= 2;

    // Linear slide power cut in half as gets close to lower limit ------------------
//            if (mLS.getCurrentPosition() < d && mLSPower < 0)
//                mLSPower   /= 2;

    // Linear lower limit --------------------------------------------
//            if (mLS.getCurrentPosition() < 10) {
//                mLS.setTargetPosition(10);
//                mLS.setPower(0.5);
//            }
//            else if (mLS.getCurrentPosition() < 5) {
//                mLS.setPower(0);
//            }

    // Linear upper limit ---------------------------------------------
//            if (mLS.getCurrentPosition() > 29140) {
//                mLS.setTargetPosition(29140);
//                mLS.setPower(0.5);
//            }
//            else if (mLS.getCurrentPosition() > 29150)
//                mLS.setPower(0);


//Intake - out (left bumper), in (right bumpers) --------------------------
            if (gamepad2.right_bumper)
                intakeOffset += 0.2;
            else if (gamepad2.left_bumper)
                intakeOffset -= 0.2;
            else
                intakeOffset = 0;

            intakeOffset = Range.clip(intakeOffset, -0.5, 0.5);
            Intake.setPosition(0.5 + intakeOffset);


// Send telemetry message to signify robot running;
            telemetry.addData("intake pos",  "Offset = %.2f", intakeOffset);

            telemetry.addData("front left power",  "%.2f", frontLeft);
            telemetry.addData("fl pow", "%.2f", mFL.getPower());
            telemetry.addData("front right power", "%.2f", frontRight);
            telemetry.addData("fr pow", "%.2f", mFR.getPower());
            telemetry.addData("back left power",  "%.2f", backLeft);
            telemetry.addData("bl pow", "%.2f", mBL.getPower());
            telemetry.addData("back right power", "%.2f", backRight);
            telemetry.addData("br pow", "%.2f", mBR.getPower());

            telemetry.addData("arm power",  "%.2f", mArmPower);
            telemetry.addData("linear power", "%.2f", mLSPower);
            telemetry.addData("arm pos", mArm.getCurrentPosition());
            telemetry.addData("linear pos", mLS.getCurrentPosition());
            telemetry.update();

        }
    }
}