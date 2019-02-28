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
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="Basic: Linear OpMode Training", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear_Training extends LinearOpMode {

    RobotBNG robot = new RobotBNG();
    private ElapsedTime runtime = new ElapsedTime();
    float velocityFwdBwd = 0, velocityLeftRight = 0, angleStick = 0;
    float minimumVelocity = 0.4f, maxSpeed = 0.15f;
    int gear = 1;
    boolean mineralRiserDown = false;
    int CounterLimit = 0;

    public void runOpMode() throws RuntimeException{

        telemetry.update();
        robot.init(hardwareMap);
        robot.sliderSucker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mineralRiser.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.suckerFlip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.sliderSucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.mineralRiser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.suckerFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData(">", "Robot Ready.");    //
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            Move();
            SliderSucker();
            FlipToLander();
            Suck();
            FlipToGIGI();
            PlasareMinerale();
            Ridicare();
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Gear", gear);

            telemetry.addData("motor extend plasare ", robot.mmPerRotationMineralRiser * robot.mineralRiser.getCurrentPosition() / robot.extendMotorCountsPerRev40);
            telemetry.addData("motor extend sucker ", robot.mmPerSuckerExtention * robot.sliderSucker.getCurrentPosition() / robot.extendMotorCountsPerRev60);
            telemetry.addData("latching ", robot.carlig.getCurrentPosition());
            telemetry.addData(" sucker flip ", robot.suckerFlip.getCurrentPosition());
            telemetry.addData("gyro heading  ", robot.gyro.getHeading());
            telemetry.update();
        }
    }


    private void Move() {


        double Speed = -gamepad1.left_stick_y;
        if (Speed < 0)
            Speed = Math.max(-0.9, Speed);
        else Speed = Math.min(0.9, Speed);

        double Strafe = gamepad1.left_stick_x;
        if (Strafe < 0)
            Strafe = Math.max(-0.9, Strafe);
        else Strafe = Math.min(0.9, Strafe);


        double Turn = gamepad1.right_stick_x;

        if (Turn < 0)
            Turn = Math.max(-0.9, Turn);
        else Turn = Math.min(0.9, Turn);

        robot.holonomic(Strafe, Turn , Speed );
    }


    private void Suck()
    {
        if (gamepad2.b)
            robot.Sucker.setPosition(0f);
        else if (gamepad2.a)
            robot.Sucker.setPosition(1f);
        else robot.Sucker.setPosition(0.51);
    }
    private void FlipToLander()
    {
        if (gamepad2.right_bumper) {
            robot.MineralFlip.setPosition(0f);
            sleep(200);
            ++CounterLimit;
        }
        else if (gamepad2.left_bumper) {
            robot.MineralFlip.setPosition(0.52f);
            mineralRiserDown = true;
            sleep(200);
        }

        if (gamepad2.dpad_left)
            mineralRiserDown = false;
    }

    private void FlipToGIGI(){
        if (gamepad2.x  &&  robot.suckerFlip.getCurrentPosition() < 350) {
            double time = runtime.milliseconds();
            robot.MineralFlip.setPosition(0.52f);

            robot.suckerFlip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.suckerFlip.setTargetPosition(660);
            robot.suckerFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if((robot.mmPerRotationMineralRiser * robot.mineralRiser.getCurrentPosition() / robot.extendMotorCountsPerRev60) <= 50)
                robot.suckerFlip.setPower(0.2f);
            while (robot.suckerFlip.isBusy() && runtime.milliseconds() - time < 1200.0) {
                robot.suckerFlip.setPower(0.2f);
                Move();
                Ridicare();
                SliderSucker();

            }
            robot.suckerFlip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            robot.suckerFlip.setPower(0f);

        }
        else if (gamepad2.y && (robot.mmPerSuckerExtention * robot.sliderSucker.getCurrentPosition() / robot.extendMotorCountsPerRev60) <= 100
                &&  robot.suckerFlip.getCurrentPosition() > 145) {
            if((robot.mmPerRotationMineralRiser * robot.mineralRiser.getCurrentPosition() / robot.extendMotorCountsPerRev60) <= 50)
                robot.MineralFlip.setPosition(0.61f);


            robot.MineralFlip.setPosition(0.61f);
            robot.suckerFlip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(robot.suckerFlip.getCurrentPosition() > 370) {
                robot.suckerFlip.setTargetPosition(350);
                robot.suckerFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.suckerFlip.setPower(-0.7);
                while (robot.suckerFlip.isBusy()) {

                    Move();
                    Ridicare();
                    SliderSucker();
                }
            }

            robot.suckerFlip.setTargetPosition(130);
            robot.suckerFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.suckerFlip.setPower(-0.2f);
            while (robot.suckerFlip.isBusy()) {
                Move();
                Ridicare();
                SliderSucker();
            }


            robot.suckerFlip.setPower(0f);
            robot.suckerFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        else if (gamepad2.y && (robot.mmPerSuckerExtention * robot.sliderSucker.getCurrentPosition() / robot.extendMotorCountsPerRev60) > 100 &&   robot.suckerFlip.getCurrentPosition() > 145)
        {
            robot.suckerFlip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.suckerFlip.setTargetPosition(300);
            robot.suckerFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.suckerFlip.setPower(-0.2f);
            while (robot.suckerFlip.isBusy()) {
                Move();
                Ridicare();
                SliderSucker();
            }

            robot.suckerFlip.setPower(0f);
            robot.suckerFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        }

    }

    private void SliderSucker()
    {
        if ((gamepad1.dpad_left ^ gamepad2.dpad_up) && (robot.mmPerSuckerExtention * robot.sliderSucker.getCurrentPosition() / robot.extendMotorCountsPerRev60) < 810)
            robot.sliderSucker.setPower(1f);
        else if (gamepad2.dpad_down && (robot.mmPerSuckerExtention * robot.sliderSucker.getCurrentPosition() / robot.extendMotorCountsPerRev60) > 3)
            robot.sliderSucker.setPower(-0.4f);
        else
            robot.sliderSucker.setPower(0f);

    }


    private void PlasareMinerale()
    {
        if (gamepad2.dpad_right
                && (robot.mmPerRotationMineralRiser * robot.mineralRiser.getCurrentPosition() / robot.extendMotorCountsPerRev60) < 581)
            robot.plasareMineraleTeleOp(1);
        else if((gamepad2.dpad_left || mineralRiserDown)
                && (robot.mmPerRotationMineralRiser * robot.mineralRiser.getCurrentPosition() / robot.extendMotorCountsPerRev60) > 30 + CounterLimit) {
            robot.plasareMineraleTeleOp(-1);
            if ((robot.mmPerRotationMineralRiser * robot.mineralRiser.getCurrentPosition() / robot.extendMotorCountsPerRev60) < 400)
                robot.MineralFlip.setPosition(0.52f);
        }
        else
        {
            robot.mineralRiser.setPower(0f);
            mineralRiserDown = false;
        }

    }


    private void Ridicare()
    {
        if (gamepad1.dpad_up)
            robot.latchingTeleOp(1);
        else if(gamepad1.dpad_down)
            robot.latchingTeleOp( -1);
        else robot.carlig.setPower(0f);
    }
}