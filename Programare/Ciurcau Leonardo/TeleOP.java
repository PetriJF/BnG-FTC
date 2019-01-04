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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class TeleOP extends LinearOpMode {

    // Declare OpMode members.
    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    float velocityFwdBwd = 0, velocityLeftRight = 0, angleStick = 0;
    float minimumVelocity = 0.2f, maxSpeed = 0.15f;
    int gear = 1;
    int Toggle_Remorca = 0, Toggle_Plasare = 0;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();
        robot.flipPosition();

        while (opModeIsActive()) {

            Move();
            ChangeSpeed();
            SliderSucker();
            Flip();
            Suge();
            PlasareMinerale();
            TurnareMinerale();
            Ridicare();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Gear", gear);
            telemetry.addData("Left1 ", robot.motorLeft1.getCurrentPosition());
            telemetry.addData("Left2 ", robot.motorLeft2.getCurrentPosition());

            telemetry.addData("Right1 ", robot.motorRight1.getCurrentPosition());
            telemetry.addData("Right2 ", robot.motorRight2.getCurrentPosition());
            telemetry.addData("Motor extend ", (float)Math.abs(robot.sliderSucker.getCurrentPosition()) / (float)robot.extendMotorCountsPerRev * 172);


            telemetry.update();
        }
    }


    private void Move() {

        double Speed = gamepad1.left_stick_y;
        double Turn = gamepad1.left_stick_x;
        double Strafe = gamepad1.right_stick_x;
        if (Math.abs(Speed) <= minimumVelocity)
            Speed = 0f;
        if (Math.abs(Turn) <= minimumVelocity)
            Turn = 0f;
        if (Math.abs(Strafe) <= minimumVelocity)
            Strafe = 0f;

         Turn = Range.scale(Math.abs(Turn), 0, 1,0, maxSpeed) * Math.signum(Turn);
         Strafe = Range.scale(Math.abs(Strafe), 0, 1, 0, maxSpeed) * Math.signum(-Strafe);
         Speed = Range.scale(Math.abs(Speed) * maxSpeed, 0, maxSpeed, 0, maxSpeed) * Math.signum(Speed);
         robot.holonomic(Turn, Strafe, Speed);
    }
    private void setSpeed()
    {
        if(gear == 1)
            maxSpeed = 0.3f;
        else if(gear == 2)
            maxSpeed = 0.45f;
        else if(gear == 3)
            maxSpeed = 0.6f;
        else if(gear == 4)
            maxSpeed = 0.8f;
    }
    private void ChangeSpeed()
    {
        if(gamepad1.right_bumper && gear < 4 )
        {
            sleep(200);
            gear++;
            setSpeed();
        }
        else if(gamepad1.left_bumper && gear > 1)
        {
            sleep(200);
            gear--;
            setSpeed();
        }
    }

    private void Flip()
    {
        if(gamepad2.a && Toggle_Remorca == 0) {
            for (double i = robot.flip1.getPosition(), j = robot.flip2.getPosition(); i <= 0.88 && j >= 0.12; i += 0.01, j -= 0.01)
            {
                robot.flip1.setPosition(i);
                robot.flip2.setPosition(j);
                sleep(15);
                Move();
            }
            Toggle_Remorca = 1;
            sleep(200);
        }else if (gamepad2.a && Toggle_Remorca == 1) {
            for (double i = robot.flip1.getPosition(),  j = robot.flip2.getPosition(); i >= 0.25 && j <= 0.75; i -= 0.01, j += 0.01) {
                robot.flip1.setPosition(i);
                robot.flip2.setPosition(j);
                sleep(15);
                Move();
            }
            Toggle_Remorca = 0;
            sleep(200);
        }
    }

    private void TurnareMinerale(){
        if(gamepad2.y && Toggle_Plasare == 0){
            robot.MineralFlip.setPosition(0f);
            Toggle_Plasare = 1;
            sleep(200);
        }else if(gamepad2.y && Toggle_Plasare == 1){
            robot.MineralFlip.setPosition(0.88f);
            Toggle_Plasare = 0;
            sleep(200);
        }
    }

    private void SliderSucker()
    {
        if (gamepad2.dpad_up && (float)Math.abs(robot.sliderSucker.getCurrentPosition()) / (float)robot.extendMotorCountsPerRev * 172 / 3 < 100.0)
            robot.sliderSucker.setPower(1f);
        else if (gamepad2.dpad_down && (float)Math.abs(robot.sliderSucker.getCurrentPosition()) / (float)robot.extendMotorCountsPerRev * 172 / 3 > 0.0)
            robot.sliderSucker.setPower(-1f);
        else
            robot.sliderSucker.setPower(0f);

    }

    private void Suge()
    {
        if(gamepad2.right_trigger > 0.3)
            robot.sugere();

        else if(gamepad2.left_trigger > 0.3)
            robot.scuipare();
        else
            robot.stopSugere();
    }

    private void PlasareMinerale()
    {
        if (gamepad2.left_stick_y > 0.3)
            robot.plasareMineraleTeleOp(1);
        else if(gamepad2.left_stick_y < -0.3)
            robot.plasareMineraleTeleOp(-1);
        else robot.mineralRiser.setPower(0f);
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
