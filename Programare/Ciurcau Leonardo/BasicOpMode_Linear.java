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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
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

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorLeft1 = null;
    private DcMotor motorRight1 = null;
    private DcMotor motorLeft2 = null;
    private DcMotor motorRight2 = null;
    private DcMotor CubeRiser = null;
    private Servo mineralRelease = null;
    private Servo mineralFlip = null;
    private DcMotor motorExtendSucker = null;
    private DcMotor motorSucker = null;
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    float velocityFwdBwd, velocityLeftRight, maxSpeed = 0.15f;
    int gear = 1, cubeRiserPos = 0;
    final float minimumVelocity = 0.01f;
    final int CubeRiserMax = 6000;
    boolean mineralReleaseStatus = true, cubeRiserStatus = true;
    int suckerExtendPos = 0;
    final int suckerExtendMax = 5049;
    boolean mineralFlipStatus = false;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        Init();

        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {

            ChangeSpeed();
            Move();
            Box();
            Sugatoare();
            ExtendSucker();
            CubeRising();
            //Launch();


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Gear ", gear);
            telemetry.addData("left stick y     ", gamepad2.left_stick_y);
            telemetry.addData("dpad up    ", gamepad2.dpad_up);
            telemetry.addData("Heading", modernRoboticsI2cGyro.getHeading());
            telemetry.addData("Rotate ticks", motorRight1.getCurrentPosition());
            telemetry.addData("Tick-uri", CubeRiser.getCurrentPosition());
            telemetry.addData("extend sucker ticks ", motorExtendSucker.getCurrentPosition());

            telemetry.update();
        }
    }

    /*private void Launch(){
        if(gamepad1.dpad_up){
            Elevator.setDirection(DcMotor.Direction.REVERSE);
            Elevator.setPower(1f);
        }
        else if(gamepad1.dpad_down){
            Elevator.setDirection(DcMotor.Direction.FORWARD);
            Elevator.setPower(0.3f);
        }else{
            Elevator.setPower(0f);
        }
        if(gamepad1.a)
        {
            Elevator.setTargetPosition(Elevator.getCurrentPosition());
            Elevator.setPower(1f);
        }

    }*/


    private void Init()
    {
        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");
        CubeRiser = hardwareMap.dcMotor.get("CubeRiser");
        mineralRelease = hardwareMap.servo.get("mineralRelease");
        mineralFlip = hardwareMap.servo.get("mineralFlip");
        //CubeRiser.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //CubeRiser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Elevator = hardwareMap.dcMotor.get("LaunchMotor");
        motorExtendSucker = hardwareMap.dcMotor.get("motorExtendSucker");
        motorSucker = hardwareMap.dcMotor.get("motorSucker");
        motorSucker.setDirection(DcMotor.Direction.FORWARD);

        mineralRelease.setPosition(0.75f);
        mineralFlip.setPosition(0.33f);
        cubeRiserPos = CubeRiser.getCurrentPosition();
        suckerExtendPos = motorExtendSucker.getCurrentPosition();
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();
        telemetry.log().add("Gyro Calibrated!");

        //servoBox = hardwareMap.servo.get("servoBox");
       // servoBox.setPosition(0.5);

    }

    private void moveRobotForward(float speed)
    {
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);
        setMotorSpeed(speed);
    }

    private void moveRobotReverse(float speed)
    {
        motorLeft1.setDirection(DcMotor.Direction.FORWARD);
        motorLeft2.setDirection(DcMotor.Direction.FORWARD);
        motorRight1.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        setMotorSpeed(speed);
    }

    private void rotateRobotRight(float speed)
    {
        motorLeft1.setDirection(DcMotor.Direction.FORWARD);
        motorLeft2.setDirection(DcMotor.Direction.FORWARD);
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);
        setMotorSpeed(speed);
    }

    private void rotateRobotLeft(float speed)
    {
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        setMotorSpeed(speed);
    }


    private void setMotorSpeed(float speed)
    {
        motorLeft2.setPower(speed);
        motorRight2.setPower(speed);
        motorLeft1.setPower(speed);
        motorRight1.setPower(speed);
    }
    private void setSpeed()
    {
        if(gear == 1)
            maxSpeed = 0.15f;
        else if(gear == 2)
            maxSpeed = 0.30f;
        else if(gear == 3)
            maxSpeed = 0.45f;
        else if(gear == 4)
            maxSpeed = 0.6f;
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

    private void Move()
    {
        velocityFwdBwd = gamepad1.left_stick_y;
        velocityLeftRight = -gamepad1.right_stick_x;

        if (Math.abs(velocityFwdBwd) > minimumVelocity)
        {

            if (velocityFwdBwd < 0f) moveRobotForward(velocityFwdBwd * maxSpeed);
            else if (velocityFwdBwd > 0f)moveRobotReverse(-velocityFwdBwd * maxSpeed);
            else setMotorSpeed(0f);

        }
        else if (Math.abs(velocityLeftRight) > minimumVelocity)
        {

            if (velocityLeftRight < 0f) rotateRobotRight(-velocityLeftRight * maxSpeed);
            else if (velocityLeftRight > 0f) rotateRobotLeft(velocityLeftRight * maxSpeed);


            else setMotorSpeed(0f);
        }
        else setMotorSpeed(0f);

    }

    private void Box()
    {
        if (gamepad2.right_bumper && mineralReleaseStatus)
        {
            mineralRelease.setPosition(0.1f);
            mineralReleaseStatus = false;
            sleep(200);
        }
        else if (gamepad2.right_bumper && !mineralReleaseStatus)
        {
            mineralReleaseStatus = true;
            mineralRelease.setPosition(0.75f);
            sleep(200);
        }
    }

    private void CubeRising()
    {
            if (gamepad2.left_bumper && cubeRiserStatus)
            {
                cubeRiserStatus = false;
                CubeRiser.setTargetPosition(CubeRiser.getCurrentPosition() + CubeRiserMax);
                CubeRiser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                CubeRiser.setPower(0.3f);
                CubeRiser.setPower(0f);
                CubeRiser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
            else if (gamepad2.left_bumper && !cubeRiserStatus)
            {
                cubeRiserStatus = false;
                CubeRiser.setTargetPosition(cubeRiserPos);
                CubeRiser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                CubeRiser.setPower(0.3f);
                CubeRiser.setPower(0f);
                CubeRiser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else CubeRiser.setPower(0f);

            if (gamepad2.a)
            {
                if (mineralFlipStatus == false)
                {
                    mineralFlip.setPosition(0.05f);
                    mineralFlipStatus = true;
                }
                else
                {
                    mineralFlip.setPosition(0.33f);
                    mineralFlipStatus = false;
                }
            }

    }

    private void Sugatoare()
    {
        if(gamepad2.y)
            motorSucker.setPower(1f);

        else motorSucker.setPower(0f);

    }

    private void ExtendSucker()
    {
        if(gamepad2.dpad_up && motorExtendSucker.getCurrentPosition() < suckerExtendMax + suckerExtendPos)
        {
            motorExtendSucker.setDirection(DcMotor.Direction.FORWARD);
            motorExtendSucker.setPower(0.5f);
        }
        else if(gamepad2.dpad_down && Math.abs(motorExtendSucker.getCurrentPosition()) > suckerExtendPos)
        {
            motorExtendSucker.setDirection(DcMotor.Direction.REVERSE);
            motorExtendSucker.setPower(0.5f);
        }

        else motorExtendSucker.setPower(0f);
    }

}
