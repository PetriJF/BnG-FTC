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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.util.Range.*;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class RobotBNG {
    /* Public OpMode members. */
    public DcMotor motorLeft1 = null;
    public DcMotor motorLeft2 = null;
    public DcMotor motorRight1 = null;
    public DcMotor motorRight2 = null;

    public DcMotor sliderSucker = null;
    public DcMotor suckerFlip = null;
    public DcMotor mineralRiser = null;
    public DcMotor carlig = null;

    public Servo MineralFlip = null;
    public Servo Sucker = null;
    public Servo marker = null;


    public int extendMotorCountsPerRev60 = 1680;
    public int mmPerRotationMineralRiser = 216, mmPerSuckerExtention = 518;
    public int extendMotorCountsPerRev40 = 1120;
    public int extendMotorCountsPerRev20 = 537;

    //ModernRoboticsI2cRangeSensor senzorDistanta;

    //  IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro gyro;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public RobotBNG() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorLeft1 = hwMap.get(DcMotor.class, "motorLeft1");
        motorLeft2 = hwMap.get(DcMotor.class, "motorLeft2");
        motorRight1 = hwMap.get(DcMotor.class, "motorRight1");
        motorRight2 = hwMap.get(DcMotor.class, "motorRight2");
        suckerFlip = hwMap.get(DcMotor.class, "suckerFlip");
        sliderSucker = hwMap.get(DcMotor.class, "sliderSucker");
        mineralRiser = hwMap.get(DcMotor.class, "mineralRiser");
        carlig = hwMap.get(DcMotor.class, "carlig");


        MineralFlip = hwMap.get(Servo.class, "MineralFlip");
        Sucker = hwMap.get(Servo.class, "Sucker");

       // senzorDistanta = hwMap.get(ModernRoboticsI2cRangeSensor.class, "senzorDistanta");
        gyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
//        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;

        //marker = hwMap.get(Servo.class, "marker");


        motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        carlig.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER );


        carlig.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);

        sliderSucker.setDirection(DcMotor.Direction.REVERSE);
        mineralRiser.setDirection(DcMotor.Direction.REVERSE);
        carlig.setDirection(DcMotor.Direction.FORWARD);
        suckerFlip.setDirection(DcMotor.Direction.FORWARD);

        motorLeft1.setPower(0f);
        motorLeft2.setPower(0f);
        motorRight1.setPower(0f);
        motorRight2.setPower(0f);

        sliderSucker.setPower(0f);
        mineralRiser.setPower(0f);
        carlig.setPower(0f);
        suckerFlip.setPower(0f);

        MineralFlip.setPosition(0.61f);

        Sucker.setPosition(0.5f);

        //marker.setPosition(0f);


    }


    public void holonomic(double left_stick_x, double right_stick_x, double left_stick_y) {

//
        final double x = Math.pow(left_stick_x, 3.0);
        final double y =  Math.pow(left_stick_y, 3.0);

        final double rotation = Math.pow(right_stick_x, 3.0);

        final double direction = Math.atan2(x, y) ;
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final double lf = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double rf = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double lr = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double rr = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

        motorLeft1.setPower(lf);
        motorRight1.setPower(rf);
        motorLeft2.setPower(lr);
        motorRight2.setPower(rr);
    }


    public void setMotorSpeed(float speed) {
        motorLeft2.setPower(speed);
        motorRight2.setPower(speed);
        motorLeft1.setPower(speed);
        motorRight1.setPower(speed);
    }


    public void landing() {
        int newTargetPos = -32650;
        carlig.setPower(-1f);
        plasareMinerale(30);

        while (carlig.getCurrentPosition() > newTargetPos) {

        }

        carlig.setPower(0f);
    }

    public void plasareMinerale(int mmToExtend) {
        int newTargetPos = mineralRiser.getCurrentPosition() + (mmToExtend * extendMotorCountsPerRev60 / mmPerRotationMineralRiser);// 216 mm o rotatie completa
        mineralRiser.setTargetPosition(newTargetPos);
        mineralRiser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mineralRiser.setPower(1f * Math.signum(mmToExtend));
        while (mineralRiser.isBusy()) {

        }

        mineralRiser.setPower(0f);
        mineralRiser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void ExtendSucker(int mmToExtend) {
        int newTargetPos = sliderSucker.getCurrentPosition() + (mmToExtend * extendMotorCountsPerRev60 / mmPerSuckerExtention);
        sliderSucker.setTargetPosition(newTargetPos);
        sliderSucker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(mmToExtend > 0)
            sliderSucker.setPower(1f * Math.signum(mmToExtend));
        else sliderSucker.setPower(-0.5f);

        while (sliderSucker.isBusy()) {

        }

        sliderSucker.setPower(0f);
        sliderSucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void latchingTeleOp(float sign) {
        carlig.setPower(1f * sign);
    }


    public void plasareMineraleTeleOp(int sign) {
        if (sign > 0)
            mineralRiser.setPower(1f);
        else
            mineralRiser.setPower(-0.5f);

    }

    public void plasareMarker() {

    }
}