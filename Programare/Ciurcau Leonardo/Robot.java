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

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.util.Range.*;

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
public class Robot {
    /* Public OpMode members. */
    public DcMotor motorLeft1 = null;
    public DcMotor motorLeft2 = null;
    public DcMotor motorRight1 = null;
    public DcMotor motorRight2 = null;
    public DcMotor sliderSucker = null;
    public Servo flip1 = null;
    public Servo flip2 = null;
    public Servo MineralFlip = null;
    public DcMotor motorSugere = null;
    public DcMotor mineralRiser = null;
    public DcMotor carlig = null;
    public Servo marker = null;

    public int extendMotorCountsPerRev = 1680;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Robot() {

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
        flip1 = hwMap.get(Servo.class, "flip1");
        flip2 = hwMap.get(Servo.class, "flip2");
        MineralFlip = hwMap.get(Servo.class, "MineralFlip");

        sliderSucker = hwMap.get(DcMotor.class, "sliderSucker");
        motorSugere = hwMap.get(DcMotor.class, "motorSugere");
        mineralRiser = hwMap.get(DcMotor.class, "mineralRiser");
        carlig = hwMap.get(DcMotor.class, "carlig");
        marker = hwMap.get(Servo.class, "marker");


        motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderSucker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralRiser.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carlig.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderSucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mineralRiser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carlig.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeft1.setDirection(DcMotor.Direction.FORWARD);
        motorLeft2.setDirection(DcMotor.Direction.FORWARD);
        motorRight1.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);

        sliderSucker.setDirection(DcMotor.Direction.FORWARD);
        mineralRiser.setDirection(DcMotor.Direction.REVERSE);
        carlig.setDirection(DcMotor.Direction.FORWARD);

        motorLeft1.setPower(0f);
        motorLeft2.setPower(0f);
        motorRight1.setPower(0f);
        motorRight2.setPower(0f);

        flip1.setPosition(0.17f);
        flip2.setPosition(0.84f);
        MineralFlip.setPosition(0.88f);
        sliderSucker.setPower(0f);
        motorSugere.setPower(0f);
        mineralRiser.setPower(0f);
        carlig.setPower(0f);
        marker.setPosition(0f);


    }


    public void holonomic(double left_stick_x, double right_stick_x, double left_stick_y) {

//
        /*double r = Math.hypot(left_stick_x, left_stick_y);
        double robotAngle = Math.atan2(left_stick_y, left_stick_x) - Math.PI / 4;
        double rightX = right_stick_x;
        final double v1 = (r * Math.cos(robotAngle) + rightX) / 2f;
        final double v2 = (r * Math.sin(robotAngle) - rightX) / 2f;
        final double v3 = (r * Math.sin(robotAngle) + rightX) / 2f;
        final double v4 = (r * Math.cos(robotAngle) - rightX) / 2f;*/

        final double v1 = left_stick_y + right_stick_x + left_stick_x;
        final double v2 = left_stick_y - right_stick_x - left_stick_x;
        final double v3 = left_stick_y + right_stick_x - left_stick_x;
        final double v4 = left_stick_y - right_stick_x + left_stick_x;

        motorLeft1.setPower(v1);
        motorRight1.setPower(v2);
        motorLeft2.setPower(v3);
        motorRight2.setPower(v4);
    }


    public void setMotorSpeed(float speed) {
        motorLeft2.setPower(speed);
        motorRight2.setPower(speed);
        motorLeft1.setPower(speed);
        motorRight1.setPower(speed);
    }

    /*    public void flipDown() throws InterruptedException {
            for (double i = flip1.getPosition(), j = flip2.getPosition(); i <= 0.98 && j >= 0.02; i += 0.01, j -= 0.01)
            {
                flip1.setPosition(i);
                flip2.setPosition(j);
                sleep(200);
            }
        }
    */
    public void flipPosition() {
        flip1.setPosition(0.25f);
        flip2.setPosition(0.75f);
    }

    public void flipUp() {
        flip1.setPosition(0.25f);
        flip2.setPosition(0.75f);
    }

    public void sugere() {
        motorSugere.setPower(0.7f);
    }

    public void scuipare() {
        motorSugere.setPower(-0.7f);
    }

    public void stopSugere() {
        motorSugere.setPower(0f);
    }


    public void latching(int mmToExtend) {
        int newTargetPos = carlig.getCurrentPosition() + (int) (mmToExtend * 5.88);
        carlig.setTargetPosition(newTargetPos);
        carlig.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carlig.setPower(1f);
        while (carlig.isBusy()) {

        }

        carlig.setPower(0f);
        carlig.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void plasareMinerale(int mmToExtend) {
        int newTargetPos = mineralRiser.getCurrentPosition() + (mmToExtend * extendMotorCountsPerRev);// / ? (172 * 3);
        mineralRiser.setTargetPosition(newTargetPos);
        mineralRiser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mineralRiser.setPower(1f * Math.signum(mmToExtend));
        while (mineralRiser.isBusy()) {

        }

        mineralRiser.setPower(0f);
        mineralRiser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void ExtendSucker(int mmToExtend) {
        int newTargetPos = sliderSucker.getCurrentPosition() + (mmToExtend * extendMotorCountsPerRev) / (172 * 3);
        sliderSucker.setTargetPosition(newTargetPos);
        sliderSucker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderSucker.setPower(1f * Math.signum(mmToExtend));
        while (sliderSucker.isBusy()) {

        }

        sliderSucker.setPower(0f);
        sliderSucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void latchingTeleOp(float sign) {
        carlig.setPower(1f * sign);
    }


    public void plasareMineraleTeleOp(int sign) {
        mineralRiser.setPower(1f * sign);
    }

    public void plasareMarker()
    {
        marker.setPosition(1f);
    }

    public void inchidereMarker()
    {
        marker.setPosition(0f);
    }
 }

