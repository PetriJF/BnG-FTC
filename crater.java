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

import android.sax.EndElementListener;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


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

@Autonomous(name="Autonomie Crater", group="Linear Opmode")
//@Disabled
public class crater extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    RobotBNG robot = new RobotBNG();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AfOR5K3/////AAABmdlBtn+igEYQhEK8iiFVxKojIHnUdNvFj3IWgHLrGZcG17s9QYJOCPRRPf5NYeizXyxUsAmGyo8vErJwbq8xXsvs2Mpx9W8+Ndlq2oRNuGYctM4FLfjoa9hFj4YFZSOLNdtO5Rlb7/QLI8ydOjc7n2pIYPIY7hHLz+RuelzhvujdlvrvCU3XGilWESXpEymhEGfatbiRoSQAt67NTyw2XXx0hP8wt43OlAqrvIh2PIf3pInJgJS44aLtVsuZX2ErL+nuWzO9I8KTriPibgdM6PD2z6YJrW9dcRACjjhRAuqS3VY/eqEJ/BMqIVpXn8QvXd5Wr39UeATNP0an5HF76/9xx6ds0VHEXUJBWrovn7VY";
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.7;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.05;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.1;     // Larger is more responsive, but also less stable
    private VuforiaLocalizer vuforia;
    public String position = "LEFT";
    private TFObjectDetector tfod;
    boolean start = false;


    @Override
    public void runOpMode() {


        robot.init(hardwareMap);
        robot.gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && robot.gyro.isCalibrating())  {
            sleep(50);
            idle();
        }
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.addData(" sucker flip ", robot.suckerFlip.getCurrentPosition());
        telemetry.addData("gyro heading  ", robot.gyro.getHeading());
        initTF();
        ActivateTF();
        RunTF();
        telemetry.update();
        waitForStart();


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        StopTF();

        robot.landing();
        robot.Sucker.setPosition(0.5f);

        EncoderStrafe(-DRIVE_SPEED, -6, -6, 3);
        // gyroDrive(DRIVE_SPEED, 3, 0);
        //EncoderStrafe(DRIVE_SPEED, 2, 2, 3);


        if (position == "LEFT")
        {
            gyroTurn(TURN_SPEED, 45.0);
            gyroHold(TURN_SPEED, 45.0, 0.5);
            gyroDrive(DRIVE_SPEED, 28.0, 45.0);
            gyroDrive(DRIVE_SPEED, 54.0, 135.0);
            gyroHold(TURN_SPEED, 135.0, 0.5);
            // EncoderStrafe(-DRIVE_SPEED, -8, -8, 3);
            //EncoderStrafe(DRIVE_SPEED, 2, 2, 3);

        }
        else if (position == "CENTER")
        {
            gyroDrive(DRIVE_SPEED, 26.0, 5.0);
            gyroDrive(DRIVE_SPEED, -7.0, 0);
            gyroTurn(TURN_SPEED, 95.0);
            gyroHold(TURN_SPEED, 95.0, 0.5);
            gyroDrive(DRIVE_SPEED, 49.0, 95.0);
            gyroTurn(TURN_SPEED, 135.0);
            gyroHold(TURN_SPEED, 135.0, 0.5);
            //EncoderStrafe(-DRIVE_SPEED, -8, -8, 3);
            //EncoderStrafe(DRIVE_SPEED, 2, 2, 3);
            gyroDrive(DRIVE_SPEED, 4, 135.0);


        }
        else
        {
            gyroDrive(DRIVE_SPEED, 37.0, -42.0);
            gyroDrive(DRIVE_SPEED, -15.0, 0.0);
            gyroTurn(TURN_SPEED, 90.0);
            gyroHold(TURN_SPEED, 90.0, 0.5);
            gyroDrive(DRIVE_SPEED, 64, 90.0);
            gyroTurn(TURN_SPEED, 135.0);
            gyroHold(TURN_SPEED, 135.0, 0.5);
            // EncoderStrafe(-DRIVE_SPEED, -8, -8, 3);
            //EncoderStrafe(DRIVE_SPEED, 2, 2, 3);
            gyroDrive(DRIVE_SPEED, 4, 135.0);


        }
        int newTargetPos = robot.mineralRiser.getCurrentPosition() + (581 * robot.extendMotorCountsPerRev60 / robot.mmPerRotationMineralRiser);// 216 mm o rotatie completa
        robot.mineralRiser.setTargetPosition(newTargetPos);
        robot.mineralRiser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        newTargetPos = robot.sliderSucker.getCurrentPosition() + (750 * robot.extendMotorCountsPerRev60 / robot.mmPerSuckerExtention);
        robot.sliderSucker.setTargetPosition(newTargetPos);
        robot.sliderSucker.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.mineralRiser.setPower(1f * Math.signum(581));
        robot.sliderSucker.setPower(1f * Math.signum(810));

        while ((robot.mineralRiser.isBusy() || robot.sliderSucker.isBusy()) && !isStopRequested()) {
            if (!robot.mineralRiser.isBusy())
            {
                robot.mineralRiser.setPower(0f);
                robot.MineralFlip.setPosition(0f);
            }

            if (!robot.sliderSucker.isBusy())
            {
                robot.Sucker.setPosition(0f);
                sleep(500);
                robot.Sucker.setPosition(0.51f);
                robot.sliderSucker.setPower(0f);
                break;
            }
        }

        robot.MineralFlip.setPosition(0f);

        robot.mineralRiser.setPower(0f);
        robot.mineralRiser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.sliderSucker.setPower(0f);
        robot.sliderSucker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        if(position == "LEFT")
            gyroDrive(DRIVE_SPEED, -32.0, 135.0);
        else  if (position == "CENTER")
            gyroDrive(DRIVE_SPEED, -34.0, 137.0);
        else    gyroDrive(DRIVE_SPEED, -34.0, 135.0);

        robot.MineralFlip.setPosition(0f);

        telemetry.addData("mineral ", position);
        telemetry.update();
        runtime.reset();

    }
    private void initVuforia() throws RuntimeException {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }


    public void initTF() throws RuntimeException
    {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    public void ActivateTF() throws RuntimeException
    {
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void RunTF() throws RuntimeException
    {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            while(!opModeIsActive() && !isStopRequested()) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() >= 2) {
                        int goldMineralX = -1;
                        int silverMineralX = -1;
                        int goldMineralMax = -1;
                        int silverMineralMax = -1;

                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL) && recognition.getHeight() * recognition.getWidth() > goldMineralMax) {
                                goldMineralX = (int) recognition.getLeft();
                                goldMineralMax = (int) recognition.getHeight();
                            } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL) && recognition.getHeight() * recognition.getWidth() > silverMineralMax) {
                                silverMineralX = (int) recognition.getLeft();
                                silverMineralMax = (int) recognition.getHeight();
                            }
                        }

                        if (goldMineralX != -1 && silverMineralX != -1) {
                            if (goldMineralX < silverMineralX) {
                                position = "RIGHT";
                                telemetry.addData("Gold Mineral Position", "RIGHT");
                            } else {
                                position = "CENTER";
                                telemetry.addData("Gold Mineral Position", "Center");
                            }
                        }
                        else {
                            position = "LEFT";
                            telemetry.addData("Gold Mineral Position", "LEFT");

                        }
                        telemetry.update();


                    }


                }
            }
        }
    }



    public void StopTF() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive() && !isStopRequested()) {

            robot.motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            double semn = 1;
            if (distance < 0) {
                robot.motorLeft1.setDirection(DcMotor.Direction.FORWARD);
                robot.motorLeft2.setDirection(DcMotor.Direction.FORWARD);
                robot.motorRight1.setDirection(DcMotor.Direction.REVERSE);
                robot.motorRight2.setDirection(DcMotor.Direction.REVERSE);
                semn = -1;
                distance = -distance;

            }
            else {
                robot.motorLeft1.setDirection(DcMotor.Direction.REVERSE);
                robot.motorLeft2.setDirection(DcMotor.Direction.REVERSE);
                robot.motorRight1.setDirection(DcMotor.Direction.FORWARD);
                robot.motorRight2.setDirection(DcMotor.Direction.FORWARD);
            }
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.motorLeft1.getCurrentPosition() + moveCounts;
            newRightTarget = robot.motorRight1.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.motorLeft1.setTargetPosition(newLeftTarget);
            robot.motorRight1.setTargetPosition(newRightTarget);

            robot.motorLeft2.setTargetPosition( robot.motorLeft2.getCurrentPosition() + moveCounts);
            robot.motorRight2.setTargetPosition( robot.motorRight2.getCurrentPosition() + moveCounts);

            robot.motorLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.

            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.motorLeft1.setPower(speed);
            robot.motorRight1.setPower(speed);
            robot.motorLeft2.setPower(speed);
            robot.motorRight2.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (!isStopRequested() && opModeIsActive() &&
                    (robot.motorLeft1.isBusy() && robot.motorRight1.isBusy() && robot.motorLeft2.isBusy() && robot.motorRight2.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (semn < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.motorLeft1.setPower(leftSpeed);
                robot.motorRight1.setPower(rightSpeed);
                robot.motorLeft2.setPower(leftSpeed);
                robot.motorRight2.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.motorLeft1.getCurrentPosition(),
                        robot.motorRight1.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("heading ",    robot.gyro.getHeading());

                telemetry.update();

            }

            // Stop all motion;
            robot.motorRight1.setPower(0);
            robot.motorLeft1.setPower(0);

            robot.motorRight2.setPower(0);
            robot.motorLeft2.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void gyroTurn (  double speed, double angle) {

        robot.motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        robot.motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        robot.motorRight1.setDirection(DcMotor.Direction.FORWARD);
        robot.motorRight2.setDirection(DcMotor.Direction.FORWARD);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF) && !isStopRequested()) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime) && !isStopRequested()) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.motorLeft1.setPower(0);
        robot.motorRight1.setPower(0);
        robot.motorLeft2.setPower(0);
        robot.motorRight2.setPower(0);

    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.motorLeft1.setPower(leftSpeed);
        robot.motorRight1.setPower(rightSpeed);

        robot.motorLeft2.setPower(leftSpeed);
        robot.motorRight2.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public void EncoderStrafe(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        robot.motorLeft1.setDirection(DcMotor.Direction.FORWARD);
        robot.motorLeft2.setDirection(DcMotor.Direction.REVERSE);

        robot.motorRight1.setDirection(DcMotor.Direction.FORWARD);
        robot.motorRight2.setDirection(DcMotor.Direction.REVERSE);

        robot.motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorLeft1.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorRight1.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.motorLeft1.setTargetPosition(newLeftTarget);
            robot.motorRight1.setTargetPosition(newRightTarget);
            robot.motorLeft2.setTargetPosition(robot.motorLeft2.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH));
            robot.motorRight2.setTargetPosition(robot.motorRight2.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH));


            // Turn On RUN_TO_POSITION
            robot.motorLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorLeft2.setPower(speed);
            robot.motorRight1.setPower(speed);
            robot.motorLeft1.setPower(speed);
            robot.motorRight2.setPower(speed);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (!isStopRequested() && opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorLeft2.isBusy() && robot.motorRight1.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.motorLeft2.getCurrentPosition(),
                        robot.motorRight2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorLeft2.setPower(0);
            robot.motorRight2.setPower(0);
            robot.motorLeft1.setPower(0);
            robot.motorRight1.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    private void FlipSuckerDown()
    {
        double time = runtime.milliseconds();
        robot.MineralFlip.setPosition(0.52f);

        robot.suckerFlip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.suckerFlip.setTargetPosition(728);
        robot.suckerFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.suckerFlip.setPower(0.25f);
        while (robot.suckerFlip.isBusy() && runtime.milliseconds() - time < 1200.0 && !isStopRequested()) {

        }
        robot.suckerFlip.setPower(0f);
    }


    private void FlipSuckerUp()
    {

        if (robot.suckerFlip.getCurrentPosition() > 350) {
            robot.MineralFlip.setPosition(0.61f);
            robot.suckerFlip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.suckerFlip.setTargetPosition(350);
            robot.suckerFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.suckerFlip.setPower(-0.5f);
            while (robot.suckerFlip.isBusy() && !isStopRequested()) {


            }
        }


        robot.suckerFlip.setTargetPosition(145);
        robot.suckerFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.suckerFlip.setPower(-0.1f);
        while (robot.suckerFlip.isBusy() && !isStopRequested()) {


        }


        robot.suckerFlip.setPower(0f);
        robot.suckerFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void FlipSuckerMiddle()
    {

        robot.suckerFlip.setTargetPosition(200);
        robot.suckerFlip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.suckerFlip.setPower(0.2f);
        while (robot.suckerFlip.isBusy() && !isStopRequested()) {


        }


        robot.suckerFlip.setPower(0f);
        robot.suckerFlip.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void Sugere()
    {
        robot.Sucker.setPosition(0f);
        sleep(1000);
        robot.Sucker.setPosition(0.49);
    }
}