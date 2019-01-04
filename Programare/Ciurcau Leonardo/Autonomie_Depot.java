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

@Autonomous(name="Autonomie Depot", group="Linear Opmode")
//@Disabled
public class Autonomie_Depot extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Robot robot = new Robot();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AfOR5K3/////AAABmdlBtn+igEYQhEK8iiFVxKojIHnUdNvFj3IWgHLrGZcG17s9QYJOCPRRPf5NYeizXyxUsAmGyo8vErJwbq8xXsvs2Mpx9W8+Ndlq2oRNuGYctM4FLfjoa9hFj4YFZSOLNdtO5Rlb7/QLI8ydOjc7n2pIYPIY7hHLz+RuelzhvujdlvrvCU3XGilWESXpEymhEGfatbiRoSQAt67NTyw2XXx0hP8wt43OlAqrvIh2PIf3pInJgJS44aLtVsuZX2ErL+nuWzO9I8KTriPibgdM6PD2z6YJrW9dcRACjjhRAuqS3VY/eqEJ/BMqIVpXn8QvXd5Wr39UeATNP0an5HF76/9xx6ds0VHEXUJBWrovn7VY";
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.4;
    private VuforiaLocalizer vuforia;
    public String position;
    private TFObjectDetector tfod;
    boolean start = false;


    @Override
    public void runOpMode() {


        robot.init(hardwareMap);
        initTF();
        ActivateTF();
        RunTF();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        StopTF();
        //robot.latching(1400);
        // EncoderStrafe(DRIVE_SPEED, 3, 3, 3);
        encoderDrive(DRIVE_SPEED,15, 15, 3);
        encoderDrive(DRIVE_SPEED, -21, 21, 3);

        if(position == "RIGHT")
        {
            encoderDrive(-DRIVE_SPEED, -9, -9, 3);
            EncoderStrafe(DRIVE_SPEED, 20, 20, 10); // loveste cubul
            sleep(200);
            EncoderStrafe(-DRIVE_SPEED,-8, -8, 10); // se retrage
            encoderDrive(DRIVE_SPEED,51, 51, 3); // merge catre perete

        }

        else if (position == "CENTER")
        {
            // encoderDrive(DRIVE_SPEED, 4,4,3); // se aliniaza cu cubul
            EncoderStrafe(DRIVE_SPEED, 25, 25, 10); // loveste cubul
            sleep(200);
            EncoderStrafe(-DRIVE_SPEED,-10, -10, 10);// se retrage
            encoderDrive(DRIVE_SPEED, 42, 42,3); // merge catre perete
        }

        else {
            encoderDrive(DRIVE_SPEED, 25, 25, 3); // se aliniaza cu cubul
            EncoderStrafe(DRIVE_SPEED, 25, 25, 10); // loveste cubul
            sleep(200);
            EncoderStrafe(-DRIVE_SPEED, -10, -10, 10);// se retrage
            encoderDrive(DRIVE_SPEED,22, 22, 3);// merge catre perete
        }

        encoderDrive(DRIVE_SPEED, -10,10, 3); // se roteste pt a fi cu spatele la depot
        EncoderStrafe(DRIVE_SPEED, 8, 8, 3); //merge catre perete pt a se alinia
        EncoderStrafe(-DRIVE_SPEED, -3,-3,3); // se intoarce pt a nu se bloca de perete
        encoderDrive(-DRIVE_SPEED, -55, -55, 10);
        robot.plasareMarker();
        sleep(500);
        robot.inchidereMarker(); // pune marker^
        encoderDrive(DRIVE_SPEED, 63, 63, 10);// pleaca la crater
        //ExtendSucker (300); // parcheaza


        // MineralPhase();


        //ExtendSucker(200);
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

    private void MineralPhase()
    {
        // encoderDrive(DRIVE_SPEED, 8, 8, 4);
        robot.ExtendSucker(1050);
        sleep(500);
        robot.scuipare();
        sleep(700);
        robot.stopSugere();

        if(position == "CENTER")
        {
            robot.ExtendSucker(-550);
            sleep(500);
            //robot.flipDown();
            sleep(500);
            robot.sugere();
            sleep(500);
            robot.stopSugere();
            sleep(500);
            //robot.flipUp();
            robot.ExtendSucker(-500);
        }

        else
        {
            robot.ExtendSucker(-1050);

            encoderDrive(DRIVE_SPEED, 14, 14, 4);

            if (position == "LEFT")
                EncoderStrafe(DRIVE_SPEED, 18, 18, 4);
            else if (position == "RIGHT")
                EncoderStrafe(-DRIVE_SPEED, -18, -18, 4);


            sleep(250);
            robot.ExtendSucker(400);
            //robot.flipDown();
            sleep(250);
            robot.sugere();
            sleep(500);
            robot.stopSugere();
            sleep(250);
            //robot.flipUp();
            sleep(250);
            robot.ExtendSucker(-250);

            if (position == "LEFT")
                EncoderStrafe(-DRIVE_SPEED, -18, -18, 4);
            else if (position == "RIGHT")
                EncoderStrafe(DRIVE_SPEED, 18, 18, 4);

            // encoderDrive(-DRIVE_SPEED, -10,-10,4);
        }





        // encoderDrive(DRIVE_SPEED,8,8,4);
        encoderDrive(DRIVE_SPEED, -15, 15,4);
        encoderDrive(DRIVE_SPEED, 46, 46, 4);
        encoderDrive(DRIVE_SPEED, -9, 9, 4);


        //if (position == "RIGHT")
        //ExtendSucker(450);
        // else ExtendSucker(300);
    }

    public void initTF()
    {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    public void ActivateTF()
    {
        if (tfod != null) {
            tfod.activate();
        }
    }

    public void RunTF()
    {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            while(!opModeIsActive()) {
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
                                position = "LEFT";
                                telemetry.addData("Gold Mineral Position", "Left");
                            } else {
                                position = "CENTER";
                                telemetry.addData("Gold Mineral Position", "Center");
                            }
                        }
                        else {
                            position = "RIGHT";
                            telemetry.addData("Gold Mineral Position", "RIGHT");

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
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        robot.motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        robot.motorLeft2.setDirection(DcMotor.Direction.REVERSE);

        robot.motorRight1.setDirection(DcMotor.Direction.FORWARD);
        robot.motorRight2.setDirection(DcMotor.Direction.FORWARD);


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
            newLeftTarget = robot.motorLeft2.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorRight2.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.motorLeft2.setTargetPosition(newLeftTarget);
            robot.motorRight2.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorLeft2.setPower(Math.abs(speed));
            robot.motorRight2.setPower(Math.abs(speed));
            robot.motorLeft1.setPower(speed);
            robot.motorRight1.setPower(speed);

            if (leftInches < 0 && rightInches > 0)
                robot.motorLeft1.setPower(-speed);
            else robot.motorLeft1.setPower(speed);

            if (rightInches < 0 && leftInches > 0)
                robot.motorRight1.setPower(-speed);
            else robot.motorRight1.setPower(speed);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorLeft2.isBusy() && robot.motorRight2.isBusy())) {

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

    public void EncoderStrafe(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        robot.motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        robot.motorLeft2.setDirection(DcMotor.Direction.FORWARD);

        robot.motorRight1.setDirection(DcMotor.Direction.REVERSE);
        robot.motorRight2.setDirection(DcMotor.Direction.FORWARD);

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
            newLeftTarget = robot.motorLeft2.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorRight1.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.motorLeft2.setTargetPosition(newLeftTarget);
            robot.motorRight1.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
            while (opModeIsActive() &&
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

}