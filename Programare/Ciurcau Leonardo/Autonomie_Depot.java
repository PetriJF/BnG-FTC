import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import java.nio.ByteBuffer;

import static java.lang.Math.atan;
import static java.lang.Math.toDegrees;


@Autonomous(name="Autonomie Depot", group="Linear Opmode")
public class Autonomie_Depot extends LinearOpMode
{

    private ElapsedTime runtime = new ElapsedTime();
    private int imageWidth, imageHeight, scale = 4, left = 0, center = 1, right = 2;
    private boolean[][] cube_color, lee_matrix;
    private DcMotor motorLeft1 = null;
    private DcMotor motorRight1 = null;
    private DcMotor motorLeft2 = null;
    VuforiaLocalizer vuforia;
    private DcMotor motorRight2 = null;
    private Servo markerRelease = null;

    private Object obj;
    private byte[] pixelArray;
    private VuforiaLocalizer.Parameters params = null;
    private VuforiaTrackables targetsRoverRuckus;
    private double Angle = 0;
    private String nav = "", S = "";
    private int initRotateTick;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.4;
    int moveAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {


        //tells VuforiaLocalizer to only store one frame at a time
        Init();
        waitForStart();
        runtime.reset();

        // nav = NavTargets();
        //MoveBack();

         moveAngle= XToAngle(MineralsPhase());

        telemetry.addData("Angle", Angle);
        telemetry.update();

        MoveMineral();
        GoToDepot();
    }

    public void Init()
    {


        params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "AfpNx0X/////AAAAGZqo1zzaiEnYutr1KL/q14AabZMVkQIew0DVRrGP3F+HyPHj4YEvNmrsUBFPix2HcpgAbnD7aH+/KCxsYf/WWTS4q15H39Y2M9g5M4Wud6WNufbs5HsGaB8ipxMCDpWqhYv6KgAihfO3Zq3az4utKZ94CIuXATsi3oNABVks52lkFng/9WOM0LaroowGIMFyprWRUjmfggfaGlu/6mpGSBkvpNRlD8TRXYt/W/Qho4B7G03thikhfsdpNRoNCcZMRaIzWyo5oGWzslxyvdDp0bxBQB8tZsTW8p288zfXBGtSQ8pTXsgKpmjwdStIP0pXjDJ9av0+CXhJblIGvcmvm7K5DqANWk0Z6whToWD0+K7S";
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

        vuforia = ClassFactory.createVuforiaLocalizer(params);
        targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
        targetsRoverRuckus.get(0).setName("Blue");
        targetsRoverRuckus.get(1).setName("Red");
        targetsRoverRuckus.get(2).setName("Front");
        targetsRoverRuckus.get(3).setName("Back");


        // For convenience, gather together all the trackable objects in one easily-iterable collection */


        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");
        markerRelease = hardwareMap.servo.get("markerRelease");
        markerRelease.setPosition(0.25f);
        initRotateTick = motorRight1.getCurrentPosition();

        telemetry.addData("Initialized", "");

    }

   /* public String NavTargets()
    {
        targetsRoverRuckus.activate();
        vuforia.setFrameQueueCapacity(30);

        rotate(-25);
        telemetry.update();
        boolean ok = true;
        while(ok)
            for (VuforiaTrackable trackable : targetsRoverRuckus)
            {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();

                if (pose != null)
                {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible())
                    {
                        telemetry.addData("Visible Target", trackable.getName());
                        return trackable.getName();
                    }
                }
            }

        return "NONE";
    }*/

   /* public void MoveBack()
    {
        rotate(25);

        setMotorSpeed(0f);

    }*/

   private int XToAngle(int x)
   {
       int maxInch = 8;
       boolean minus = (x > imageWidth / 2);
       if (minus) x = imageWidth - x;
       double angle0 = (int)((double)(maxInch) - (double)(2 * maxInch) * ( (double)(x) / (double)(imageWidth) ));
       if ( (angle0) - (int)(angle0) >= 0.5) angle0 = (int)(angle0 + 1);
       if (minus) angle0 = -angle0;
       return (int)angle0;
   }

    public int MineralsPhase() throws InterruptedException
    {
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        int ind = 1;
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();

        while (ind < 10)
        {
            frame = vuforia.getFrameQueue().take();
            ind++;
        }
        double whileStart = runtime.seconds();
        boolean ok = true;
        Object Cube = new Object();
        double angle = 0;

        while(ok)
        {
            frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
            Image img = null;
            long numImages = frame.getNumImages();

            for (int i = 0; i < numImages; i++)
            {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB888)
                {
                    img = frame.getImage(i);
                    if (img != null)
                    {

                        telemetry.addData("Status", "Start img processing: " + runtime.toString());
                        ByteBuffer pixels = img.getPixels();
                        pixelArray = new byte[pixels.remaining()];
                        pixels.get(pixelArray, 0, pixelArray.length);
                        pixels = null;
                        imageWidth = img.getWidth();
                        imageHeight = img.getHeight();
                /*int stride = img.getStride();
                  telemetry.addData("Image", "Image width: " + imageWidth);
                  telemetry.addData("Image", "Image height: " + imageHeight);
                  telemetry.addData("Image", "Image stride: " + stride);
                  telemetry.addData("Image", "First pixel byte: " + pixelArray[0]);
                  telemetry.addData("Length", pixelArray.length);
                  0, 0 right up
                  imageWidth - 1, imageHeight  - 1 left down*/

                        cube_color = new boolean[imageWidth / scale][imageHeight / scale];
                        FindColor(127, 90, 0);
                        pixelArray = null;

                        Cube = new Object();
                        lee_matrix = new boolean[imageWidth][imageHeight];

                        for (int y = 2 * imageHeight / 3; y < imageHeight; y++)
                            for (int x = 0; x < imageWidth; x++) {
                                if (!lee_matrix[x][y] && cube_color[x][y])
                                {
                                    obj = new Object();
                                    Lee(x, y);
                                    if (obj.size > Cube.size) Cube = obj;
                                }
                            }

                        String S;
                        if (Cube.size == 0)
                        {
                            return 0;
                        }
                        else S = Location(Cube.Loc);
                        telemetry.addData("LOCATION", S);
                        if (Cube.size != 0)
                        {
                            telemetry.addData("Cube X", Cube.x * scale);
                            telemetry.addData("Cube Y", Cube.y * scale);
                            telemetry.addData("Cube Size", Cube.size * scale * scale);
                            double tanA = (double)(Cube.x - imageWidth / 2) / (double)(imageHeight - Cube.y);
                            angle = toDegrees(atan(tanA));
                            return Cube.x;
                        }

                        return 0;
                    }
                }
            }

        }
        return 0;
    }

    public void MoveMineral()
    {
        /*if(S == "LEFT" || S == "NONE")
        {
            encoderDrive(TURN_SPEED, -4, 4, 10);
            encoderDrive(DRIVE_SPEED, 27, 27, 3);
        }
        else if (S == "RIGHT")
        {
            encoderDrive(TURN_SPEED, 4, -4, 10);
            encoderDrive(DRIVE_SPEED, 27, 27, 3);
        }
        else encoderDrive(DRIVE_SPEED, 25, 25, 3);*/

        encoderDrive(TURN_SPEED, -moveAngle, moveAngle, 10);

        if (moveAngle < 3 && moveAngle > -3) S = "CENTER";
        else if (moveAngle < -3) S = "RIGHT";
        else S = "LEFT";

        if (S == "CENTER")
            encoderDrive(DRIVE_SPEED, 75, 75, 3);
        else if (S == "LEFT")
            encoderDrive(DRIVE_SPEED, 44, 44, 3);
        else if (S == "RIGHT")
            encoderDrive(DRIVE_SPEED, 47, 47, 3);

        telemetry.addData("Angle", Angle);
        telemetry.update();

    }

    public void GoToDepot()
    {
        encoderDrive(TURN_SPEED, moveAngle, -moveAngle, 10);

        if (S == "RIGHT")
            encoderDrive(TURN_SPEED, -6, 6, 10);
        else if (S == "LEFT")
            encoderDrive(TURN_SPEED, 6, -6, 10);
        sleep(1000);

       // if(S == "CENTER")
          //  encoderDrive(TURN_SPEED, 11, -11, 10);
        if (S == "RIGHT" || S == "LEFT")
            encoderDrive(TURN_SPEED, cmToInches(60), cmToInches(60), 10);

        markerRelease.setPosition(1f);

        /*if (S == "CENTER")
            encoderDrive(TURN_SPEED, 12, -12, 10);
        // encoderDrive(-0.9, -cmToInches(100), -cmToInches(100), 15);
        if(S == "RIGHT")
            encoderDrive(TURN_SPEED, 19, -19, 10);
        if (S == "LEFT")
            encoderDrive(TURN_SPEED, 5, -5, 10);

        encoderDrive(-TURN_SPEED, -70, -70, 10);*/

    }


    private void moveRobotReverse(float speed) {
        motorLeft1.setDirection(DcMotor.Direction.FORWARD);
        motorLeft2.setDirection(DcMotor.Direction.FORWARD);
        motorRight1.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        setMotorSpeed(speed);
    }

    private void moveRobotForward(float speed) {
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);
        setMotorSpeed(speed);
    }

    private void rotateRobotRight(float speed) {
        motorLeft1.setDirection(DcMotor.Direction.FORWARD);
        motorLeft2.setDirection(DcMotor.Direction.FORWARD);
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);
        setMotorSpeed(speed);
    }

    private void rotateRobotLeft(float speed) {
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        setMotorSpeed(speed);
    }


    private void setMotorSpeed(float speed) {
        motorLeft2.setPower(speed);
        motorRight2.setPower(speed);
        motorLeft1.setPower(speed);
        motorRight1.setPower(speed);
    }
    private void FindColor(int r, int g, int b){

        int scale2 = scale * scale, minD = 5000;//minD = minimal difference between 2 colors for comparision
        for (int y = 0; y < imageHeight - scale; y += scale)
            for (int x = 0; x < imageWidth - scale; x += scale)
            {

                int[] pixel;
                int sRed = 0;
                int sGreen = 0;
                int sBlue = 0;

                for (int j = y; j < y + scale; j++)
                    for (int i = x; i < x + scale; i++)
                    {
                        pixel = GetRGB(i, j);
                        sRed += pixel[0];
                        sGreen += pixel[1];
                        sBlue += pixel[2];
                    }

                int d = Distance(sRed / scale2, sGreen / scale2, sBlue / scale2, r, g, b);
                if(d < minD)
                    cube_color[x / scale][y / scale] = true;
            }

        imageWidth /= scale;
        imageHeight /= scale;

    }

    private int[] GetRGB(int x, int y){

        int index = 3 * (y * imageWidth + x);
        int[] pixel = new int[3];
        for (int i = index; i <= index + 2; i++)
        {
            if (pixelArray[i] < 0)
                pixelArray[i] += 128; //sometimes it gives negative values: -1 = 127, -2 = 126 etc
            pixel[i - index] = pixelArray[i];
        }
        return pixel;
    }

    private int Distance(int x1, int y1, int z1, int x2, int y2, int z2){

        int dx = x1 - x2, dy = y1 - y2, dz = z1 - z2;
        return dx * dx + dy * dy + dz * dz;
    }

    class Object{
        int x, y, size;
        int[] Loc = new int[3];
        private Object(){}
    }

    private void Lee(int x, int y){

        int[] dx = {-1, 0, 1, -1, 1, -1, 0, 1};
        int[] dy = {-1, -1, -1, 0, 0, 1, 1, 1};
        int[][] Q = new int[imageWidth * imageHeight + 1][2];
        int st = 0, dr = 1;

        obj.x = x;
        obj.y = y;

        Q[st][0] = x;
        Q[st][1] = y;
        lee_matrix[x][y] = true;

        while(st < dr)
        {
            ++obj.size;
            CheckPos(Q[st][0]);
            for (int d = 0; d < 8; d++)
            {
                int x2 = Q[st][0] + dx[d], y2 = Q[st][1] + dy[d];
                if (Inside(x2, y2) && !lee_matrix[x2][y2] && cube_color[x2][y2])
                {
                    lee_matrix[x2][y2] = true;
                    Q[dr][0] = x2;
                    Q[dr][1] = y2;
                    ++dr;
                }
            }
            ++st;
        }

    }

    private void CheckPos(int x){

        if(x < imageWidth / 3)++obj.Loc[left];
        else if(x < 2 * imageWidth / 3)++obj.Loc[center];
        else ++obj.Loc[right];
    }

    private boolean Inside(int x, int y){

        return (x >= 0 && x < imageWidth && y >= 0 && y < imageHeight);
    }

    private String Location(int[] Loc){

        int locMax = Loc[left];
        if (Loc[center] > locMax)
            locMax = Loc[center];
        if (Loc[right] > locMax)
            locMax = Loc[right];

        if (Loc[left] == locMax) return "LEFT";
        if (Loc[center] == locMax) return "CENTER";
        if (Loc[right] == locMax) return "RIGHT";

        return "?";
    }

    private int cmToInches(int cm)
    {
        return (int)((double)cm / 2.54);
    }

    public void rotate(int degrees)
    {

        int ticks = toTicks(degrees);

        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorSpeed(ticks > 0 ? 0.4f : -0.4f);

        int startPos = Math.abs(motorRight2.getCurrentPosition());
        while (Math.abs(motorRight2.getCurrentPosition()) < Math.abs(ticks) + startPos)
        {
            telemetry.addData("Ticks", motorRight2.getCurrentPosition());
            telemetry.addData("Goal", ticks);
            telemetry.update();
        }

        setMotorSpeed(0f);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(350);

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        motorLeft1.setDirection(DcMotor.Direction.FORWARD);
        motorLeft2.setDirection(DcMotor.Direction.FORWARD);
        motorRight1.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);

        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorLeft2.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motorRight2.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            motorLeft2.setTargetPosition(newLeftTarget);
            motorRight2.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorLeft2.setPower(Math.abs(speed));
            motorRight2.setPower(Math.abs(speed));
            motorLeft1.setPower(speed);
            motorRight1.setPower(speed);

            if (leftInches < 0 && rightInches > 0)
                motorLeft1.setPower(-speed);
            else motorLeft1.setPower(speed);

            if (rightInches < 0 && leftInches > 0)
                motorRight1.setPower(-speed);
            else motorRight1.setPower(speed);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorLeft2.isBusy() && motorRight2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorLeft2.getCurrentPosition(),
                        motorRight2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorLeft2.setPower(0);
            motorRight2.setPower(0);
            motorLeft1.setPower(0);
            motorRight1.setPower(0);
            // Turn off RUN_TO_POSITION
            motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public int toTicks(int degrees)
    {
        int maxTick = 2300;
        return maxTick * degrees / 360;
    }


}

