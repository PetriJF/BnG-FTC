import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.tree.DCTree;


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

@TeleOp(name="TestMotoareEncoder", group="Linear Opmode")
public class TestMotoareEncoder extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motor4 = null;



    HardwareMap hwMap = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // motor2 = hardwareMap.get(DcMotor.class, "motor2");
        // motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1 = hardwareMap.get(DcMotor.class, "motor1 ");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setPower(0f);
        motor2.setPower(0f);
        motor3.setPower(0f);
        motor4.setPower(0f);


        // motor2.setPower(0f);
        waitForStart();

        motor1.setPower(0.5f);
        motor2.setPower(0.5f);
        motor3.setPower(0.5f);
        motor4.setPower(0.5f);
        sleep(3000);

        motor1.setPower(0f);
        motor2.setPower(0f);
        motor3.setPower(0f);
        motor4.setPower(0f);

        sleep(1000);

        motor1.setPower(0.5f);
        motor2.setPower(-0.5f);
        motor3.setPower(-0.5f);
        motor4.setPower(0.5f);
        sleep(3000);

        motor1.setPower(0f);
        motor2.setPower(0f);
        motor3.setPower(0f);
        motor4.setPower(0f);

    }
}
