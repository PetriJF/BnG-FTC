import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;



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

@TeleOp(name="testServoContinuu", group="Linear Opmode")
public class testServoContinuu extends LinearOpMode
{

    // Declare OpMode members.
    private Servo servo = null;
    double pozitie = 0.5;
    HardwareMap hwMap = null;

    @Override
    public void runOpMode()
    {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(pozitie);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.dpad_up) {
                pozitie += 0.01;
                servo.setPosition(pozitie);
                sleep(200);
            }
            else if (gamepad1.dpad_down) {
                pozitie -= 0.01;
                servo.setPosition(pozitie);
                sleep(200);
            }

            telemetry.addData("pozitie ", pozitie);
            telemetry.update();
        }
    }
}
