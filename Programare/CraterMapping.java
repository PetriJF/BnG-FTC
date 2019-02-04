import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="CraterMappping", group="Linear Opmode")

public class CraterMapping extends LinearOpMode {
    HardwareMap hwMap = null;
    int[][] Crater = new int[8][8];
    int cnti = 1, cntj = 1, col = 1;
    @Override
    public void runOpMode()
    {

        while (gamepad1.a || (cnti < 8 && cntj < 8))
        {
            if (gamepad1.x)
            {

                if (cntj == col)
                {
                    Crater[cnti][cntj] = 1;
                    cntj = 1;
                    cnti++;
                    col++;
                }
                else Crater[cnti][cntj++] = 1;
                sleep(300);

            }

            if (gamepad1.y)
            {
                if (cntj == col)
                {
                    Crater[cnti][cntj] = 0;

                    cntj = 1;
                    cnti++;
                    col++;
                }

                else cntj++;
                sleep(300);

            }
            telemetry.addData("i ", cnti);
            telemetry.addData("j ", cntj);
            telemetry.update();
        }
        telemetry.addData("mapping finished ", cnti);
        telemetry.update();

        waitForStart();

    }
}
