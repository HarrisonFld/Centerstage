package org.firstinspires.ftc.teamcode.auto.archived_auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoJavaBlueBackLeftPark", group = "Archived")
public class AutoJavaBlueBackLeftPark extends AutoJavaBlueBackBase {


    public AutoJavaBlueBackLeftPark() {
        super(true);
    }


    // Quick auto guide:
    // For turnBot, it turns in a clockwise direction. So negative values turn CCW, while positive values turn CW
    // For moveBot:
    // The robot will travel forward if the vertical parameter is positive. Otherwise, it will travel backwards.
    // The robot will travel right if the horizontal parameter is positive. Otherwise, it will travel left.

    // The "x inches input" comment is supposed to represent how many inches were initially plugged
    // into this equation: y=3.340x-2.356 (which is a lin reg model which converts inches to in-code units)
    // to get the value. The value will almost always be adjusted afterwards from its initial value for accuracy.
    @Override
    public void runOpMode() {

        this.commonAutoInit();
        this.tapeAuto();
        switch(pixelDetection.getPosition()) {
            case LEFT:
                sleep(500);
                moveBot(27.041668920139077, -1, 0, 0);
                sleep(500);
                moveBot(94.12500767708399, 0, 0, -1);
                sleep(500);
                trussArm();
                sleep(500);
                moveBot(276.37502303125194, 1, 0, 0);
                break;
            // TODO
            case CENTER:
                return;
            case RIGHT:
                sleep(500);
                moveBot(41.79, -1, 0, 0);
                sleep(500);
                turnBot(-115);
                sleep(500);
                moveBot(10.020834001736167, 0, 0, -1);
                sleep(500);
                trussArm();
                sleep(500);
                moveBot(302.27085852257158, 1, 0, 0);
                break;
        }
                sleep(500);
                moveBot(162.52084637673721, 0, 0, 1);
                moveBot(36.55208637934054, 1, 0, 0);



    }
}
