package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoJavaBlue", group = "Auto")
public class AutoJavaBlue extends AutoJava {


    public AutoJavaBlue() {
        super(true, false);
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
        switch (pixelDetection.getPosition()) {

            case LEFT: {
                // 27 inches input
                moveBot(59.824, 0, 0, 1);
                sleep(1000);
// 45 inches input
                moveBot(93.944, 1, 0, 0);
                sleep(500);
                turnBot(-115);
                sleep(500);
                lowerArm();
                sleep(500);
// 39 inches input
// 27 inches input
// -5.5 inches input (80.824-16.014)
/*
moveBot(47.136, 1, 0, 0);
sleep(500);
// 8 inches input
moveBot(14.364, -1, 0, 0);
sleep(500);
tapePlace();
sleep(500);
moveBot(14.364, -1, 0, 0);
sleep(500);
*/
// 54 inches input
                moveBot(148.004, 1, 0, 0);
                break;

            }


            case CENTER: {

                moveBot(10, 0, 0, 1);
// 53.625 inches input
                moveBot(20, 1, 0, 0);
                sleep(500);
                lowerArm();
                sleep(500);
                moveBot(90, 1, 0, 0);
                sleep(1000);
                moveBot(30, -1, 0, 0);
                sleep(500);
                tapePlace();
                break;

            }


            case RIGHT: {

// 13.5 inches input PLUS 7.5 inches input (31.04 + 22.694)
                moveBot(57.934, 0, 0, 1);
                sleep(500);
// 29 inches input
                moveBot(24, 1, 0, 0);
                sleep(500);
                lowerArm();
                sleep(500);
                moveBot(74.504, 1, 0, 0);
                sleep(500);
// 10.5 inches input
                moveBot(32.714, -1, 0, 0);
                sleep(1000);
                tapePlace();
                break;

            }

        }


    }
}
