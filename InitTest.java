package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by aditiseshadri on 12/3/17.
 */
@Autonomous(name="InitTest")
public class InitTest extends OpMode
{
    Servo wrist;
    private ElapsedTime runtime = new ElapsedTime();
    public boolean initsecondMovement;
    public boolean firstMovement;
    public boolean secondMovement;
    public boolean thirdMovement;
    public boolean fourthMovement;
    public int loopCount;

    public double firstTarget;
    public double secondTarget;
    public double thirdTarget;
    public double fourthTarget;

    // Here is the init() method. We don't have anything to do here so we could
    // have left it out.

    @Override
    public void init()
    {
        runtime.reset();

        initsecondMovement = true;
        firstMovement = true;
        secondMovement = true;
        thirdMovement = true;
        fourthMovement = true;
        firstTarget = 1;
        secondTarget = 0.75;
        thirdTarget = 0.0;
        fourthTarget = 1.0;


        wrist = hardwareMap.servo.get("wrist");
        telemetry.addData("Status", "wrist has been initialized");
    //    telemetry.update();

        wrist.setPosition(0.5);
        telemetry.addData( "in init 0.5", wrist.getPosition());
   //     telemetry.update();
        loopCount = 0;



    }

    // Here we are intializing the variables we are using each time we run the
    // OpMode.

    @Override
    public void init_loop()
    {
        if (initsecondMovement){
            wrist.setPosition(0.75);
            telemetry.addData("in init 0.75", wrist.getPosition());
       //     telemetry.update();
            initsecondMovement = false;

        }

    }

    // Here are the start() and stop() methods

    @Override
    public void start()
    {
        //no movement of servo occurs within start
//        wrist.setPosition(0);
//        telemetry.addData("Status", "wrist should have moved to 0");
//        telemetry.update();

    }

    @Override
    public void stop()
    {
    }

    // The loop() method is called over and over until the Stop button is pressed.
    // The method displays the elapsed run time on the driver station using the
    // telemtery field of the base OpMode class.

    //Is Busy Method for Servos
    // Create booleans such as stillMoving and have a series of if statements
    // so that the next servo.setPosition is not called until stillMoving is set to false within the if statement
    public boolean busy(double targetPosition){
        boolean moving;
        if(wrist.getPosition() == targetPosition){
            moving = false;
        }
        else{
            moving = true;
        }
        return moving;
    }

    @Override
    public void loop()
    {
//        telemetry.addData("loopCount", loopCount);
//        telemetry.addData("ifStatementCount", ifCount);
//        telemetry.update();
        if(firstMovement){
            wrist.setPosition(firstTarget);
            telemetry.addData("Status", "wrist should have moved to 1");
    //        telemetry.update();
            while (busy(firstTarget)){
                firstMovement = true;
            }
            firstMovement = false;
        }

         else if (secondMovement && !firstMovement) {
                wrist.setPosition(secondTarget);
                telemetry.addData("in start 0.75", wrist.getPosition());
                //        telemetry.update();
             while (busy(secondTarget)){
                 secondMovement = true;
             }
             secondMovement = false;
         }
         else if (thirdMovement && !secondMovement && !firstMovement){
             wrist.setPosition(thirdTarget);
             telemetry.addData("in start 0.0", wrist.getPosition());
             while (busy(thirdTarget)){
                 thirdMovement = true;
             }
             thirdMovement = false;
         }
         else if (fourthMovement && !thirdMovement && !secondMovement && !firstMovement){
             wrist.setPosition(fourthTarget);
             telemetry.addData("in start 1.0", wrist.getPosition());
             while (busy(fourthTarget)){
                 fourthMovement = true;
             }
             fourthMovement = false;
         }

         else if (!fourthMovement && !thirdMovement && !secondMovement && !firstMovement){
            firstMovement = true;
            secondMovement = true;
            thirdMovement = true;
            fourthMovement = true;
        }
       /* fourthMovement = true;
        thirdMovement = true;
        secondMovement = true;*/
       loopCount +=1;
    }

}
