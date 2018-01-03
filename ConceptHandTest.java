package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by eileen on 11/19/17.
 */
@TeleOp(name = "Concept: HandTest", group = "Concept")
public class ConceptHandTest extends OpMode {


    Servo gripperServo;
    Servo yawServo;
    //Servo pitchServo;


    /**
     * Constructor
     */
    public ConceptHandTest() {

    }

    @Override
    public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */


        gripperServo = hardwareMap.servo.get("gripperServo");
        gripperServo.setPosition(0.0);

        yawServo = hardwareMap.servo.get("yawServo");
        yawServo.setPosition(0.0);

        //pitchServo = hardwareMap.servo.get("pitchServo");
        //pitchServo.setPosition(0.0);


    }

    @Override
    public void loop() {
        boolean movehandsmall = gamepad1.b;
        boolean movehandabitmore = gamepad1.x;
        boolean movehandmedium = gamepad1.y;
        boolean movehandextralarge = gamepad1.a;

        if (movehandsmall) {
            gripperServo.setPosition(0.25);
            String position = Double.toString(gripperServo.getPosition());
            telemetry.addData("gripperServoPosition" , gripperServo.getPosition());
        }
        if (movehandabitmore) {
            gripperServo.setPosition(0.5);
        }
        if (movehandmedium) {
            gripperServo.setPosition(.75);
        }
        if (movehandextralarge) {
            gripperServo.setPosition(1);
        }


        boolean movehandsmallyaw = gamepad2.b;
        boolean movehandabitmoreyaw = gamepad2.x;
        boolean movehandmediumyaw = gamepad2.y;
        boolean movehandextralargeyaw = gamepad2.a;

        if (movehandsmallyaw) {
            yawServo.setPosition(0.25);
        }
        if (movehandabitmoreyaw) {
            yawServo.setPosition(0.5);
        }
        if (movehandmediumyaw) {
            yawServo.setPosition(.75);
        }
        if (movehandextralargeyaw) {
            yawServo.setPosition(1);
        }






    }

    @Override
    public void stop() {

    }
}
