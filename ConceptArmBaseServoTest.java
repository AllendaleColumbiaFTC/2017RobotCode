package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Created by eileen on 10/29/17.
 */
@TeleOp(name = "Concept: ArmBaseServoTest", group = "Concept")
//@Disabled
public class ConceptArmBaseServoTest extends OpMode {


        CRServo servoMotor;
        GyroSensor gyroSensor;
        double SLOWSPEED = 0.04;
        //speed needs to be somewhere between 0.04 (too slow) and 0.4 (too fast)
        int targetHeading = 0;




        /**
         * Constructor
         */
        public ConceptArmBaseServoTest() {

        }

        @Override
        public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */


            servoMotor = hardwareMap.crservo.get("armBaseServo");
            servoMotor.setPower(0);
            gyroSensor = hardwareMap.gyroSensor.get("gyroSensor");
            gyroSensor.calibrate();

        }

        @Override
        public void loop() {

           // float armPan = gamepad1.left_stick_x;
            boolean turnClockwise90 = gamepad2.b;
            boolean turnCounterClockwise90 = gamepad2.x;

            //exit loop if not done calibrating
            if (gyroSensor.isCalibrating())
                    return;




            //function to have the correct target heading when using a joystick

            //way to have the correct targetHeading
            //if turning clockwise
            //turnSize = vector defined by right stick x and y values
            double turnSize  =  Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) * (180/Math.PI);
            double targetClockwise = Math.abs(turnSize + gyroSensor.getHeading());
            if(targetClockwise <= 360) {
                targetClockwise = targetClockwise;
            }
            else {
                while (targetClockwise > 360) {
                    targetClockwise = targetClockwise - 360;
                }
            }

            servoMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            while (gyroSensor.getHeading() < Math.abs(targetClockwise)) {
                servoMotor.setPower(SLOWSPEED);
            }



//            //if turning counterclockwise
//            double targetCounterClockwise =  Math.abs( turnSize- gyroSensor.getHeading());
//            if(targetCounterClockwise <= 360) {
//                targetCounterClockwise = targetCounterClockwise;
//            }
//            else if(targetCounterClockwise > 360){
//                targetCounterClockwise = targetCounterClockwise - 360;
//            }


            //function to have correct target heading (only 90 degree) built into function

            //X button = servo motor turns 90 degrees counter clockwise, B button = servo motor turns 90 degrees clockwise
            if (turnClockwise90) {
                servoMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                targetHeading=gyroSensor.getHeading()+90;
                if(targetHeading <= 360) {
                    while (gyroSensor.getHeading() < targetHeading) {
                        servoMotor.setPower(SLOWSPEED);
                    }
                    servoMotor.setPower(0);
                }
                else if (targetHeading > 360) {
                    targetHeading = Math.abs(targetHeading-360);
                    while (gyroSensor.getHeading() < targetHeading) {
                        servoMotor.setPower(SLOWSPEED);
                    }
                    servoMotor.setPower(0);
                }
            }

            if (turnCounterClockwise90) {
                servoMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                targetHeading = Math.abs(gyroSensor.getHeading() - 90);
                if (targetHeading <= 360) {
                    while (gyroSensor.getHeading() > targetHeading) {
                        servoMotor.setPower(SLOWSPEED);
                    }
                    servoMotor.setPower(0);
                }
                else if(targetHeading > 360){
                    targetHeading = Math.abs(targetHeading-360);
                        while (gyroSensor.getHeading() > targetHeading) {
                            servoMotor.setPower(SLOWSPEED);
                        }
                    servoMotor.setPower(0);
                }

            }


		/*
		 * Telemetry for debugging
		 */
            telemetry.addData("Text", "*** Robot Data ***");
            if (turnClockwise90){
                telemetry.addData("Button B was pressed", "armTest");
            }
            if (turnCounterClockwise90) {
                telemetry.addData("Button X was pressed", "armTest");
            }
            if(turnSize>0){
                telemetry.addData("joystick was moved", "armTest");
            }
            //telemetry.addData("RawZ", gyroSensor.rawZ() );
            telemetry.addData("getHeading", gyroSensor.getHeading());
            telemetry.addData("targetHeading", targetHeading);
            telemetry.addData("turnSize", turnSize);

        }

        @Override
        public void stop() {

        }




    }
