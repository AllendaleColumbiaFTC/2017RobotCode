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
 * Created by aditiseshadri on 11/15/17.
 */

@TeleOp(name = "Concept: ArmJointTest", group = "Concept")
//@Disabled
public class ConceptArmJointTest extends OpMode {
    DcMotor neveRestMotor;

    public ConceptArmJointTest() {

    }

    @Override
    public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */


        neveRestMotor = hardwareMap.dcMotor.get("testNeveRest");
        neveRestMotor.setPower(0);
        neveRestMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        neveRestMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d",
                neveRestMotor.getCurrentPosition());
        telemetry.update();
    }


//    public void setJointPower(double power)
//    {
//        if (power < 0.0 && neveRestMotor.getCurrentPosition() > 0 || power > 0.0 && neveRestMotor.getCurrentPosition() < 12000)
//        {
//            // We only allow the motor to move if we are retracting and encoder is still positive or if we are extending and encoder has not
//            // exceeded 12000.
//            neveRestMotor.setPower(power);
//        }
//        else
//        {
//            neveRestMotor.setPower(0.0);
//        }
//    }

    @Override
    public void loop()
    {
        double power = gamepad1.right_stick_y;
        neveRestMotor.setPower(power);


        //Telemetry
        telemetry.addData("Text", "*** Robot Data ***");
        telemetry.addData("Power for arm joint", "joint0");
    }

    @Override
    public void stop() {

    }
}
