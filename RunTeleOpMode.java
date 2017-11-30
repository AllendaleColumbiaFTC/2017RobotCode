package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.code.Attribute;
import org.firstinspires.ftc.teamcode.RobotArmMath;
import org.firstinspires.ftc.teamcode.Position;
import org.firstinspires.ftc.teamcode.ConceptIK;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.teamcode.HolonomicDrive;
import org.firstinspires.ftc.teamcode.ArmAngles;

/**
 * Created by aditiseshadri on 11/29/17.
 */
@TeleOp(name="TeleOp Mode", group="Pushbot")


public class RunTeleOpMode extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    @Override
    public void init(){
        ConceptIK armAngleControl = new ConceptIK();
        HolonomicDrive holonomicControl = new HolonomicDrive();
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //

    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Code for controlling arm angles
        Position currentPos;
        currentPos = new Position();
        ArmAngles armAngles = new ArmAngles();
        //get driver inputs
        boolean zright = gamepad1.dpad_right;
        boolean zleft = gamepad1.dpad_left;
        boolean xforward = gamepad1.dpad_up;
        boolean xback = gamepad1.dpad_down;
        double yupdown = gamepad1.right_stick_y;
        double theta3 = 0; //TODO: Update via gamepad2.x and gamepad2.y
        //TODO: Keep track of  "oldTheta3" variable across calls of loop()


        telemetry.addData("R L F B Z", Boolean.toString(zright) + Boolean.toString(zleft) + Boolean.toString(xforward) +
                Boolean.toString(xback) + Double.toString(yupdown));

        //calculate desired newPosition
        telemetry.addData("lastPosition", currentPos);
        currentPos = currentPos.calcNewPosition(zright, zleft, xforward, xback, yupdown);
        telemetry.addData("newPosition", currentPos);

        //calculate desired joint angles
        RobotArmMath robotArm = new RobotArmMath();
        armAngles = robotArm.InverseKinematics(currentPos,theta3);
        telemetry.addData("armAngles", armAngles);

        //run base, shoulder, elbow and wrist motors using encoders to achieve angles calculated by IK
        //In order for this code to work, next line needs to be UNCOMMENTED
        //RunArmMotors(ArmAngles);


        //Code for Holonomic Drive
        // left stick controls direction
        // right stick X controls rotation

        float gamepad2LeftY = -gamepad2.left_stick_y;
        float gamepad2LeftX = gamepad2.left_stick_x;
        float gamepad2RightX = gamepad2.right_stick_x;

        // holonomic formulas

        float FrontLeft = -gamepad2LeftY - gamepad2LeftX - gamepad2RightX;
        float FrontRight = gamepad2LeftY - gamepad2LeftX - gamepad2RightX;
        float BackRight = gamepad2LeftY + gamepad2LeftX - gamepad2RightX;
        float BackLeft = -gamepad2LeftY + gamepad2LeftX - gamepad2RightX;

        // clip the right/left values so that the values never exceed +/- 1
        FrontRight = Range.clip(FrontRight, -1, 1);
        FrontLeft = Range.clip(FrontLeft, -1, 1);
        BackLeft = Range.clip(BackLeft, -1, 1);
        BackRight = Range.clip(BackRight, -1, 1);

        // write the values to the motors
        motorFrontRight.setPower(FrontRight * 0.4);
        motorFrontLeft.setPower(FrontLeft * 0.4);
        motorBackLeft.setPower(BackLeft * 0.4);
        motorBackRight.setPower(BackRight * 0.4);

        //Test which motor is which by pressing gamepad buttons (y = front right, a = front left, b = back right, x = back left
        if (gamepad2.y)
            motorFrontRight.setPower(0.02);
        else if (gamepad2.a)
            motorFrontLeft.setPower(0.02);
        else if (gamepad2.b)
            motorBackRight.setPower(0.02);
        else if (gamepad2.x)
            motorBackLeft.setPower(0.02);



		/*
		 * Telemetry for debugging
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Joy XL YL XR",  String.format("%.2f", gamepad2LeftX) + " " +
                String.format("%.2f", gamepad2LeftY) + " " +  String.format("%.2f", gamepad2RightX));
        telemetry.addData("f left pwr",  "front left  pwr: " + String.format("%.2f", FrontLeft));
        telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FrontRight));
        telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BackRight));
        telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BackLeft));


    }

}

