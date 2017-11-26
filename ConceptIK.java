

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


/**
 * Created by Liza on 10/22/17.
 */


@TeleOp(name = "Concept: IK", group = "Concept")
//@Disabled

public class ConceptIK extends OpMode {

    DcMotor shoulder; //am 3104
    DcMotor elbow;    //am 3104
    Servo wrist;
    CRServo baseServo;
    int TICKS = 7168;  // number of motor ticks per revolution
    double last_x, last_y, last_z; // previous (x,y,z) position
    double new_x, new_y, new_z;      // requested next (x,y,z) position
    double lastTheta0, lastTheta1, lastTheta2, lastTheta3;
    double newTheta0, newTheta1, newTheta2, newTheta3;
    final double QUANTA = 0.1;       // maximum distance away from current position in each dimension
    Position currentPos;
    CRServo servoMotor;
    GyroSensor gyroSensor;
    double SLOWSPEED = 0.4;
    int targetHeading = 0;
    int NAVTHRESHOLD = 3;

    public ConceptIK() {
        //Constructor

    }

    @Override
    public void init() {

        shoulder = hardwareMap.dcMotor.get("shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(0.0);
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);
        elbow = hardwareMap.dcMotor.get("elbow");
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(0.0);
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        wrist = hardwareMap.servo.get("wrist");
        baseServo = hardwareMap.crservo.get("baseServo");

        last_x = 0;
        last_y = 0;
        last_z = 0;
        currentPos = new Position();
    }


    @Override
    public void loop() {
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
        //RunArmMotors(ArmAngles);
    }


    private void RunArmMotors(double[] ArmAngles) {     //four angles passed in

        // swing base to theta0 controlled by gyro
        lastTheta0 = gyroSensor.getHeading();
        if (lastTheta0 < ArmAngles[0]) {
            servoMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            targetHeading = (int) Math.round(ArmAngles[0]);
            if (targetHeading < 360) {
                //do nothing
            } else if (targetHeading >= 360) {
                targetHeading = targetHeading - 360;
            }
            while (Math.abs(gyroSensor.getHeading() - targetHeading) > NAVTHRESHOLD) {
                servoMotor.setPower(SLOWSPEED);
            }
        } else {
            servoMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            targetHeading = gyroSensor.getHeading() - 90;
            if (targetHeading >= 0) {
                //do nothing
            } else if (targetHeading < 0) {
                targetHeading = targetHeading + 360;
            }
            while (Math.abs(gyroSensor.getHeading() - targetHeading) > NAVTHRESHOLD) {
                servoMotor.setPower(SLOWSPEED);
            }
        }

        // raise/lower shoulder to theta1 controlled by encoder
        shoulder.setTargetPosition((int) Math.round(ArmAngles[1] * TICKS / 360));

        // extend/contract elbow to theta2 controlled by encoder
        elbow.setTargetPosition((int) Math.round(ArmAngles[2] * TICKS / 360));

        // flex wrist to theta3 controlled by pitch servo
        wrist.setPosition(ArmAngles[3] / 200);


    }
}