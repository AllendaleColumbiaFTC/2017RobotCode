package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    double SLOWSPEED = 0.1;
    int targetHeading = 0;
    int NAVTHRESHOLD = 3;
    int PACKEDTHETA0 = 0;  //position at match start
    int PACKEDTHETA1 = 225;
    int PACKEDTHETA2 = 50;
    int PACKEDTHETA3 = 190;
    int UNFURLEDTHETA0 = 0; //initial driving position
    int UNFURLEDTHETA1 = 7;
    int UNFURLEDTHETA2 = 89;
    int UNFURLEDTHETA3 = 0;



    public ConceptIK() {
        //Constructor
    }

    @Override
    public void init() {

        shoulder = hardwareMap.dcMotor.get("shoulder");
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(SLOWSPEED);
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);
        elbow = hardwareMap.dcMotor.get("elbow");
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(SLOWSPEED);
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        wrist = hardwareMap.servo.get("wrist");
        wrist.setPosition(.3);
        baseServo = hardwareMap.crservo.get("baseServo");

        last_x = 0;
        last_y = 0;
        last_z = 0;
        currentPos = new Position();

        lastTheta0 = PACKEDTHETA0;
        lastTheta1 = PACKEDTHETA1;
        lastTheta2 = PACKEDTHETA2;
        lastTheta3 = PACKEDTHETA3;
    }


    @Override
    public void loop() {
/*      boolean elbowAngleIncrease = gamepad1.y;
        boolean elbowAngleDecrease = gamepad1.a;
        boolean shoulderAngleIncrease = gamepad1.y;
        boolean shoulderAngleDecrease = gamepad1.a;

        if (shoulder.isBusy())
                return;

        if (shoulderAngleIncrease)
            turnShoulderMotor(2);
        if (shoulderAngleDecrease)
            turnShoulderMotor(-2);
*/
        boolean unfurlArmRequested = gamepad1.y && gamepad1.a;
        if (unfurlArmRequested && (lastTheta0==PACKEDTHETA0) && (lastTheta1==PACKEDTHETA1)
                && (lastTheta2==PACKEDTHETA2) && (lastTheta3==PACKEDTHETA3)) {
            unfurlArm();
        }


        /*
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
        armAngles = robotArm.InverseKinematics(currentPos, theta3);
        telemetry.addData("armAngles", armAngles);

        //run base, shoulder, elbow and wrist motors using encoders to achieve angles calculated by IK
        RunArmMotors(armAngles);
        lastTheta0 = armAngles.getTheta0();
        lastTheta1 = armAngles.getTheta1();
        lastTheta2 = armAngles.getTheta2();
        lastTheta3 = armAngles.getTheta3();
*/
    }


    private void RunArmMotors(ArmAngles armAngles) {     //four angles passed in

        // swing base to theta0 controlled by gyro
        //calculate required base turn
        newTheta0 = armAngles.getTheta0();
        double requiredTurn = newTheta0 - lastTheta0;
        //command base servo to turn required distance (robot cannot be turning while the base servo is turning)
        turnBaseServo(requiredTurn);

        // raise/lower shoulder to theta1 controlled by encoder
        newTheta1 = armAngles.getTheta1();
        double shoulderTurn = newTheta1 - lastTheta1;
        turnShoulderMotor(shoulderTurn);


        // extend/contract elbow to theta2 controlled by encoder
        newTheta2 = armAngles.getTheta2();
        double elbowTurn = newTheta2 - lastTheta2;
        turnElbowMotor(elbowTurn - shoulderTurn);

        // flex wrist to theta3 controlled by pitch servo
        wrist.setPosition(armAngles.getTheta3() / 200);
        //newTheta3 = armAngles.getTheta3();
        double wristTurn = elbowTurn; //TODO command to change from horizontal
        turnWristServo (elbowTurn);

        //turns completed! update values and send to Driver Station
        lastTheta0 = newTheta0;
        lastTheta1 = newTheta1;
        lastTheta2 = newTheta2;

        telemetry.addData("Theta0", lastTheta0);
        telemetry.addData("Theta1", lastTheta1);
        telemetry.addData("Theta2", lastTheta2);
    }

    private void turnBaseServo(double requiredTurn){
        double currentHeading = gyroSensor.getHeading();
        targetHeading = (int) Math.round(requiredTurn +  currentHeading);
        if (requiredTurn > 0) {
            servoMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            if (targetHeading < 360) {
                //do nothing
            } else if (targetHeading >= 360) {
                targetHeading = targetHeading - 360;
            }
            while ((gyroSensor.getHeading() - targetHeading) > NAVTHRESHOLD) {
                servoMotor.setPower(SLOWSPEED);
            }
        } else {
            servoMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            if (targetHeading >= 0) {
                //do nothing
            } else if (targetHeading < 0) {
                targetHeading = targetHeading + 360;
            }
            while ((gyroSensor.getHeading() - targetHeading) > NAVTHRESHOLD) {
                servoMotor.setPower(SLOWSPEED);
            }
        }
    }

    private void turnShoulderMotor(double turnDegrees){
        int currentShoulderMotorTicks = shoulder.getCurrentPosition();
        shoulder.setTargetPosition((int) (currentShoulderMotorTicks + Math.round((turnDegrees) * TICKS / 360)));
    }
    private void turnElbowMotor(double turnDegrees){
        int currentElbowMotorTicks = shoulder.getCurrentPosition();
        elbow.setTargetPosition((int) (currentElbowMotorTicks + Math.round((turnDegrees) * TICKS / 360)));
    }
    private void turnWristServo(double wristPosition){
        wrist.setPosition(wristPosition);
    }

    private void unfurlArm(){
        turnShoulderMotor(UNFURLEDTHETA1 - PACKEDTHETA1);
        turnElbowMotor(UNFURLEDTHETA2 - PACKEDTHETA2);
        try {
            while (shoulder.isBusy() || elbow.isBusy()){
                Thread.sleep(500); // pause for .5 seconds
            }
        }
        catch (InterruptedException intExc) {
            telemetry.addData("Interrupted!", intExc);
        }
        //unfurled!!  update lastTheta values and send info to DS
        lastTheta1 = UNFURLEDTHETA1;
        lastTheta2 = UNFURLEDTHETA2;
        telemetry.addData("Unfurl Complete. Theta1=", Double.toString(lastTheta1)
                + " Theta2=" + Double.toString(lastTheta2));
    }

    private void rePackArm(){
        turnBaseServo(PACKEDTHETA0 - lastTheta0);
        turnShoulderMotor(PACKEDTHETA1 - lastTheta1);
        turnElbowMotor(PACKEDTHETA2 - lastTheta2);
        try {
            while (shoulder.isBusy() || elbow.isBusy()){
                Thread.sleep(500); // pause for .5 seconds
            }
        }
        catch (InterruptedException intExc) {
            telemetry.addData("Interrupted!", intExc);
        }
        //packed!!  update lastTheta values and send info to DS
        lastTheta1 = PACKEDTHETA1;
        lastTheta2 = PACKEDTHETA2;
        telemetry.addData("rePack Complete. Theta1=", Double.toString(lastTheta1)
                + " Theta2=" + Double.toString(lastTheta2));
    }


}