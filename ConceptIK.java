package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by Liza on 10/22/17.
 */


@TeleOp(name = "Concept: IK", group = "Concept")
//@Disabled

public class ConceptIK extends OpMode {
    DcMotor shoulder, elbow; //am 3104, arm motors
    DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight;  //drive motors
    Servo wristPitch, wristYaw, wristOpenClose;
    CRServo baseServo;
    GyroSensor gyroSensor;

    int TICKS = 7168;  // number of motor ticks per revolution (28ppr * 256/1)
    double last_x, last_y, last_z; // previous (x,y,z) position
    double new_x, new_y, new_z;      // requested next (x,y,z) position
    double lastTheta0, lastTheta1, lastTheta2, lastTheta3;
    double newTheta0, newTheta1, newTheta2, newTheta3;
    Position currentPos;

    double SLOWSPEED = 0.2;
    double BASESERVOSLOWSPEED = 0.07;
    int targetHeading = 0;
    double NAVTHRESHOLD = 0.5;
    int PACKEDTHETA0 = 0;  //position at match start
    int PACKEDTHETA1 = 230;
    int PACKEDTHETA2 = 50;
    int PACKEDTHETA3 = 190;
    double UNFURLEDTHETA0 = 0; //initial driving position
    double UNFURLEDTHETA1 = -7.78;
    double UNFURLEDTHETA2 = -135.9;
    double UNFURLEDTHETA3 = 0;

    //public boolean moving;

    enum STATE {PACKED, UNPACKING, UNPACKED, REPACKING, MOVINGARM, DRIVING, SLOWDRIVING};
    STATE robotState = STATE.PACKED;

    public ConceptIK() {
        //Constructor
    }

    @Override
    public void init() {
        shoulder = hardwareMap.dcMotor.get("shoulder");
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shoulder.setPower(SLOWSPEED);
        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);

        elbow = hardwareMap.dcMotor.get("elbow");
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbow.setPower(SLOWSPEED);
        elbow.setDirection(DcMotorSimple.Direction.FORWARD);

        wristPitch = hardwareMap.servo.get("wristPitch");
        slowMove(wristPitch,.5);
        //telemetry.addData("pitchPosition", Double.toString(wristPitch.getPosition()));

        wristYaw = hardwareMap.servo.get("wristYaw");
        //slowMove(wristYaw, .1);
        //telemetry.addData("yawPosition", Double.toString(wristYaw.getPosition()));

        wristOpenClose = hardwareMap.servo.get("wristOpenClose");
        //slowMove(wristOpenClose, .5);
        //telemetry.addData("openClosePosition", Double.toString(wristOpenClose.getPosition()));

        baseServo = hardwareMap.crservo.get("baseServo");
        baseServo.setPower(0.0);

        gyroSensor = hardwareMap.gyroSensor.get("gyroSensor");
        gyroSensor.calibrate();
        int i = 0;
        while (gyroSensor.isCalibrating()) {
            telemetry.addData("Gyro Calibrating... ", i++);
        }
        gyroSensor.resetZAxisIntegrator();

        last_x = -12;
        last_y = -12;
        last_z = 0;
        currentPos = new Position(last_x, last_y, last_z);

        robotState = STATE.PACKED;
        lastTheta0 = PACKEDTHETA0;
        lastTheta1 = PACKEDTHETA1;
        lastTheta2 = PACKEDTHETA2;
        lastTheta3 = PACKEDTHETA3;

        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
    }


    @Override
    public void loop() {
        //read driver inputs!!!

        //State change requests
        boolean unfurlArmRequested = gamepad1.y && gamepad1.a;
        boolean packArmRequested = gamepad1.b && gamepad1.x;
        boolean drivingRequested = (gamepad1.left_trigger>0) && (gamepad1.right_trigger>0);
        boolean moveArmRequested = (gamepad2.left_trigger>0) && (gamepad2.right_trigger>0);


        //Driving inputs
        float gamepad1LeftY = -gamepad1.left_stick_y;
        float gamepad1LeftX = gamepad1.left_stick_x;
        float gamepad1RightX = gamepad1.right_stick_x;


        //Individidual joint tests
        boolean baseAngleIncrease = gamepad2.right_bumper;
        boolean baseAngleDecrease = gamepad2.left_bumper;

        boolean shoulderAngleIncrease = gamepad1.dpad_down;
        boolean shoulderAngleDecrease = gamepad1.dpad_up;

        boolean elbowAngleIncrease = gamepad1.dpad_right;
        boolean elbowAngleDecrease = gamepad1.dpad_left;

        boolean wristPitchIncrease = gamepad1.right_bumper;
        boolean wristPitchDecrease = gamepad1.left_bumper;

        boolean wristYawIncrease = gamepad1.b;
        boolean wristYawDecrease = gamepad1.x;

        boolean wristOpen = gamepad1.a;
        boolean wristClose = gamepad1.y;

        //Decide what to do about Driver Inputs based on robotState
        if (robotState==STATE.PACKED){
            if (unfurlArmRequested) {
                if ((lastTheta0 == PACKEDTHETA0) && (lastTheta1 == PACKEDTHETA1)
                        && (lastTheta2 == PACKEDTHETA2) && (lastTheta3 == PACKEDTHETA3)
                        && (!gyroSensor.isCalibrating()) && (!shoulder.isBusy())
                        && (!elbow.isBusy())) {
                    telemetry.addData("PACKED=>UNPACKING", "in process...");
                    unfurlArm();
                    robotState = STATE.UNPACKING;
                    return;
                }
            }
            if (drivingRequested){
                telemetry.addData("PACKED=>DRIVING", "in process...");
                robotState = STATE.DRIVING;
                return;
            }
        }

        if (robotState==STATE.UNPACKING){
            if (shoulder.isBusy() || elbow.isBusy()) {
                telemetry.addData("ShoulderPosition: ", shoulder.getCurrentPosition());
                telemetry.addData("ElbowPosition: ", elbow.getCurrentPosition());
                return;
            }
            else {  //UNPACKED!!
                lastTheta0 = 0;
                lastTheta1 = UNFURLEDTHETA1;
                lastTheta2 = UNFURLEDTHETA2;
                lastTheta3 = 0;
                last_x = 18.0;
                last_y = 3.0;
                last_z = 0.0;
                currentPos = new Position(last_x, last_y, last_z);

                telemetry.addData("UNPACKING=>UNPACKED", "complete");
                telemetry.addData("Theta1=", Double.toString(lastTheta1)
                        + " Theta2=" + Double.toString(lastTheta2));
                telemetry.update();
                robotState = STATE.UNPACKED;
                return;
            }
        }

        if (robotState==STATE.UNPACKED){
            if (drivingRequested) {
                telemetry.addData("UNPACKED=>DRIVING", "complete");
                telemetry.update();
                robotState = STATE.DRIVING;
                return;
            }

            if (moveArmRequested) {
                telemetry.addData("lastPosition", currentPos);
                telemetry.addData("UNPACKED=>MOVINGARM", "complete");
                telemetry.update();
                robotState = STATE.MOVINGARM;
                return;
            }

            if (packArmRequested) {
                rePackArm();
                telemetry.addData("UNPACKED=>REPACKING", "complete");
                robotState = STATE.REPACKING;
                return;
            }
            // Tests
            // telemetry.addData("GyroReading" , gyroSensor.getHeading());
            if (baseAngleIncrease) {
                //telemetry.addData("Angle Increase" , "baseServo");
                turnBaseServo(2);
            }
            if (baseAngleDecrease) {
                //telemetry.addData("Angle Decrease" , "baseServo");
                turnBaseServo(-2);
            }
            if (shoulderAngleIncrease)
                turnShoulderMotor(0.2);
            if (shoulderAngleDecrease)
                turnShoulderMotor(-0.2);

            if (elbowAngleIncrease) {
                 turnElbowMotor(0.2);
                //moveElbowMotorToAngle(PACKEDTHETA2);
            }
            if (elbowAngleDecrease) {
                 turnElbowMotor(-0.2);
                //moveElbowMotorToAngle(UNFURLEDTHETA2);
            }
            if (wristPitchIncrease) {
                //telemetry.addData("Pitch increase", wristPitch.getPosition());
                turnWristPitchServo(5);
            }
            if (wristPitchDecrease) {
                //telemetry.addData("Pitch decrease", wristPitch.getPosition());
                turnWristPitchServo(-5);
            }
            if (wristYawIncrease)
                turnWristYawServo(5);
            if (wristYawDecrease)
                turnWristYawServo(-5);

            if (wristOpen)
                turnWristOpenCloseServo(5);
            if (wristClose)
                turnWristOpenCloseServo(-5);

            return;
        }

        if (robotState==STATE.REPACKING){
            if (baseBusy() || shoulder.isBusy() || elbow.isBusy() || wristPitchBusy(.6)) {
                telemetry.addData("ShoulderPosition: ", shoulder.getCurrentPosition());
                telemetry.addData("ElbowPosition: ", elbow.getCurrentPosition());
                return;
            }
            else {  //PACKED!!
                telemetry.addData("UNPACKED=>PACKED", "complete");
                robotState = STATE.PACKED;
                return;
            }
        }

        if  (robotState==STATE.MOVINGARM) {
            if (packArmRequested) {
                rePackArm();
                telemetry.addData("UNPACKED=>REPACKING", "complete");
                robotState = STATE.REPACKING;
                return;
            }
            if (drivingRequested) {
                telemetry.addData("MOVINGARM=>DRIVING", "complete");
                robotState = STATE.DRIVING;
                return;
            }
            if (elbow.isBusy() || shoulder.isBusy() || baseBusy()){
                return;
            }

            ArmAngles armAngles = new ArmAngles();

            //get driver inputs
            boolean zright = gamepad2.dpad_right;
            boolean zleft = gamepad2.dpad_left;
            boolean xforward = gamepad2.dpad_up;
            boolean xback = gamepad2.dpad_down;
            double yupdown = gamepad2.right_stick_y;
            double theta3 = 0; //TODO: Update via gamepad2.x and gamepad2.y
            // String temp = Boolean.toString(zright) + Boolean.toString(zleft) + Boolean.toString(xforward) +
            //        Boolean.toString(xback) + Double.toString(yupdown);
            // telemetry.addData("R L F B Z", temp);

            //calculate desired newPosition
            //telemetry.addData("lastPosition", currentPos);
            currentPos.updatePosition(zright, zleft, xforward, xback, yupdown);
            telemetry.addData("newPosition", currentPos);

            //calculate desired joint angles
            RobotArmMath robotArm = new RobotArmMath();
            armAngles = robotArm.InverseKinematics(currentPos, theta3);
            telemetry.addData("lastArmAngles", new ArmAngles(lastTheta0, lastTheta1, lastTheta2, lastTheta3));
            telemetry.addData("requestedArmAngles", armAngles);

            //run base, shoulder, elbow and wrist motors using encoders to achieve angles calculated by IK
            RunArmMotors(armAngles);

            return;
        }

        if (robotState==STATE.DRIVING){
             if (moveArmRequested) {
                 telemetry.addData("DRIVING=>MOVINGARM", "complete");
                 robotState = STATE.MOVINGARM;
                 return;
             }

            // left stick controls direction
            // right stick X controls rotation
            float drivingScaleFactor = (float)0.5;

            float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

            // clip the right/left values so that the values never exceed +/- 1
            FrontRight = drivingScaleFactor * Range.clip(FrontRight, -1, 1);
            FrontLeft = drivingScaleFactor * Range.clip(FrontLeft, -1, 1);
            BackLeft = drivingScaleFactor * Range.clip(BackLeft, -1, 1);
            BackRight = drivingScaleFactor * Range.clip(BackRight, -1, 1);

            // write the values to the motors
            motorFrontRight.setPower(FrontRight);
            motorFrontLeft.setPower(FrontLeft);
            motorBackLeft.setPower(BackLeft);
            motorBackRight.setPower(BackRight);

		/*
		 * Telemetry for debugging

            telemetry.addData("Text", "*** Robot Data***");
            telemetry.addData("Joy XL YL XR",  String.format("%.2f", gamepad1LeftX) + " " +
                    String.format("%.2f", gamepad1LeftY) + " " +  String.format("%.2f", gamepad1RightX));
            telemetry.addData("f left pwr",  "front left  pwr: " + String.format("%.2f", FrontLeft));
            telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FrontRight));
            telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BackRight));
            telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BackLeft));
            return;
        */
        }

        if (robotState == STATE.SLOWDRIVING){
            if (moveArmRequested) {
                telemetry.addData("DRIVING=>MOVINGARM", "complete");
                robotState = STATE.MOVINGARM;
                return;
            }

            // left stick controls direction
            // right stick X controls rotation

            float slowDrivingScaleFactor = (float)0.25;

            float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

            // clip the right/left values so that the values never exceed +/- 1
            FrontRight = slowDrivingScaleFactor * Range.clip(FrontRight, -1, 1);
            FrontLeft = slowDrivingScaleFactor * Range.clip(FrontLeft, -1, 1);
            BackLeft = slowDrivingScaleFactor * Range.clip(BackLeft, -1, 1);
            BackRight = slowDrivingScaleFactor * Range.clip(BackRight, -1, 1);

            // write the values to the motors
            motorFrontRight.setPower(FrontRight);
            motorFrontLeft.setPower(FrontLeft);
            motorBackLeft.setPower(BackLeft);
            motorBackRight.setPower(BackRight);

		/*
		 * Telemetry for debugging

            telemetry.addData("Text", "*** Robot Data***");
            telemetry.addData("Joy XL YL XR",  String.format("%.2f", gamepad1LeftX) + " " +
                    String.format("%.2f", gamepad1LeftY) + " " +  String.format("%.2f", gamepad1RightX));
            telemetry.addData("f left pwr",  "front left  pwr: " + String.format("%.2f", FrontLeft));
            telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FrontRight));
            telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BackRight));
            telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BackLeft));

            return;
            */
        }
    }

    private void RunArmMotors(ArmAngles armAngles) {     //four angles passed in

        // swing base to theta0 controlled by gyro
        //calculate required base turn
        newTheta0 = armAngles.getTheta0();
        double requiredTurn = newTheta0 - lastTheta0;
        //command base servo to turn required distance (robot cannot be turning while the base servo is turning)
        if (turnBaseServo(requiredTurn)) {
            lastTheta0 = newTheta0;
        }
        else {
            telemetry.addData("BaseServoTurn FAILED", requiredTurn);
        }
        // raise/lower shoulder to theta1 controlled by encoder
        // double shoulderTurn = newTheta1 - lastTheta1;

        newTheta1 = armAngles.getTheta1();
        if (moveShoulderMotorToAngle(90+newTheta1)) {
            lastTheta1 = newTheta1;
        }
        else {
            telemetry.addData("shoulderTurn FAILED", 90+newTheta1);
        }
        /*
        if (turnShoulderMotor(shoulderTurn)) {
            lastTheta1 = newTheta1;
        }
        else {
            telemetry.addData("shoulderTurn FAILED", shoulderTurn);
        }
        */

        // extend/contract elbow to theta2 controlled by encoder
        // double elbowTurn = newTheta2 - lastTheta2;

        newTheta2 = armAngles.getTheta2();
        if (moveElbowMotorToAngle(newTheta2 - 180 - newTheta1)) {
            lastTheta2 = newTheta2;
        }
        else {
            telemetry.addData("elbowTurn FAILED", newTheta2 - 180 - newTheta1);
        }
        /*
        if (turnElbowMotor(elbowTurn - shoulderTurn)){
            lastTheta2 = newTheta2;
        }
        else {
            telemetry.addData("elbowTurn FAILED", elbowTurn - shoulderTurn);
        }
        */


        /*
        // flex wrist to theta3 controlled by pitch servo
        wristPitch.setPosition(armAngles.getTheta3() / 200);
        //newTheta3 = armAngles.getTheta3();
        double wristPitchTurn = elbowTurn; //TODO command to change from horizontal
        turnWristPitchServo (elbowTurn);
        */

        return;
    }

    private boolean turnBaseServo(double requiredTurn) {
        // telemetry.addData("requiredTurn:", requiredTurn);

        double currentHeading = gyroSensor.getHeading();
        // telemetry.addData("baseCurrentHeading:", currentHeading);

        targetHeading = (int) Math.round(requiredTurn +  currentHeading);
        // telemetry.addData("baseTarget:", targetHeading);
        // telemetry.update();

        if ((baseBusy()) || (Math.abs(requiredTurn) < NAVTHRESHOLD))
            return false; // we didn't start to turn the motor

        else if (requiredTurn > 0) {
            baseServo.setDirection(DcMotorSimple.Direction.FORWARD);
            if (targetHeading < 360) {
                //do nothing
            }
            else if (targetHeading >= 360) {
                targetHeading = targetHeading - 360;
            }
            while (Math.abs(gyroSensor.getHeading() - targetHeading) > NAVTHRESHOLD) {
                baseServo.setPower(BASESERVOSLOWSPEED);
                telemetry.addData("in loop baseCurrentHeading",  gyroSensor.getHeading());
                telemetry.addData("in loop baseTarget", targetHeading);
                telemetry.update();
            }
        }
        else {
            baseServo.setDirection(DcMotorSimple.Direction.REVERSE);
            if (targetHeading >= 0) {
                //do nothing
            } else if (targetHeading < 0) {
                targetHeading = targetHeading + 360;
            }
            while (Math.abs(gyroSensor.getHeading() - targetHeading) > NAVTHRESHOLD) {
                baseServo.setPower(BASESERVOSLOWSPEED);
                /*
                telemetry.addData("in loop baseCurrentHeading",  gyroSensor.getHeading());
                telemetry.addData("in loop baseTarget", targetHeading);
                telemetry.update();
                */
            }
        }
        baseServo.setPower(0.0);  //STOP TURNING!!
        return true;
    }

    private boolean turnShoulderMotor(double turnDegrees){
        double dTurnTicks = turnDegrees*TICKS/360;
        long lTurnTicks = Math.round(dTurnTicks);
        int iTurnTicks = (int) lTurnTicks;

        if (shoulder.isBusy() || (iTurnTicks==0))
            return false; // we didn't start to turn the motor
        else {
            int currentShoulderMotorTicks = shoulder.getCurrentPosition();
            shoulder.setTargetPosition(currentShoulderMotorTicks + iTurnTicks);
            telemetry.addData("CurrShoulder:" + Integer.toString(currentShoulderMotorTicks),
                    // " dTicks:" + Double.toString(dTurnTicks) + " lTicks:" + Long.toString(lTurnTicks) +
                    " iTurnTicks:" + Integer.toString(iTurnTicks));
            return true; // we started to turn the motor
        }
    }

    private boolean moveShoulderMotorToAngle(double desiredShoulderAngle){

        // this angle should be 90+theta1
        // 0 ticks is PACKEDTHETA1+90
        int desiredShoulderTicks = (int) Math.round((desiredShoulderAngle-(PACKEDTHETA1+90))*TICKS/360);

        if (shoulder.isBusy())
            return false;
        else {
            shoulder.setTargetPosition(desiredShoulderTicks);
            return true;
        }
    }

    private boolean turnElbowMotor(double turnDegrees){
        double dTurnTicks = turnDegrees*TICKS/360;
        long lTurnTicks = Math.round(dTurnTicks);
        int iTurnTicks = (int) lTurnTicks;

        if (elbow.isBusy() || (iTurnTicks==0))
            return false; // we didn't start to turn the motor
        else {
            int currentElbowMotorTicks = elbow.getCurrentPosition();
            elbow.setTargetPosition(currentElbowMotorTicks + iTurnTicks);
            telemetry.addData("CurrElbow:" + Integer.toString(currentElbowMotorTicks),
                    // " dTicks:" + Double.toString(dTurnTicks) + " lTicks:" + Long.toString(lTurnTicks) +
                    " iTicks:" + Integer.toString(iTurnTicks));
            return true;
        }
    }

    private boolean moveElbowMotorToAngle(double desiredElbowAngle){

        //this angle should be theta2-180-theta1
        // 0 ticks is PACKEDTHETA2-180-PACKEDTHETA1
        int desiredElbowTicks = (int) Math.round((desiredElbowAngle-(PACKEDTHETA2-180-PACKEDTHETA1))*TICKS/360);

        if (elbow.isBusy())
            return false;
        else {
            elbow.setTargetPosition(desiredElbowTicks);
            return true;
        }
    }

    private void turnWristPitchServo(double wristPitchAngleChange){
        double currentWristPitch = wristPitch.getPosition();

       // telemetry.addData("currentWristPitch", currentWristPitch);
        slowMove(wristPitch,(currentWristPitch + wristPitchAngleChange/180));

    }

    private void turnWristYawServo(double wristYawAngleChange) {
        double currentWristYaw = wristYaw.getPosition();

        slowMove(wristYaw, currentWristYaw + wristYawAngleChange/180);
    }

    private void turnWristOpenCloseServo(double wristOpenCloseAngleChange) {
        double currentWristOpenClose = wristOpenClose.getPosition();

        slowMove(wristOpenClose, currentWristOpenClose + wristOpenCloseAngleChange/180);
    }

    private void unfurlArm(){
        turnShoulderMotor(UNFURLEDTHETA1 - PACKEDTHETA1);
        turnElbowMotor(((UNFURLEDTHETA2 - PACKEDTHETA2) - (UNFURLEDTHETA1 - PACKEDTHETA1)));

        return;
    }

    private void rePackArm(){
        turnBaseServo(PACKEDTHETA0 - lastTheta0);
        turnShoulderMotor(PACKEDTHETA1 - lastTheta1);
        turnElbowMotor(PACKEDTHETA2 - lastTheta2);

        //packed!!  update lastTheta values and send info to DS
        lastTheta1 = PACKEDTHETA1;
        lastTheta2 = PACKEDTHETA2;
        telemetry.addData("rePack Complete. Theta1=", Double.toString(lastTheta1)
                + " Theta2=" + Double.toString(lastTheta2));
        return;
    }
    public boolean wristPitchBusy(double targetPosition){
        boolean moving;
        if(wristPitch.getPosition() == targetPosition){
            moving = false;
        }
        else{
            moving = true;
        }
        return moving;
    }

    public boolean wristYawBusy(double targetPosition) {
        boolean moving;
        if (wristYaw.getPosition() == targetPosition) {
            moving = false;
        } else {
            moving = true;
        }
        return moving;
    }

    public boolean wristOpenCloseBusy(double targetPosition) {
        boolean moving;
        if (wristOpenClose.getPosition() == targetPosition) {
            moving = false;
        } else {
            moving = true;
        }
        return moving;
    }
    public boolean baseBusy() {
        boolean moving;
        if (baseServo.getPower() == 0.0) {
            moving = false;
        }
        else {
            moving = true;
        }
        return moving;
    }

    public void slowMove(Servo wristServo, double targetPosition){

        double distance = targetPosition - wristServo.getPosition();
        double increment = distance/1000;
        // telemetry.addData("servo target position", targetPosition);
        // telemetry.addData("servo increment", increment);
        while (Math.abs(wristServo.getPosition() - targetPosition) >= 0.01){
            double wristPosition = wristServo.getPosition();
            //telemetry.addData("servo current position", wristPosition);
            wristServo.setPosition(distance/1000 + wristServo.getPosition());

            long sleepTime = 1000;
            long wakeupTime = System.currentTimeMillis() + sleepTime;
/*
            while (sleepTime > 0)
            {
                try
                {
                    wristServo.wait(sleepTime);
                }
                catch (InterruptedException e)
                {
                    sleepTime = wakeupTime - System.currentTimeMillis();
                }
            }
*/

        }
    }
}