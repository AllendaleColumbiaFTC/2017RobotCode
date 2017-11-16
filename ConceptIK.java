package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;



/**
 * Created by Liza on 10/22/17.
 */
@TeleOp(name = "Concept: IK", group = "Concept")
//@Disabled

public class ConceptIK extends OpMode {

    DcMotor shoulder;
    DcMotor elbow;
    double last_x, last_y, last_z, ;   // previous (x,y,z) position
    double new_x, new_y, new_z;      // requested next (x,y,z) position
    final double QUANTA= 0.1;       // maximum distance away from current position in each dimension
    Position currentPos;

    public ConceptIK() {
        //Constructor

    }

   @Override
    public void init() {

       shoulder = hardwareMap.dcMotor.get("shoulder");
       elbow = hardwareMap.dcMotor.get("elbow");
       last_x = 0;
       last_y = 0;
       last_z = 0;
       currentPos = new Position();


   }

   @Override
    public void loop(){
       double[] ArmAngles = null;

       //get driver inputs
       boolean xright = gamepad1.dpad_right;
       boolean xleft = gamepad1.dpad_left;
       boolean yup = gamepad1.dpad_up;
       boolean ydown = gamepad1.dpad_down;
       double z = gamepad1.right_stick_y;
       double theta3 = 0; //TODO: Update via gamepad2.x and gamepad2.y
                            //TODO: Keep track of  "oldTheta3" variable across calls of loop()

       //calculate desired newPosition
       currentPos = currentPos.calcNewPosition(xright, xleft, yup, ydown, z);

       //calculate desired joint angles
       ArmAngles = RobotArmMath.InverseKinematics(currentPos, theta3);

       //run shoulder and elbow motors using encoders to achieve angles calculated by IK
       RunArmMotors(ArmAngles)
   }
}
