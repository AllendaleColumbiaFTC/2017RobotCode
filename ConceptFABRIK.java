package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import au.edu.federation.utils.Utils;



/**
 * Created by Liza on 10/22/17.
 */
@TeleOp(name = "Concept: FABRIK", group = "Concept")
//@Disabled

public class ConceptFABRIK extends OpMode {

    DcMotor shoulder;
    DcMotor elbow;
    float last_x, last_y, last_z;   // previous (x,y,z) position
    float new_x, new_y, new_z;      // requested next (x,y,z) position
    final float QUANTA= 0.1f;       // maximum distance away from current position in each dimension

    public ConceptFABRIK () {
        //Constructor
    }

   @Override
    public void init(){

       shoulder = hardwareMap.dcMotor.get("shoulder");
       elbow = hardwareMap.dcMotor.get("elbow");
       last_x=0;
       last_y=0;
       last_z=0;



   }

   @Override
    public void loop(){

       //get driver inputs



       //determine new position
       new_x = last_x;
       if(gamepad2.dpad_right)
           new_x += QUANTA;
       if(gamepad2.dpad_left)
           new_x -= QUANTA;

       new_y = last_y;
       if(gamepad2.dpad_up)
           new_y += QUANTA;
       if(gamepad2.dpad_down)
           new_y -= QUANTA;

       new_z = last_z + gamepad2.right_stick_y * QUANTA;

       //pass desired poisition to FABRIK




       //run shoulder and elbow motors using encoders to achieve angles calculated by FABRIK
   }
}
