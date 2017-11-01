/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="RobotArmMath")
@Disabled
public class RobotArmMath extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.
    double
    @Override
    public void runOpMode() {
        double armPower;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("PlaceHolder");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        }

    }
    private double c1(double l1, double l3, double theta3, double x0, double theta0) {
        double c1;
        c1 = 2 * l1 * l3 * Math.cos(theta3) - 2 * x0 * l1 / Math.cos(theta0);
        return c1;
    }
    private double c2(double h, double l1,double l3, double theta3, double y0) {
        double c2;
        c2 = 2 * h * l1 + 2 * l1 * l3 * Math.sin(theta3) - 2 * y0 * l1;
        return c2;
    }
    private double c3(double x0, double theta0, double y0, double h, double l1, double l3, double l1, double l2, double theta3){
        double c3;
        c3 = Math.pow(x0,2)/((Math.cos(theta0)* Math.cos(theta0)+Math.pow(y0,2)+ Math.pow(h,2) + l1^2+ l3^2 -l2^2 - 2x0*l3*Math.cos(theta3)/Math.cos(theta0)-2y0*h+2l3*(Math.sin(theta3)*(h-y0);
        return c3;
    }
    private double c4(double l2, double l3, double theta3, double x0, double theta0) {
        double c4;
        c4 = 2 * l2 * l3 * Math.cos(theta3) - 2 * x0 * l2 / Math.cos(theta0);
        return c4;
    }
    private double c5(double c5, double h, double l2, double l3, double theta3, double y0){
        double c5;
        c5 = 2*h*l2+2*l2*l3*Math.sin(theta3) - 2*y0*l2);
        return c5;
    }
    private double c6(double x0, double theta0, double y0, double h, double l2, double l3, double l1, double theta3) {
        double c6;
        c6 = x0 ^ 2 / (Math.cos(theta0) * Math.cos(theta0)) + y0 ^ 2 + h ^ 2 + l2 ^ 2 + l3 ^ 2 - l1 ^ 2 - 2 * x0 * l3 * Math.cos(theta3) / Math.cos(theta0) - 2 * y0 * h + 2 * l3 * Math.sin(theta3) * (h - y0);
    }

    /* Equation 22:
    private double eq22{
        double theta2;
        theta2= 2 * Math.atan(c5 + sqrt(c5^2 - c6^2 + c4^2)/(c4-c6);
        }
        */

}
