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

package org.firstinspires.ftc.teamcode;

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



public class RobotArmMath  {
    private double l1 = 18.125;  //length in inches of the first arm from joint to joint
    private double l2 = 15;  //length in inches of the first arm from joint to joint
    private double l3 = 3;  //legth in inches of the end effector
    private double theta0, theta1, theta2, theta3; //theta0 = the base angle, theta1 = the angle between the second arm and the base, theta2 = angle between the second arm and the horizontal, theta3 = the angle between the end effector and the horizontal
    private double h = 16.06; //height in inches of the first revolute joint above the base


    private double c1(double theta3, double x0, double theta0) {
        double c1;
        c1 = 2 * l1 * l3 * Math.cos(theta3) - 2 * x0 * l1 / Math.cos(theta0);
        return c1;
    }
    private double c2(double theta3, double y0) {
        double c2;
        c2 = 2 * h * l1 + 2 * l1 * l3 * Math.sin(theta3) - 2 * y0 * l1;
        return c2;
    }
    private double c3(double x0, double theta0, double y0, double theta3){
        double c3;
        c3 = Math.pow(x0,2)/((Math.cos(theta0)* Math.cos(theta0)+Math.pow(y0,2)+ Math.pow(h,2) + Math.pow(l1,2) + Math.pow(l3,2) - Math.pow(l2,2) - 2x0*l3*Math.cos(theta3)/Math.cos(theta0)-2y0*h+2l3*(Math.sin(theta3)*(h-y0);
        return c3;
    }
    private double c4(double theta3, double x0, double theta0) {
        double c4;
        c4 = 2 * l2 * l3 * Math.cos(theta3) - 2 * x0 * l2 / Math.cos(theta0);
        return c4;
    }
    private double c5(double theta3, double y0){
        double c5;
        c5 = 2*h*l2+2*l2*l3*Math.sin(theta3) - 2*y0*l2);
        return c5;
    }
    private double c6(double x0, double theta0, double y0, double theta3) {
        double c6;
        c6 = ((Math.pow(x0,2)) / (Math.cos(theta0) * Math.cos(theta0)) + Math.pow(y0,2) + Math.pow(h,2) + Math.pow(l2,2) + Math.pow(l3,2) - Math.pow(l1,2) - 2 * x0 * l3 * Math.cos(theta3) / Math.cos(theta0) - 2 * y0 * h + 2 * l3 * Math.sin(theta3) * (h - y0);
    }

    /* Equation 22:
    private double eq22{
        double theta2;
        theta2= 2 * Math.atan(c5 + sqrt(c5^2 - c6^2 + c4^2)/(c4-c6);
        }
        */

    public double[] InverseKinematics (Position desiredPosition, double theta3) {
        double[] ArmAngles=null;

        double theta0, theta1, theta1plus, theta1minus, theta2, theta2plus, theta2minus;

        theta0 = Math.atan(desiredPosition.Z/desiredPosition.X);

        double c1, c2, c3, c4, c5, c6;
        c1=c1(theta3, desiredPosition.X, theta0);
        c2=c2(theta3, desiredPosition.Y);
        c3=c3(desiredPosition.X, theta0, desiredPosition.Y, theta3);
        c4=c4(theta3, desiredPosition.X, theta0);
        c5=c5(theta3, desiredPosition.Y);
        c6=c6(desiredPosition.X, theta0, desiredPosition.Y, theta3);

        theta1plus = 2*Math.atan2((c2+Math.sqrt(Math.pow(c2,2)-Math.pow(c3,2)+Math.pow(c1,2))),c1-c3);
        theta1minus = 2*Math.atan2((c2-Math.sqrt(Math.pow(c2,2)-Math.pow(c3,2)+Math.pow(c1,2))),c1-c3);
        if (theta1plus>theta1minus)  //Select the biggest angle for theta1
            theta1 = theta1plus;
        else
            theta1 = theta1minus;

        theta2plus = 2*Math.atan2((c5+Math.sqrt(Math.pow(c5,2)-Math.pow(c6,2)+Math.pow(c4,2))),c4-c6);
        theta2minus = 2*Math.atan2((c5-Math.sqrt(Math.pow(c5,2)-Math.pow(c6,2)+Math.pow(c4,2))),c4-c6);
        if (theta2plus>theta2minus)  //Select the biggest angle for theta2
            theta2 = theta2plus;
        else
            theta2 = theta2minus;

        ArmAngles[0]=theta0;
        ArmAngles[1]=theta1;
        ArmAngles[2]=theta2;

        return ArmAngles;
    }

}
