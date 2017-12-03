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

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.ArmAngles;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RobotArmMath {
    private double l1 = 18.125;  //length in inches of the first arm from joint to joint
    private double l2 = 15.25;  //length in inches of the first arm from joint to joint
    private double l3 = 11;  //legth in inches of the end effector
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

    private double c3(double x0, double theta0, double y0, double theta3) {
        double c3;
        c3 = Math.pow(x0, 2) / ((Math.cos(theta0) * Math.cos(theta0))) + Math.pow(y0, 2) + Math.pow(h, 2) + Math.pow(l1, 2) + Math.pow(l3, 2) - Math.pow(l2, 2) - (2 * (x0) * l3 * Math.cos(theta3)) / (Math.cos(theta0)) - 2 * (y0) * h + 2 * (l3) * (Math.sin(theta3) * (h - y0));
        return c3;
    }

    private double c4(double theta3, double x0, double theta0) {
        double c4;
        c4 = 2 * l2 * l3 * Math.cos(theta3) - 2 * x0 * l2 / Math.cos(theta0);
        return c4;
    }

    private double c5(double theta3, double y0) {
        double c5;
        c5 = 2 * h * l2 + 2 * l2 * l3 * Math.sin(theta3) - 2 * y0 * l2;
        return c5;
    }

    private double c6(double x0, double theta0, double y0, double theta3) {
        double c6;
        c6 = ((Math.pow(x0, 2)) / (Math.cos(theta0) * Math.cos(theta0)) + Math.pow(y0, 2) + Math.pow(h, 2) + Math.pow(l2, 2) + Math.pow(l3, 2) - Math.pow(l1, 2) - 2 * x0 * l3 * Math.cos(theta3) / Math.cos(theta0) - 2 * y0 * h + 2 * l3 * Math.sin(theta3) * (h - y0));
        return c6;
    }



    // Equation 22:
    private double eq22(double c4, double c5, double c6) {
        double theta2;
        theta2 = 2 * Math.atan(c5 + Math.sqrt(Math.pow(c5, 2) - Math.pow(c6, 2) + Math.pow(c4, 2)) / (c4 - c6));
        return theta2;
    }


    public ArmAngles InverseKinematics(Position desiredPosition, double theta3) {
        ArmAngles armAngles = new ArmAngles();

        double theta0, theta1, theta2;

        theta0 = Math.atan2(desiredPosition.Z, desiredPosition.X);

        double c1, c2, c3, c4, c5, c6;
        c1 = c1(theta3, desiredPosition.X, theta0);
        c2 = c2(theta3, desiredPosition.Y);
        c3 = c3(desiredPosition.X, theta0, desiredPosition.Y, theta3);
        c4 = c4(theta3, desiredPosition.X, theta0);
        c5 = c5(theta3, desiredPosition.Y);
        c6 = c6(desiredPosition.X, theta0, desiredPosition.Y, theta3);

        theta1 = 2 * Math.atan2((c2 - Math.sqrt(Math.pow(c2, 2) - Math.pow(c3, 2) + Math.pow(c1, 2))), c1 - c3);
        theta2 = 2 * Math.atan2((c5 + Math.sqrt(Math.pow(c5, 2) - Math.pow(c6, 2) + Math.pow(c4, 2))), c4 - c6);

        armAngles.setTheta0(theta0);
        armAngles.setTheta1(theta1);
        armAngles.setTheta2(theta2);

        return armAngles;
    }



    }








