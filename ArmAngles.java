package org.firstinspires.ftc.teamcode;

/**
 * Created by liza on 11/26/17.
 */


public class ArmAngles {
    double theta0, theta1, theta2, theta3;

    ArmAngles(){
            theta0=0;
            theta1=0;
            theta2=0;
            theta3=0;
        }

        ArmAngles(double newTheta0, double newTheta1, double newTheta2, double newTheta3)
        {
            theta0 = newTheta0;
            theta1 = newTheta1;
            theta2 = newTheta2;
            theta3 = newTheta3;
        }

        void setTheta0 (double newTheta0){
            theta0 = newTheta0;
        }
        void setTheta1 (double newTheta1){
            theta1 = newTheta1;
        }
        void setTheta2 (double newTheta2){
            theta2 = newTheta2;
        }
        void setTheta3 (double newTheta3){
            theta3 = newTheta3;
        }
        double getTheta0(){
            return theta0;
        }
        double getTheta1(){
            return theta1;
        }
        double getTheta2(){
            return theta2;
        }
        double getTheta3() {
            return theta3;
        }

        public String toString() {
            return "Rounded Degrees theta0:" + Math.round(theta0*180/Math.PI) + " t1" + Math.round(theta1*180/Math.PI) +
                    " t2:" + Math.round(theta2*180/Math.PI) + " t3:" + Math.round(theta3*180/Math.PI);
        }

    }
