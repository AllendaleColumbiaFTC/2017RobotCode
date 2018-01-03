package org.firstinspires.ftc.teamcode;


// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by liza on 11/12/17.
 */

// @TeleOp(name = "Concept: Position", group = "Concept")
// @Disabled

public class Position {
    double X, Y, Z;
    final double STEP = 1.0;

    public Position() {
        X = 0;  //Forward
        Y = 0;  //UP!!!
        Z = 0;
    }

    public Position(double x, double y, double z) {
        X = x;  //Forward
        Y = y;  //UP!!!
        Z = z;
    }

    public void updatePosition(boolean zright, boolean zleft, boolean xforward, boolean xback, double yupdown) {

        double gamepadX, gamepadY, gamepadZ;

        if (zright)  //REVERSED 12/27 because gyro increasing counter clockwise!
            gamepadZ = -1 * STEP;
        else if (zleft)
            gamepadZ = 1 * STEP;
        else gamepadZ = 0;

        if (xforward)
            gamepadX = 1 * STEP;
        else if (xback)
            gamepadX = -1 * STEP;
        else gamepadX = 0;

        gamepadY = yupdown * STEP * -0.1;   //reversed because forward on gamepad is -1!
                                            //set to 0.1 because stick inputs seem to refresh quickly

        X = X+gamepadX;
        Y = Y+gamepadY;
        Z = Z+gamepadZ;
    }

    public String toString() {  //show positions rounded to 3 decimal places
        return "Pos: X:" + (double)Math.round(X*1000)/1000 + " Y:" + (double)Math.round(Y*1000)/1000 +
                " Z:" + (double)Math.round(Z*1000)/1000;
    }

}

