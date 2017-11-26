package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by liza on 11/12/17.
 */

@TeleOp(name = "Concept: Position", group = "Concept")
//@Disabled

public class Position {
    double X, Y, Z;
    final double QUANTA = 0.05;

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

    public Position calcNewPosition(boolean zright, boolean zleft, boolean xforward, boolean xback, double yupdown) {

        double gamepadX, gamepadY, gamepadZ;

        if (zright)
            gamepadZ = 1 * QUANTA;
        else if (zleft)
            gamepadZ = -1 * QUANTA;
        else gamepadZ = 0;

        if (xforward)
            gamepadX = 1 * QUANTA;
        else if (xback)
            gamepadX = -1 * QUANTA;
        else gamepadX = 0;

        gamepadY = yupdown * QUANTA;

        return new Position(X+gamepadX, Y+gamepadY, Z+gamepadZ);
    }

    public String toString()
    {
        return "Rounded Pos: X:" + Math.round(X) + " Y:" + Math.round(Y) + " Z:" + Math.round(Z);
    }
}

