package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * Created by eileen on 11/12/17.
 */

@TeleOp(name = "Concept: Position", group = "Concept")
//@Disabled

public class Position {
    double X, Y, Z;
    final double QUANTA = 0.05;

    public Position() {
        X = 0;
        Y = 0;
        Z = 0;
    }

    public Position(double x, double y, double z) {
        X = x;
        Y = y;
        Z = z;

    }

    public Position calcNewPosition(boolean xright, boolean xleft, boolean yup, boolean ydown, double Z) {

        double gamepadX, gamepadY, gamepadZ;

        if (xright)
            gamepadX = 1 * QUANTA;
        else if (xleft)
            gamepadX = -1 * QUANTA;
        else gamepadX = 0;

        if (yup)
            gamepadY = 1 * QUANTA;
        else if (ydown)
            gamepadY = -1 * QUANTA;
        else gamepadY = 0;

        gamepadZ = Z * QUANTA;

        return new Position(gamepadX, gamepadY, gamepadZ);

    }
}

