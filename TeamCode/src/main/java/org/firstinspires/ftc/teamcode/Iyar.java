package org.firstinspires.ftc.teamcode;

public class Iyar {

    double lastPos = 0;
    double lastTime = 0;

    public double getVelocity(double currentPos, double currentTime){
        double deltaPos = 0;
        double deltaTime = 0;
        double velocity = 0;

        deltaPos = currentPos - lastPos;
        lastPos = currentPos;
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        velocity = currentPos / currentTime;

        return velocity;
    }
}