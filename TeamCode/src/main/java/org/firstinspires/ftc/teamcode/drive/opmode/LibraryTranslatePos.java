package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LibraryTranslatePos {
    int startXPos = 84;
    int startYPos = 12;
    Telemetry telemetry;
    public LibraryTranslatePos(Telemetry newtelemetry){
        telemetry = newtelemetry;
    }

    public double TranslatingX(double newXPos) {
        double newX = (newXPos*24) + startXPos;
//        int newX = 22;
        telemetry.addData("Translating xPos",newX);
        telemetry.update();
        return newX;

    }public double TranslatingY(double newYPos) {
        double newY = (newYPos*24) + startYPos;
//        int newY = newYPos-22;
        telemetry.addData("Translating yPos",newY);
        telemetry.update();
        return newY;
    }
}


