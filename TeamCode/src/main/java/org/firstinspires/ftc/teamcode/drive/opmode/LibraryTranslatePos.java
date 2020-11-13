package org.firstinspires.ftc.teamcode.drive.opmode;

public class LibraryTranslatePos {
    int startXPos = -60;
    int startYPos = -60;

    public double TranslatingX(int newXPos) {
        int Xt;

        Xt = newXPos-startXPos;

        return Xt;

    }public double TranslatingY(int newYPos) {
        int Yt;
        Yt = newYPos-startYPos;
        return Yt;
    }
}


