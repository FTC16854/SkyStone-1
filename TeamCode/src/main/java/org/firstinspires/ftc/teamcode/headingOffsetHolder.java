package org.firstinspires.ftc.teamcode;

public class headingOffsetHolder{
    private static double offset = 0;

    public static void setOffset(double heading){
        offset = heading+180;
    }

    public static double getOffset(){
        return offset;
    }
}
