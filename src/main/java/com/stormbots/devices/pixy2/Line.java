package com.stormbots.devices.pixy2;

/**
 * Represent a Line from the getMainFeautures() call
 * When handling raw resolutions, note the camera resolution is 79 x 52
 * in mainFeautures mode.
 */
public class Line{
    public double x0=0,y0=0,x1=0,y1=0;
    //TODO : Possibly determine out a good way to verify the resolution without soaking time in critical loop
    //Possibly static class variables?

    public Line(){}
    public Line(byte x0,byte x1,byte y0,byte y1){
        this.x0 = x0;
        this.y0 = y0;
        this.x1 = x1;
        this.y1 = y1;
    }

    public String toString(){
        return String.format("Line (%.2f,%.2f)->(%.2f,%.2f)",x0,y0,x1,y1);
    }

    /** Convert coordinates to [-0.1..0.1], with the bottom center of the screen as (0,0) */
    public Line normalizeBottom(){
        x0 = 2*(x0/79.0-0.5);
        y0 = 2*(1-y0/52.0);
        x1 = 2*(x1/79.0-0.5);
        y1 = 2*(1-y1/52.0);
        return this;
    }
    
    /** Convert coordinates to [-0.1..0.1], with the center of the screen as (0,0)  */
    public Line normalizeCenter(){
        x0 = 2*(x0/79.0-0.5);
        y0 = 2*(0.5-y0/52.0);
        x1 = 2*(x1/79.0-0.5);
        y1 = 2*(0.5-y1/52.0);
        return this;
    }

}    