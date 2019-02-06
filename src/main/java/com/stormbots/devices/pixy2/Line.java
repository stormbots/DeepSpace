package com.stormbots.devices.pixy2;

public class Line{
    public int x0=0,y0=0,x1=0,y1=0;
    public Line(){}
    public Line(byte x0,byte x1,byte y0,byte y1){
        this.x0 = x0;
        this.y0 = y0;
        this.x1 = x1;
        this.y1 = y1;
    }

    public String toString(){
        return String.format("Line (%d,%d)->(%d,%d)",x0,y0,x1,y1);
    }

}    