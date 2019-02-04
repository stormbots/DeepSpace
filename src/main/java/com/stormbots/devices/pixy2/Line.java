package com.stormbots.devices.pixy2;

public class Line{
    public byte featureLength = 0;
    public byte featureData = 0; //not sure what this does everything "varies"

    public Line(){}

    public String toString(){
        return "Line " + " " + featureLength;
    }
}