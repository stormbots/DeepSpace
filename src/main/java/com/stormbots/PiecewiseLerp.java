
package com.stormbots;


public class PiecewiseLerp{

    double[] toList;
    double[] fromList;

    public PiecewiseLerp(double[] toList, double[] fromList){
        this.toList = toList;
        this.fromList = fromList;
    }

    public double getOutputAt(double input) {
        if(fromList.length != toList.length){
    		System.err.println("Number of elements in the list does not match!!!");
    		return  toList[0];
        }
        for (int i=0; i<fromList.length-1;i++){
            if(fromList[i+1]>input){
                return Lerp.lerp(input, fromList[i], fromList[i+1],toList[i], toList[i+1]);
            }
        }
        return toList[fromList.length-1];
    }
    




}












