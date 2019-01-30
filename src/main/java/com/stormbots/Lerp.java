package com.stormbots;

/**
 * Functions for handling linear interpolation
 */
public class Lerp {

	/**
	 * Arduino Map function. <br>
	 * Prefer the {@link #lerp()} function which does the same thing, but is not improperly 
	 * named.
	 */
	@Deprecated
	static double map(double x, double in_min, double in_max, double out_min, double out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	/**
	 * Convert one linear data range into another.
	 */
	static double lerp(double x, double in_min, double in_max, double out_min, double out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
}
