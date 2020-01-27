/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;
import java.util.*;

/**
 */

public class Average {
	private int n;
	private ArrayList<Double> a;
	private int index = 0;
	private int numberSamples;
	public Average(int n) {
		this.n = n;
		a = new ArrayList<>(n);
	}
	
	public void add(double val) {
		
		if(n>numberSamples) {
			numberSamples++;
			a.add(index, val);
		} else {
			a.set(index,val);
		}
		index = (index +1) % n;
	}
	public double averaged() {
		double sum = 0;
		for(int i = 0; i< numberSamples;i++) {
			sum += a.get(i);
		}
		sum/=numberSamples;
		return sum;
	}
}

