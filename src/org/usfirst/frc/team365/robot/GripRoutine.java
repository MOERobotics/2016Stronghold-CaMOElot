package org.usfirst.frc.team365.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;

public class GripRoutine
{
	static final int HEIGHT=360;
	static final int WIDTH=480;
	int xOff,yOff;
	int targetX,targetY;
	NetworkTable table;
	public GripRoutine(int xOff, int yOff)
	{
		this.xOff=xOff;
		this.yOff=yOff;
		targetX=WIDTH/2+xOff;
		targetY=HEIGHT/2+yOff;
		table=NetworkTable.getTable("GRIP");
	}
	public DualTransfer<Double, Double> analyze()
	{
		DualTransfer<Double, Double>send=new DualTransfer<>(0.0,0.0);
		ITable it=table.getSubTable("myContoursReport");
		double[]areas=it.getNumberArray("area", new double[]{-1});
		double[]xVals=it.getNumberArray("centerX",new double[]{});
		double[]yVals=it.getNumberArray("centerY",new double[]{});
		double pastArea=-1;
		int index=-1;
		for(int n=0;n<areas.length;n++)
		{
			if(pastArea<areas[n])
			{
				pastArea=areas[n];
				index=n;
			}
		}
		if(pastArea>0)
		{
			//proportional differences in x&y from -1 to 1
			double dx=(xVals[index]-targetX)/WIDTH;	
			double dy=(yVals[index]-targetY)/HEIGHT;
			send.setArg1(dx);
			send.setArg2(dy);
		}
		return send;
		
	}
}
