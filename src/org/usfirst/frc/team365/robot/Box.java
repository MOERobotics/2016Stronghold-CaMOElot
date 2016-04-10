package org.usfirst.frc.team365.robot;

public class Box
{
	private double x,y,w,h;
	public Box(double x, double y, double w, double h)
	{
		this.x=x;
		this.y=y;
		this.w=w;
		this.h=h;
	}
	public double getX(){return x;}
	public double getY(){return y;}
	public double getW(){return w;}
	public double getH(){return h;}
}
