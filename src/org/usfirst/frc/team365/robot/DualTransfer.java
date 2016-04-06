package org.usfirst.frc.team365.robot;

public class DualTransfer<T1,T2>
{
	private T1 arg1;
	private T2 arg2;
	public DualTransfer(T1 arg1, T2 arg2)
	{
		this.arg1=arg1;
		this.arg2=arg2;
	}
	public void setArg1(T1 val){arg1=val;}
	public void setArg2(T2 val){arg2=val;}
	public T1 getArg1(){return arg1;}
	public T2 getArg2(){return arg2;}
}
