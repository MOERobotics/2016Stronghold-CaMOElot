package org.usfirst.frc.team365.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

public class AutoData
{
	public double
	shooterAngle,
	armAngle,
	powerA,
	powerB,
	yaw,
	roll,
	pitch;
	private int autoChoice;
	public AutoData(int autoChoice)
	{
		this.autoChoice=autoChoice;
	}
	public boolean save()
	{
		try
		{
			File file=new File("/home/lvuser/AutoData.txt");
			file.delete();file.createNewFile();
			FileWriter fw=new FileWriter(file);
			BufferedWriter bw=new BufferedWriter(fw);
			bw.write("AutoChoice: "+autoChoice);bw.newLine();
			bw.write("ShooterAngle: "+shooterAngle);bw.newLine();
			bw.write("ArmAngle: "+armAngle);bw.newLine();
			bw.write("PowerA: "+powerA);bw.newLine();
			bw.write("PowerB: "+powerB);bw.newLine();
			bw.write("Yaw: "+yaw);bw.newLine();
			bw.write("Pitch: "+pitch);bw.newLine();
			bw.write("Roll: "+roll);
			return true;
		}
		catch(Exception e)
		{
			return false;
		}
	}
}
