package org.usfirst.frc.team365.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.Date;

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
	public static boolean safeSave(AutoData ad)
	{
		if(ad!=null)
			return ad.save();
		else return false;
	}
	public boolean save()
	{
		try
		{
			File file=new File("/home/lvuser/AutoData.txt");
			file.delete();file.createNewFile();
			FileWriter fw=new FileWriter(file);
			BufferedWriter bw=new BufferedWriter(fw);
			String text=""+timestamp();
			text+="AutoChoice: "+autoChoice+"\n";
			text+="ShooterAngle: "+shooterAngle+"\n";
			text+="ArmAngle: "+armAngle+"\n";
			text+="PowerA: "+powerA+"\n";
			text+="PowerB: "+powerB+"\n";
			text+="Yaw: "+yaw+"\n";
			text+="Pitch: "+pitch+"\n";
			text+="Roll: "+roll+"\n";
			bw.write(text);
			bw.close();
			return true;
		}
		catch(Exception e)
		{
			return false;
		}
	}
	public static String timestamp()
	{
		try{
			Date d=new Date(System.currentTimeMillis());
			return d.toString()+"\n";
		}catch(Exception e){
			return "DFLT TimeStamp\n";
		}
	}
}
