package org.usfirst.frc.team365.robot;

import java.io.File;
import java.io.FileOutputStream;
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
			file.delete();
			file.createNewFile();
			FileOutputStream fos = new FileOutputStream(file);
			String text=""+timestamp();
			text+="AutoChoice: "+autoChoice+"\n";
			text+="ShooterAngle: "+shooterAngle+"\n";
			text+="ArmAngle: "+armAngle+"\n";
			text+="PowerA: "+powerA+"\n";
			text+="PowerB: "+powerB+"\n";
			text+="Yaw: "+yaw+"\n";
			text+="Pitch: "+pitch+"\n";
			text+="Roll: "+roll+"\n";
			fos.write(text.getBytes());
			fos.close();
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
