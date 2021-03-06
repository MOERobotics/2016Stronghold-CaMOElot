package org.usfirst.frc.team365.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoFile
{
	final static String PATH="/home/lvuser/AutoChoice.txt";
	File file=new File(PATH);
	static boolean open=true;
	
	
	public void fileConfirm() throws IOException
	{
		file.mkdirs();
		if(file.exists())
		{
			file.delete();
		}
		file.createNewFile();
	}
	public void writeAutoFile(String choice)
	{
		try
		{
			if(open)
			{
				open=false;
				fileConfirm();
				FileOutputStream fos = new FileOutputStream(file);
				fos.write(choice.getBytes());
				fos.close();
				open=true;
			}	
		}
		catch(Exception e)
		{
			e.printStackTrace();
			System.out.println("Failed File Write");
		}
	}
	public int readAutoFile()
	{
		try
		{
			FileReader fr=new FileReader(file);
			BufferedReader br=new BufferedReader(fr);
			String str=br.readLine();
			int choice = Integer.parseInt(str);
			br.close();
			return choice;
		}
		catch (Exception e)
		{
			System.out.println("Failed File Read, default to routine do-nothing");
			return 0;
		}
	}
}
