package org.usfirst.frc.team365.robot;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class AutoFile
{
	final static String PATH="/home/lvuser/AutoChoice.txt";
	File file=new File(PATH);
	static boolean open;
	
	
	private int getChoice() throws Exception
	{
		String str=javax.swing.JOptionPane.showInputDialog("Write the Auto Choice");
		return Integer.parseInt(str);
	}
	public void writeAutoFile()
	{
		try
		{
			if(open)
			{
				open=false;
				BufferedWriter bw=new BufferedWriter(new FileWriter(file));
				bw.write(""+getChoice());
				bw.close();
				open=true;
			}
			else
				throw new Exception();
		}
		catch(Exception e)
		{
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
