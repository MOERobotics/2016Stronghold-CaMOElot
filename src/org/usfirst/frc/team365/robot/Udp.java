package org.usfirst.frc.team365.robot;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

public class Udp implements Runnable
{
	static final int port=5801;
	static String lastData="";
	DatagramSocket socket;
	protected volatile boolean more=true;
	protected String name;
	public Udp(String name) throws SocketException
	{
		this.name=name;
		socket=new DatagramSocket(port);
	}
	public void run()
	{
		byte[]buf=new byte[256];
		DatagramPacket packet=new DatagramPacket(buf, buf.length);
		while(more)
		{
			try
			{
				packet.setLength(buf.length);
				socket.receive(packet);
				byte[]data=packet.getData();
				lastData=new String(data, 0, packet.getLength());
				System.out.println(lastData);
			}
			catch(Exception e)
			{
				e.printStackTrace();
				more = false;
			}
		}
		socket.close();
	}

}
