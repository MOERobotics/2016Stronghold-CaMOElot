package org.usfirst.frc.team365.robot;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Udp implements Runnable
{
	static final short noneFound=1, oneFound=2, twoFound=3;
	static final int port=5801;
	static final double width=320,height=240;
	volatile int lastID;
	DatagramSocket socket;
	protected volatile boolean more=true;
	protected String name;
	protected Box[]boxes=new Box[2];
	public Udp(String name) throws SocketException
	{
		this.name=name;
		socket=new DatagramSocket(port);
	}
	public void run()
	{
		ByteBuffer buf=ByteBuffer.allocate(72);
		DatagramPacket packet=new DatagramPacket(buf.array(), buf.limit());
		while(more)
		{
			try
			{
				buf.position(0);
				packet.setLength(buf.limit());
				System.out.println("preRecieve");
				socket.receive(packet);
				System.out.print("postRecieve ");
				int id=buf.getInt();
				if(id<=lastID)
					continue;
				lastID=id;
				short status=buf.getShort();System.out.println(status);
				buf.getShort();
				boxes=new Box[2];
				// doubles as l,r,w,h
				switch(status)
				{
					case twoFound:
					{
						double x=buf.getDouble();
						double y=buf.getDouble();
						double w=buf.getDouble();
						double h=buf.getDouble();
						boxes[1]=new Box(width*(x+w/2),height*(y+h/2),w,h);
					}
					case oneFound:
					{
						double x=buf.getDouble();
						double y=buf.getDouble();
						double w=buf.getDouble();
						double h=buf.getDouble();
						boxes[0]=new Box(width*(x+w/2),height*(y+h/2),width*w,height*h);
					}
					case noneFound:
					{
						break;
					}
				}
				for(byte n=0;n<boxes.length;n++)
				{
					if(boxes[n]!=null)
					{
						SmartDashboard.putNumber("x-"+n, boxes[n].getX());
					}
				}
			}
			catch(Exception e)
			{
				e.printStackTrace();
				more = false;
			}
		}
		socket.close();
	}
	public synchronized Box[] getData(){return boxes;}
}