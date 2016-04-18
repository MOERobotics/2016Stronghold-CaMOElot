package org.usfirst.frc.team365.robot;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;

public class Udp implements Runnable
{
	static final short noneFound=1, oneFound=2, twoFound=3;
	static final int port=5801;
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
						double l=buf.getDouble();
						double r=buf.getDouble();
						double w=buf.getDouble();
						double h=buf.getDouble();
						boxes[1]=new Box(l,r,w,h);
					}
					case oneFound:
					{
						double l=buf.getDouble();
						double r=buf.getDouble();
						double w=buf.getDouble();
						double h=buf.getDouble();
						boxes[0]=new Box(l,r,w,h);
					}
					case noneFound:
					{
						break;
					}
				}
				for(byte n=0;n<boxes.length;n++)
					if(boxes[n]!=null)
						System.out.print(boxes[n].toString());
				System.out.println();
//				byte[]data=packet.getData();
//				lastData=new String(data, 0, packet.getLength());
//				System.out.println(lastData);
				
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