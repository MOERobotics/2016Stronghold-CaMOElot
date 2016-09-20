package org.usfirst.frc.team365.robot;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Udp implements Runnable {
	static final short noneFound = 1, oneFound = 2, twoFound = 3;
	static final int port = 5801;
	static final double width = 320, height = 240;
	int lastID;
	DatagramSocket socket;
	protected volatile boolean more = true;
	protected String name;
	protected volatile Box[] boxes = new Box[2];

	public Udp(String name) throws SocketException {
		this.name = name;
		socket = new DatagramSocket(port);
	}

	public void run() {
		ByteBuffer buf = ByteBuffer.allocate(72);
		DatagramPacket packet = new DatagramPacket(buf.array(), buf.limit());
		while (true) {
			try {
				buf.position(0);
				packet.setLength(buf.limit());
		//		System.out.println("preRecieve");
				socket.receive(packet);
		//		System.out.print("postRecieve ");
				final int id = buf.getInt();
				if (id <= lastID)
					continue;
				lastID = id;
		//		System.out.print(id+" ");
				final short status = buf.getShort();
		//		System.out.println(status);
				buf.getShort();
				Box[] data = new Box[2];
				// doubles as l,r,w,h
				double x, y, w, h;
				switch (status) {
					case twoFound:
						x = buf.getDouble();
						y = buf.getDouble();
						w = buf.getDouble();
						h = buf.getDouble();
						data[1] = new Box(width * (x + w / 2), height * (y + h / 2), width * w, height * h);
					case oneFound:
						x = buf.getDouble();
						y = buf.getDouble();
						w = buf.getDouble();
						h = buf.getDouble();
						data[0] = new Box(width * (x + w / 2), height * (y + h / 2), width * w, height * h);
					case noneFound:
						break;
				}
				this.boxes = data;
				for (byte n = 0; n < boxes.length; n++)
					if (boxes[n] != null){
						SmartDashboard.putNumber("x-" + n, boxes[n].getX());
						SmartDashboard.putNumber("y-" + n, boxes[n].getY());
					}
			} catch (Exception e) {
				e.printStackTrace();
				break;
			}
		}
		socket.close();
	}

	public synchronized Box[] getData() {
		return boxes;
	}
	
}