package pid.netclient;

import java.io.IOException;
import java.net.InetAddress;
import java.net.DatagramSocket;
import java.net.Socket;
import java.net.DatagramPacket;
import java.net.UnknownHostException;

public class PID_UDP_Client implements IPIDClient {
	private static final int TIME_OUT = 5000;   // 5 secs
    // timeout used when waiting in receive()
	private static final int PACKET_SIZE = 1024;  // max size of a message
	
	volatile boolean active = true;
	private DatagramSocket socket = null;
	private int SERVER_PORT = 5000;
	private String SERVER_IP = null;
	private InetAddress serverAddr = null;
	
	public boolean serverConnect(String serverIP, int port) throws UnknownHostException, IOException {
		SERVER_IP = serverIP;
		SERVER_PORT = port;
		if(SERVER_IP != null) {
			serverAddr = InetAddress.getByName(SERVER_IP);
			socket = new DatagramSocket();
			socket.setSoTimeout(TIME_OUT);
			return true;
		}
		else return false;
	}
	
	void sendServerMessage(String msg)
	  // Send message to NameServer
	  {
		//if(socket == null) return;
	    try {
	      DatagramPacket sendPacket =
	          new DatagramPacket( msg.getBytes(), msg.length(), 
	   						serverAddr, SERVER_PORT);
	      socket.send( sendPacket );
	    }
	    catch(IOException ioe)
	    {  System.out.println(ioe);  }
	  } // end of sendServerMessage()
	public void run() {
		while(active) {
			int azimut = PIDNetActivity.avgAzimut;
			sendServerMessage(Integer.toString(azimut));
			try {
				Thread.sleep(10);
			}
			catch(InterruptedException ex){
				active = false;
			}
		}
		socket.close();
	}
}
