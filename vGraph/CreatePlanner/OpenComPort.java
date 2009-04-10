/**
 * IMPORTANCE NOTICE
 * 
 * Configuration for using javax.comm.*
 * Copy comm.jar to \jre1.6.0_07\lib
 * Copy javax.comm.properties to \jre1.6.0_07\lib
 * Copy win32com.dll to \jre1.6.0_07\dll
 * Update eclipse\Project\properties -> Java build path\Libraries\Add External JARs
 * 
 * */

/**
 * @author joseph
 * @date 10/23/2008
 */
package CreatePlanner;

import javax.comm.*;		// See the Notice !!!
import java.util.*;
import java.io.*;			

public class OpenComPort {
	
	SerialPort port = null;
	OutputStream os = null;
	InputStream is = null;

	/** The baud rate to use. */
	public static final int BAUD = 57600;
	String wantedPortName = "COM5";
	
	OpenComPort()
	{
	}
	
	public int SetupBAM(){
		Enumeration<?> portIdentifiers = CommPortIdentifier.getPortIdentifiers();
		CommPortIdentifier portId = null;
		
		while (portIdentifiers.hasMoreElements())
		{
		    CommPortIdentifier pid = (CommPortIdentifier) portIdentifiers.nextElement();
		    if(pid.getPortType() == CommPortIdentifier.PORT_SERIAL &&
		       pid.getName().equals(wantedPortName)) 
		    {
		        portId = pid;
		        System.out.println("Found port: "+pid.getName());
		        break;
		    }
		}
		if(portId == null)
		{
		    System.err.println("Could not find serial port " + wantedPortName);
		    System.exit(1);
		}
		
		try {
		    port = (SerialPort) portId.open(
		        "iRobotBAM", // Name of the application asking for the port 
		        10000   // Wait max. 10 sec. to acquire port
		    );
		} catch(PortInUseException e) {
		    System.err.println("Port already in use: " + e);
		    System.exit(1);
		}
		
		try{
		port.setSerialPortParams(
			    57600,
			    SerialPort.DATABITS_8,
			    SerialPort.STOPBITS_1,
			    SerialPort.PARITY_NONE);
		} catch(UnsupportedCommOperationException e){
			System.err.println("Fail to set up port: " + e);
		}
		
		return 0;
	}
	
	public void Write(byte[] data){
		try{
			os = port.getOutputStream();
			}catch(IOException e){
			System.err.println(e);
		}
		// Write to the output 
		try{
			os.write(data);
		}catch(IOException e) {}
	}
	
	public void Read(byte[] data){
		try{
			is = port.getInputStream();
			}catch(IOException e){
				System.err.println(e);
		}
		try{
			is.read(data);
		}catch(IOException e) {}
	}
	
}

