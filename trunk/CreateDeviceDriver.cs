using System;
using System.Collections.Generic;
//using System.Linq;
using System.Text;
using System.IO.Ports;

namespace motion
{
    
    public class CreateDeviceDriver
    {
        private const int MAX_VELOCITY = 500;
        private const int MIN_VELOCITY = 0;

        public SerialPort serialPort1 = new SerialPort();

        public CreateDeviceDriver()
        {
            //InitializeComponent();
        }

        //OpenPort button service
        public void OpenPort_Click()
        {
            InitializeSerialPort();
        }

        //SafeMode button service
        public void SafeMode_Click()
        {
            SetMode();
            LED_On();
        }

        //Back button service
        public void Back_Click()
        {
            byte[] data = new byte[5];
            int numBytes = 5;

            data[0] = 145;  //Direct Drive command
            data[1] = ((-200)>>8 & 0x00FF);  //[Right velocity high byte] 
            data[2] = ((-200) & 0x00FF);    //[Right velocity low byte]
            data[3] = ((-200) >> 8 & 0x00FF);  //[Left velocity high byte]
            data[4] = ((-200) & 0x00FF);    //[Left velocity low byte]

            serialPort1.Write(data, 0, numBytes);
        }

        //Stop Button service
        public void Stop_Click()
        {
            byte[] data = new byte[5];
            int numBytes = 5;

            data[0] = 145;  //Direct Drive command
            data[1] = 0;    //[Right velocity high byte] 
            data[2] = 0;  //[Right velocity low byte]
            data[3] = 0;    //[Left velocity high byte]
            data[4] = 0;  //[Left velocity low byte]

            serialPort1.Write(data, 0, numBytes);
        }

        //Go button service
        public void Go_Click()
        {
            
            byte[] data = new byte[5];  
            int numBytes = 5;  

            data[0] = 145;  //Direct Drive command
            data[1] = 0;    //[Right velocity high byte] 
            data[2] = 200;  //[Right velocity low byte]
            data[3] = 0;    //[Left velocity high byte]
            data[4] = 200;  //[Left velocity low byte]

            serialPort1.Write(data, 0, numBytes);
        }

        //TurnRight Service
        public void TurnRight_Click(int Velocity)
        {
            byte[] data = new byte[5];
            int numBytes = 5;

            if ((Velocity <= MAX_VELOCITY) && (Velocity >= MIN_VELOCITY))
            {

                data[0] = 145;  //Direct Drive command
                data[1] = (byte)((-Velocity) >> 8 & 0x00FF);  //[Right velocity high byte] 
                data[2] = (byte)((-Velocity) & 0x00FF);    //[Right velocity low byte]
                data[3] = 0;  //[Left velocity high byte]
                data[4] = (byte)Velocity;    //[Left velocity low byte]

                serialPort1.Write(data, 0, numBytes);
            }else{
                Console.WriteLine("Velocity is out of range !!");
            }
        }

        //TurnLeft Service
        public void TurnLeft_Click(int Velocity)
        {
            byte[] data = new byte[5];
            int numBytes = 5;

            if ((Velocity <= MAX_VELOCITY) && (Velocity >= MIN_VELOCITY))
            {
                data[0] = 145;  //Direct Drive command
                data[1] = 0;  //[Right velocity high byte] 
                data[2] = (byte)Velocity;    //[Right velocity low byte]
                data[3] = (byte)((-Velocity) >> 8 & 0x00FF);  //[Left velocity high byte]
                data[4] = (byte)((-Velocity) & 0x00FF);    //[Left velocity low byte]

                serialPort1.Write(data, 0, numBytes);
            }
            else
            {
                Console.WriteLine("Velocity is out of range !!");
            }
        }

        public void InitializeSerialPort()
        {
            serialPort1.PortName = "COM4";
            serialPort1.BaudRate = 57600;

            serialPort1.Open();
        }

        public void SetMode()
        {
            byte[] data = new byte[2];
            int numBytes = 2;

            data[0] = 128;  //Direct Drive command
            data[1] = 131;  //[Right velocity high byte] 

            serialPort1.Write(data, 0, numBytes);
        }

        public void LED_On()
        {
            byte[] data = new byte[4];
            int numBytes = 4;

            data[0] = 139;  //LED command
            data[1] = 8;    //Select LED (Play : 8, power : 2) 
            data[2] = 0;    //Color 0 = green, 255 = red
            data[3] = 128;  //Intensity

            serialPort1.Write(data, 0, numBytes);

            data[0] = 139;  //LED command
            data[1] = 2;    //Select LED (Play : 8, power : 2) 
            data[2] = 255;    //Color 0 = green, 255 = red
            data[3] = 128;  //Intensity

            serialPort1.Write(data, 0, numBytes);
        }


        //sensor reading
        public byte ReadSensor_Click(byte packetID)
        {
            byte[] data = new byte[2];
            int numBytes = 2;
            byte readbuffer;

            data[0] = 142;  //Read Sensors
            data[1] = packetID;    //Packet ID

            serialPort1.Write(data, 0, numBytes);

            readbuffer = 255;
            do
            {
                readbuffer = (byte)serialPort1.ReadByte();
            } while (readbuffer == 255);

            return readbuffer;
        }

        public void WaitForAngle(int theAngle)
        {
            //don't allow any action until turn is complete
            int numBytes = 3;
            byte[] data = new byte[numBytes];

            //defaults to right turn
            data[0] = 157;  //WaitEvent command
            data[1] = (byte)((theAngle) >> 8 & 0x00FF); //angle high byte
            data[2] = (byte)((theAngle) & 0x00FF);     //angle low byte 

            serialPort1.Write(data, 0, numBytes);
        }

    }



}
