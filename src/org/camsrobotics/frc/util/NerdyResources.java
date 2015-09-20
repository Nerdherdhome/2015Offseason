package org.camsrobotics.frc.util;

import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Resources and Constants for the robot
 * 
 * @author Wesley
 *
 */
public class NerdyResources {
	//Prevents this class from being initialized
	private NerdyResources(){}
	
	/**
	 * Components for driver control
	 * 
	 * @author Wesley
	 *
	 */
	public static class Control {
		// Joysticks
		public static NerdyJoystick leftJoy			= new NerdyJoystick(0);
		public static NerdyJoystick rightJoy		= new NerdyJoystick(1);
		public static NerdyJoystick articJoy		= new NerdyJoystick(2);	
		
		/*
		 * Drive Buttons
		 */
		public static NerdyButton driveMode			= leftJoy.getButton(7);
		public static NerdyButton calibrateDrive	= leftJoy.getButton(4);
		
		/*
		 * Articulation Buttons
		 */
		// Manual Buttons
		public static NerdyButton openElevator		= articJoy.getButton(3);
		public static NerdyButton closeElevator		= articJoy.getButton(4);
		public static NerdyButton openIntake		= articJoy.getButton(5);
		public static NerdyButton closeIntake		= articJoy.getButton(6);
		public static NerdyButton intake			= articJoy.getButton(11);
		public static NerdyButton outake			= articJoy.getButton(12);
		public static NerdyButton stopIntakes		= articJoy.getButton(2); 
		public static NerdyButton calibrateArtic	= articJoy.getButton(1);
		public static NerdyButton openClaw			= articJoy.getButton(7);
		public static NerdyButton closeClaw 		= articJoy.getButton(8);
		// Automated Buttons
		public static NerdyButton stackToStack		= articJoy.getButton(10);
		
		// Override
		public static NerdyButton articOverride		= articJoy.getButton(9);
	}
	
	/**
	 * Components for the robot's drive subsystem
	 * 
	 * @author Wesley
	 *
	 */
	public static class Drive	{
		// Drive motors
		public static VictorSP ftLeftDrive		= new VictorSP(2);
		public static VictorSP ftRightDrive		= new VictorSP(4);
		public static VictorSP bkLeftDrive		= new VictorSP(1);
		public static VictorSP bkRightDrive		= new VictorSP(5);
		
		// Drive encoders
		public static Encoder ftLeftEncoder 	= new Encoder(0,1);
		public static Encoder ftRightEncoder	= new Encoder(2,3);
		public static Encoder bkLeftEncoder		= new Encoder(4,5);
		public static Encoder bkRightEncoder	= new Encoder(6,7);
		
		// Nav
		public static IMUAdvanced imu			= new IMUAdvanced(new SerialPort(57600,SerialPort.Port.kMXP));
	}
	
	/**
	 * Components for the articulation subsystem
	 * 
	 * @author Wesley
	 *
	 */
	public static class Articulation	{
		// Articulation motors
		public static CANTalon elevator			= new CANTalon(1);
		public static Talon leftIntake			= new Talon(6);
		public static Talon rightIntake			= new Talon(7);
		
		// Articulation Solenoids
		public static Compressor compressor		= new Compressor();
		public static DoubleSolenoid leftSol	= new DoubleSolenoid(0,1);
		public static DoubleSolenoid rightSol	= new DoubleSolenoid(2,3);
		public static DoubleSolenoid intakeSol	= new DoubleSolenoid(4,5);
		public static DoubleSolenoid canClaw	= new DoubleSolenoid(6,7);
		
		// Articulation Encoder
		public static Encoder elevatorEncoder	= new Encoder(8,9);
	}
	
	/**
	 * Robot Constants
	 * 
	 * @author Wesley
	 *
	 */
	public static class Constants	{
		/*
		 * 	Drive constants
		 */
		public final static double driveRotationP	= 0.00444444;
		public final static double driveRotationI	= 0.00004444;
		public final static double driveRotationD	= 0.00000000;
		
		/*
		 * 	Articulation constants
		 */
		public final static double articLiftP		= 0.05;
		public final static double articLiftI		= 0.004;
		public final static double articLiftD		= 1;
		//Autostack Heights
		public final static double stackHeight1		= 20.0;
		public final static double stackHeight2		= 8.0;
		public final static double stackHeight3		= 26.0;
		
		public final static double driveHeight		= 8.0;
		
		public final static double stackHeight21	= 32.0;
		public final static double stackHeight22	= 18.0;
		public final static double stackHeight23	= 38.0;
		//Autostack Pause
		public final static double pause			= .5;
	}
	
	/**
	 * Reports the sensors to the SmartDashboard
	 */
	public static void reportSensors()	{
		SmartDashboard.putNumber("Front Left Encoder Value", Drive.ftLeftEncoder.getRaw());
		SmartDashboard.putNumber("Front Right Encoder Value", Drive.ftRightEncoder.getRaw());
		SmartDashboard.putNumber("Back Left Encoder Value", Drive.bkLeftEncoder.getRaw());
		SmartDashboard.putNumber("Back Right Encoder Value", Drive.bkRightEncoder.getRaw());
		
		SmartDashboard.putNumber("Yaw", Drive.imu.getYaw());

		SmartDashboard.putNumber("Elevator Current", Articulation.elevator.getOutputCurrent());
		SmartDashboard.putNumber("Elevator Voltage", Articulation.elevator.getOutputVoltage());
		
		SmartDashboard.putNumber("Articulation Encoder", Articulation.elevatorEncoder.getRaw()/256+8);
	}
}
