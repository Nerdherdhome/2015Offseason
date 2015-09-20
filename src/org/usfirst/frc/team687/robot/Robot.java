
package org.usfirst.frc.team687.robot;

import org.camsrobotics.frc.subsystems.NerdyElevator;
import org.camsrobotics.frc.subsystems.NerdyDrive;
import org.camsrobotics.frc.subsystems.NerdyIntake;
import org.camsrobotics.frc.util.NerdyButton;
import org.camsrobotics.frc.util.NerdyDrivebase;
import org.camsrobotics.frc.util.NerdyJoystick;
import org.camsrobotics.frc.util.NerdyResources;
import org.camsrobotics.frc.util.NerdyStates;

import com.kauailabs.nav6.frc.IMUAdvanced;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is where the magic happens!
 */
public class Robot extends IterativeRobot {
	
	// The Bot
	NerdyBot			robot;
	
	NerdyDrivebase		drivebase;

	//	Subsystems
	NerdyDrive			drive;
	NerdyElevator		elevator;
	NerdyIntake			intake;
	
	// Joysticks
	NerdyJoystick		leftJoy;
	NerdyJoystick		rightJoy;
	NerdyJoystick		articJoy;
	
	// Drive Buttons
	NerdyButton			driveMode;
	NerdyButton			calibrateDrive;
	
	// Artic Buttons
	NerdyButton			openElevator;
	NerdyButton			closeElevator;
	NerdyButton			stackToStack;
	NerdyButton			articOverride;
	NerdyButton			openIntake;
	NerdyButton			closeIntake;
	NerdyButton			in;
	NerdyButton			out;
	NerdyButton			stopIntakes;
	NerdyButton			openClaw;
	NerdyButton			closeClaw;
	NerdyButton			calibrateArtic;
	
	// Sensors
	Encoder				ftLeftDriveEncoder;
	Encoder				ftRightDriveEncoder;
	Encoder				bkLeftDriveEncoder;
	Encoder				bkRightDriveEncoder;
	Encoder				articEncoder;
	DigitalInput		lowerLimit;
	DigitalInput		upperLimit;
	IMUAdvanced			imu;
	CANTalon			elevatorMotor;
	
    /**
     * Initializes the subsystems
     */
    public void robotInit() {
    	leftJoy			= NerdyResources.Control.leftJoy;
    	rightJoy		= NerdyResources.Control.rightJoy;
    	articJoy		= NerdyResources.Control.articJoy;
    	
    	driveMode		= NerdyResources.Control.driveMode;
    	calibrateDrive	= NerdyResources.Control.calibrateDrive;
    	
    	openElevator	= NerdyResources.Control.openElevator;
    	closeElevator	= NerdyResources.Control.closeElevator;
    	stackToStack	= NerdyResources.Control.stackToStack;
    	articOverride = NerdyResources.Control.articOverride;
    	openIntake		= NerdyResources.Control.openIntake;
    	closeIntake		= NerdyResources.Control.closeIntake;
    	in				= NerdyResources.Control.intake;
    	out				= NerdyResources.Control.outake;
    	stopIntakes		= NerdyResources.Control.stopIntakes;
    	openClaw		= NerdyResources.Control.openClaw;
    	closeClaw		= NerdyResources.Control.closeClaw;
    	calibrateArtic	= NerdyResources.Control.calibrateArtic;
    	
    	drivebase		= new NerdyDrivebase(NerdyResources.Drive.ftLeftDrive,
    						  NerdyResources.Drive.ftRightDrive,
    						  NerdyResources.Drive.bkLeftDrive,
    						  NerdyResources.Drive.bkRightDrive,
    						  NerdyResources.Drive.ftLeftEncoder,
    						  NerdyResources.Drive.ftRightEncoder,
    						  NerdyResources.Drive.bkLeftEncoder,
    						  NerdyResources.Drive.bkRightEncoder);
    	
    	drive			= new NerdyDrive(leftJoy, rightJoy, NerdyResources.Drive.imu, drivebase);
    	
    	elevator 		= new NerdyElevator(articJoy, 
							  NerdyResources.Articulation.elevator,
							  NerdyResources.Articulation.leftSol,
							  NerdyResources.Articulation.rightSol,
							  NerdyResources.Articulation.canClaw,
							  NerdyResources.Articulation.elevatorEncoder);
    	
    	intake 			= new NerdyIntake(NerdyResources.Articulation.leftIntake,
			    			  NerdyResources.Articulation.rightIntake,
			    			  NerdyResources.Articulation.intakeSol);
    	
    	robot			= new NerdyBot(drive, elevator, intake);

    	ftLeftDriveEncoder  = NerdyResources.Drive.ftLeftEncoder;
    	ftRightDriveEncoder = NerdyResources.Drive.ftRightEncoder;
    	bkLeftDriveEncoder  = NerdyResources.Drive.bkLeftEncoder;
    	bkRightDriveEncoder = NerdyResources.Drive.bkRightEncoder;
    	articEncoder        = NerdyResources.Articulation.elevatorEncoder;
    	
    	imu                 = NerdyResources.Drive.imu;
    	elevatorMotor       = NerdyResources.Articulation.elevator;
    	
    	NerdyStates.put("Robot", "Initializing");
    }
    
    /**
     * Initializes Disabled mode
     */
    public void disabledInit()	{
    	drive.calibrate();
    	NerdyStates.put("Robot", "Disabled");
    }
    
    /**
     * Disabled mode
     */
    public void disabledPeriodic()	{
    	reportStates();
    }

    /**
     * Initializes Autonomous mode
     */
    public void autonomousInit()	{
    	NerdyStates.put("Robot", "Autonomous Mode");
    }
    
    /**
     * Autonomous Mode
     */
    public void autonomousPeriodic() {
    	reportStates();
    }

    /**
     * Initializes Teleoperated Mode
     */
    public void teleopInit()	{
    	NerdyStates.put("Robot", "Initializing");
    }
    
    /**
     * Teleoperated Mode
     */
    public void teleopPeriodic() {
    	updateButtons();
    	
    	// Drive functions
        if(driveMode.wasPressed())	{
        	switchDriveMode();
        }
        if(calibrateDrive.wasPressed())	{
        	robot.calibrateDrive();
        }
        
        // Artic Functions
        if(openElevator.wasPressed())	{
        	robot.openElevator();
        }	else if(closeElevator.wasPressed())	{
        	robot.closeElevator();
        }
        
        if(stackToStack.wasPressed())	{
        	robot.stackToStack();
        }
        
        if(articOverride.wasPressed())	{
        	robot.enableArtic();
        }
        
    	if(openIntake.wasPressed())	{
    		robot.openIntakes();
    	}	else if(closeIntake.wasPressed())	{
    		robot.closeIntakes();
    	}
    	
    	if(openClaw.wasPressed()) robot.openClaw();
    	if(closeClaw.wasPressed()) robot.closeClaw();
    	
    	if(in.get())	{
    		robot.intake();
    	}	else if(out.get())	{
    		robot.outake();
    	}	else if(stopIntakes.get())	{
    		robot.idleIntakes();
    	}
    	
    	if(calibrateArtic.wasPressed())	{
        	robot.calibrateElevator();
        }
        
    	reportStates();
    	NerdyResources.reportSensors();
    	robot.run();
    }
    
    /**
     * Updates all the buttons
     */
    void updateButtons()	{
    	driveMode.update();
    	calibrateDrive.update();
    	
    	openElevator.update();
    	closeElevator.update();
    	stackToStack.update();
    	articOverride.update();
    	openIntake.update();
    	closeIntake.update();
    	in.update();
    	out.update();
    	stopIntakes.update();
    	openClaw.update();
    	closeClaw.update();
    	calibrateArtic.update();
    }
    
    /**
     * Switches drive modes
     */
    void switchDriveMode()	{
    	if(NerdyStates.get("Drive") == NerdyDrive.State.kFieldCentric)	{
    		robot.robotCentric();
    	}	else if(NerdyStates.get("Drive") == NerdyDrive.State.kRobotCentric)	{
    		robot.fieldCentric();
    	}
    }
    
    /**
     * Reports the subsystem states to the SmartDashboard
     */
    void reportStates()	{
    	SmartDashboard.putString("Robot State", NerdyStates.get("Robot").toString());
    	SmartDashboard.putString("Drive State", NerdyStates.get("Drive").toString());
    	SmartDashboard.putString("Elevator", NerdyStates.get("Elevator").toString());
    	SmartDashboard.putString("Intake State", NerdyStates.get("Intake").toString());
    }
    
}
