package org.usfirst.frc.team687.robot;

import org.camsrobotics.frc.subsystems.NerdyDrive;
import org.camsrobotics.frc.subsystems.NerdyElevator;
import org.camsrobotics.frc.subsystems.NerdyIntake;
import org.camsrobotics.frc.util.NerdyResources;

import edu.wpi.first.wpilibj.Compressor;

/**
 * Wrapper for all three subsystems
 * 
 * @author Wesley
 *
 */
public class NerdyBot {
	// Subsystems
	private NerdyDrive m_drive;
	private NerdyElevator m_elevator;
	private NerdyIntake m_intake;
	private Compressor m_compressor;
	private boolean m_inToggle = false;
	private boolean m_outToggle = false;
	
	/**
	 * Default Constructor
	 * 
	 * @param drive
	 * @param elevator
	 * @param intake
	 */
	public NerdyBot(NerdyDrive drive, NerdyElevator elevator, NerdyIntake intake)	{
		m_drive			= drive;
		m_elevator		= elevator;
		m_intake		= intake;
		m_compressor	= NerdyResources.Articulation.compressor;
		m_compressor.start();
	}
	
	// Drive functions
	
	/**
	 * Runs the robot. Should be called every iteration
	 */
	public void run()	{
		m_drive.run();
		m_elevator.run();
		m_intake.run();
	}
	
	/**
	 * Disables the drivebase
	 */
	public void disableDrive()	{
		m_drive.disable();
	}
	
	/**
	 * Enables the drivebase
	 */
	public void enableDrive()	{
		m_drive.enable();
	}
	
	/**
	 * Changes the drive into robot centric
	 */
	public void robotCentric()	{
		m_drive.robotCentric();
	}
	
	/**
	 * Changes the drive into Field Centric
	 */
	public void fieldCentric()	{
		m_drive.fieldCentric();
	}
	
	/**
	 * Calibrates the drive. Should be facing forward when called.
	 */
	public void calibrateDrive()	{
		m_drive.calibrate();
	}
	
	// Elevator Functions
	
	/**
	 * Disables the elevator
	 */
	public void disableElevator()	{
		m_elevator.disable();
	}
	
	/**
	 * Enables the elevator
	 */
	public void enableElevator()	{
		m_elevator.enable();
	}
	
	/**
	 * Switches the elevator to manual mode
	 */
	public void manualElevator()	{
		m_elevator.manual();
	}
	
	/**
	 * Auto Stacks, ending at the configured stacking height
	 */
	public void stackToStack()	{
		m_elevator.stackToStack();
	}
	
	/**
	 * Auto Stacks, ending at the configured drive height
	 */
	public void stackToStack2()	{
		m_elevator.stackToStack2();
	}
	
	/**
	 * Switches the elevator to command mode and sets the desired height
	 * 
	 * @param desired The number of inches off the floor
	 */
	public void commandElevator(double desired)	{
		m_elevator.command(desired);
	}

	/**
	 * Opens the elevator arms
	 */
	public void openElevator()	{
		m_elevator.open();
	}
	
	/**
	 * Closes the elevator arms
	 */
	public void closeElevator()	{
		m_elevator.close();
	}
	
	/**
	 * Calibrates the elevator. Should be on the bottom when called.
	 */
	public void calibrateElevator()	{
		m_elevator.calibrate();
	}
	
	// Intake functions
	
	/**
	 * Disables the intakes
	 */
	public void disableIntakes()	{
		m_intake.disable();
	}
	
	/**
	 * Enables the intakes
	 */
	public void enableIntakes()	{
		m_intake.enable();
	}
	
	/**
	 * Intakes a tote
	 */
	public void intake()	{
		m_intake.intake();
	}
	
	/**
	 * Outakes a tote
	 */
	public void outake()	{
		m_intake.outake();
	}
	
	/**
	 * Opens the can claw
	 */
	public void openClaw()	{
		m_elevator.openClaw();
	}
	
	/**
	 * Closes the can claw
	 */
	public void closeClaw()	{
		m_elevator.closeClaw();
	}
	
	public void intakeToggle()	{
		if(m_inToggle)	{
			idleIntakes();
			m_inToggle = false;
		}	else	{
			intake();
			m_outToggle = false;
			m_inToggle = true;
		}
	}
	
	public void outakeToggle()	{
		if(m_outToggle)	{
			idleIntakes();
			m_outToggle = false;
		}	else	{
			outake();
			m_inToggle = false;
			m_outToggle = true;
		}
	}
	
	/**
	 * Idles the intakes
	 */
	public void idleIntakes()	{
		m_intake.idle();
	}
	
	/**
	 * Open the intakes
	 */
	public void openIntakes()	{
		m_intake.open();
	}
	
	/**
	 * Close the intakes
	 */
	public void closeIntakes()	{
		m_intake.close();
	}
	
	// Articulation functions
	
	public void disableArtic()	{
		m_elevator.disable();
		m_intake.disable();
	}
	
	public void enableArtic()	{
		m_elevator.enable();
		m_intake.enable();
	}
}
