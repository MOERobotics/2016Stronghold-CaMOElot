package org.usfirst.frc.team365.robot;

import java.net.SocketException;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot implements PIDOutput {

//	 CameraServer server;

	AHRS navX;

	CANTalon driveLA = new CANTalon(12);
	CANTalon driveLB = new CANTalon(13);
	CANTalon driveLC = new CANTalon(14);
	CANTalon driveRA = new CANTalon(1);
	CANTalon driveRB = new CANTalon(2);
	CANTalon driveRC = new CANTalon(3);
	CANTalon shooterA = new CANTalon(9);
	CANTalon shooterB = new CANTalon(10);
	CANTalon shootAngle = new CANTalon(11);
	CANTalon scaleL = new CANTalon(15);
	CANTalon scaleR = new CANTalon(0);
	CANTalon collector = new CANTalon(6);
	CANTalon ballControl = new CANTalon(5);
	CANTalon arm = new CANTalon(4);

	Joystick driveStick = new Joystick(0);
	Joystick funStick = new Joystick(1);

	Counter shootSpeedA = new Counter(5);
	Counter shootSpeedB = new Counter(4);

	Encoder distanceL = new Encoder(0, 1, false, EncodingType.k1X);
	Encoder distanceR = new Encoder(2, 3, true, EncodingType.k1X);

	DigitalInput ballSensor = new DigitalInput(7);
	DigitalInput armLimit = new DigitalInput(6);
	DigitalInput tapeSensor = new DigitalInput(8);
	DigitalInput armUpLimit = new DigitalInput(9);

	AnalogInput armPot = new AnalogInput(0);
	AnalogInput launchAngle = new AnalogInput(1);
	AnalogInput moeSonar = new AnalogInput(2);

	Timer autoTimer = new Timer();

	PIDController angleController;
	PIDController driveStraight;
	
	NetworkTable table;

	int disabledLoop;
	int teleopLoop;
	int autoLoop;
	int testLoop;
	int startLoop;
	int autoStep;
	int loopCount;
	int autoRoutine;
	int turnCount;
	int floorCount;
	boolean shooterOn;
	int onTape;
	boolean pastButton2D;
	boolean pastButton4;
	boolean pastButton3D;
	boolean pastButton4D;
	boolean pastButton5D;
	boolean pastButton6D;
	boolean pastButton7D;
	boolean pastButton8D;
	boolean pastButton9D;
	boolean pastButton10D;
	boolean pastButton11D;
	boolean pastButton12D;
	boolean pidContOn;
	boolean onSpeed;
	int defenseStep;
	double moat;
	
	double powerA;
	double powerB;
	double speedSumA;
	double speedSumB;
	double setSpeedA;
	double setSpeedB;
	double direction;
	double startYaw;
	double turnToAngle;
	double turnSum;
	double maxTurnPower;
	double lastOffYaw;
	double minRoll;
	double maxRoll;
	double rollOffset;
	double defenseDist;
	double seeTapeDist;
	double wallDist;
	double lowBarDist;
//	double sonarDist;
	// double PIDKP;
	// double PIDKI;
	// double PIDKD;
	double desiredSpeedA = 4200.0;
	double desiredSpeedB = 4000.0;

	final double KPspeedA = 0.0001; // .0004;
	final double KPspeedB = 0.0001; // .0004;
	 final double KIspeedA = .00001;
	 final double KIspeedB = .00001;

	final double armDown = 1.9;
	final double armCheval = 2.6;
	final double armBumper = 3.4;
	final double arm90 = 4.4;
	final double armClimb = 4.6;

//	final double farAngleLimit = 3.8;
//	final double closeAngleLimit = 2.9;
	final double shooterClose = 2.9; // 32.8 degrees
	final double shooterOuterWorks = 3.8; // 50.7 degrees
	
	//grip: 480x320
	//liam: 640x480
	final double halfWidth = 240.0;
	final double halfHeight = 160.;
	boolean analyzePicture=true;
	boolean foundTarget = false;
	
	Udp udpVision;
	Thread udpThread;
	
	AutoFile autoFile=new AutoFile();
	int autoFileChoice=0;
	final static int NAK=0;
	AutoData autoData;
	
	

	public Robot() {
		
//		  server = CameraServer.getInstance(); 
//		  server.setSize(1);
//		  server.setQuality(50); 
//		  server.startAutomaticCapture("cam1");
		
		table=NetworkTable.getTable("GRIP");
		 
		navX = new AHRS(SPI.Port.kMXP, (byte) 50);
	}

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {

		driveRA.setInverted(true);
		driveRB.setInverted(true);
		driveRC.setInverted(true);

		driveLA.enableBrakeMode(true);
		driveLB.enableBrakeMode(true);
		driveLC.enableBrakeMode(true);
		driveRA.enableBrakeMode(true);
		driveRB.enableBrakeMode(true);
		driveRC.enableBrakeMode(true);

		shootSpeedA.setSamplesToAverage(5);

		arm.enableBrakeMode(true);
		shootAngle.enableBrakeMode(true);
		ballControl.enableBrakeMode(true);

		// SmartDashboard.putNumber("desiredSpeedA", 4000.);
		// SmartDashboard.putNumber("desiredSpeedB", 4000.);
		// SmartDashboard.putNumber("KP", .04);
		// SmartDashboard.putNumber("KI", 0);
		// SmartDashboard.putNumber("KD", .04);
		SmartDashboard.putNumber("autoChoice", 0);
		SmartDashboard.putNumber("Moat", 0);

		angleController = new PIDController(0.03, 0.0005, 0.5, navX, this);
		angleController.setContinuous();
		angleController.setInputRange(-180.0, 180.0);
		angleController.setOutputRange(-0.6, 0.6);
		angleController.setAbsoluteTolerance(1.0);

		driveStraight = new PIDController(0.04, 0.00005, 0.03, navX, this);
		driveStraight.setContinuous();
		driveStraight.setInputRange(-180.0, 180.0);
		driveStraight.setOutputRange(-1.0, 1.0);
		
		autoFileChoice=autoFile.readAutoFile();
		
		
		try
		{
			udpVision=new Udp("vision");
			udpThread=new Thread(udpVision);
			udpThread.start();
		}
		catch(SocketException se)
		{
			
		}
		
	}

	public void disabledInit() {
		disabledLoop = 0;
		shooterOn = false;
		shooterA.set(0);
		shooterB.set(0);
		arm.set(0);
		// driveRobot(0,0);
		collector.set(0);
		if (angleController.isEnabled()) {
			angleController.disable();
		}
		if (driveStraight.isEnabled()) {
			driveStraight.disable();
		}
	}

	public void disabledPeriodic() {
		disabledLoop++;

		if(driveStick.getRawButton(1))
			autoFile.writeAutoFile(Integer.toString((int)SmartDashboard.getNumber("dfltChoice")));
		
		if (driveStick.getRawButton(4)) {
			distanceL.reset();
			distanceR.reset();
		}

		if (driveStick.getRawButton(2)) {
			navX.zeroYaw();
			if (!navX.isCalibrating())
				rollOffset = navX.getRoll();
		}

		if (driveStick.getRawButton(3)) {
			minRoll = 0;
			maxRoll = 0;
		}
		
		if (driveStick.getRawButton(6)) autoRoutine = 1;
		else if (driveStick.getRawButton(8)) autoRoutine = 2;
		else if (driveStick.getRawButton(10)) autoRoutine = 3;
		else if (driveStick.getRawButton(12)) autoRoutine = 4;
		else if (driveStick.getRawButton(11)) autoRoutine = 5;
		else if (driveStick.getRawButton(5)) autoRoutine = 0;
		else if (driveStick.getRawButton(7)) autoRoutine = 6;
		else if (driveStick.getRawButton(9)) autoRoutine = 7;
		else if (funStick.getRawButton(6)) autoRoutine = 8;
		else if (funStick.getRawButton(7)) autoRoutine = 9;
		else if (funStick.getRawButton(10)) autoRoutine = 10;
		else if (funStick.getRawButton(11)) autoRoutine = 11;
		
		if (funStick.getRawButton(9)) moat = 1;
		else if (funStick.getRawButton(8)) moat = 0;

		if (disabledLoop % 25 == 0) {
			SmartDashboard.putNumber("powerA", powerA);
			SmartDashboard.putNumber("speedA", 60. / shootSpeedA.getPeriod());
			SmartDashboard.putNumber("powerB", powerB);
			SmartDashboard.putNumber("speedB", 60. / shootSpeedB.getPeriod());
			SmartDashboard.putNumber("desiredSpeedA", setSpeedA);
			SmartDashboard.putNumber("desiredSpeedB", setSpeedB);

			SmartDashboard.putBoolean("ballSensor", ballSensor.get());
			SmartDashboard.putBoolean("tapeSensor", tapeSensor.get());
			SmartDashboard.putNumber("armPot", armPot.getAverageVoltage());
			SmartDashboard.putNumber("shooterAngle", launchAngle.getVoltage());
			SmartDashboard.putNumber("distL", distanceL.getRaw());
			SmartDashboard.putNumber("distR", distanceR.getRaw());
			SmartDashboard.putBoolean("armLimit", armLimit.get());
			SmartDashboard.putBoolean("armUpLimit", armUpLimit.get());
			SmartDashboard.putNumber("yaw", navX.getYaw());
			SmartDashboard.putNumber("pitch", navX.getPitch());
			SmartDashboard.putNumber("roll", navX.getRoll());
			SmartDashboard.putBoolean("navXenabled", navX.isConnected());
			SmartDashboard.putNumber("minRoll", minRoll);
			SmartDashboard.putNumber("maxRoll", maxRoll);
			SmartDashboard.putNumber("lastAutoStep", autoStep);
			SmartDashboard.putNumber("defenseDistance", defenseDist);
			SmartDashboard.putNumber("seeTapeDist", seeTapeDist);
			SmartDashboard.putNumber("wallDist", wallDist);
			SmartDashboard.putBoolean("onSpeed", onSpeed);
			SmartDashboard.putNumber("sonar", moeSonar.getAverageVoltage());
			SmartDashboard.putNumber("turnAngle", turnToAngle);
			SmartDashboard.putBoolean("shooterOn", false);

			
			SmartDashboard.putNumber("autoChoice", autoRoutine);
			SmartDashboard.putNumber("moat", moat);
//			double chooseAuto = SmartDashboard.getNumber("autoChoice");
//			autoRoutine = (int) chooseAuto;
			
//			moat=(int)(SmartDashboard.getNumber("Moat"));
			
//			SmartDashboard.putNumber("AutoFile", getAuto());
			
			//autoRoutine=getAuto();

		}
	}

	public void teleopInit() {
//		autoData.save();
		
		teleopLoop = 0;
		shooterOn = false;
		floorCount = 0;
		pastButton4 = false;
		pidContOn = false;
//		rollOffset = navX.getRoll();
		pastButton3D = false;
		pastButton4D = false;
		pastButton2D = false;
		pastButton5D = false;
		pastButton6D = false;
		pastButton7D = false;
		pastButton8D = false;
		pastButton9D = false;
		pastButton10D = false;
		pastButton11D = false;
		pastButton12D = false;
		driveStraight.setOutputRange(-1.0, 1.0);
		onTape = 0;
	//	analyzePicture = true;
	}

	public void teleopPeriodic() {
		teleopLoop++;
		boolean newButton4 = funStick.getRawButton(4);
		boolean newButton3D = driveStick.getRawButton(3);
		boolean newButton2D = driveStick.getRawButton(2);
		boolean newButton4D = driveStick.getRawButton(4);
		boolean newButton5D = driveStick.getRawButton(5);
		boolean newButton6D = driveStick.getRawButton(6);
		boolean newButton7D = driveStick.getRawButton(7);
		boolean newButton8D = driveStick.getRawButton(8);
		boolean newButton9D = driveStick.getRawButton(9);
		boolean newButton10D = driveStick.getRawButton(10);
		boolean newButton11D = driveStick.getRawButton(11);
		boolean newButton12D = driveStick.getRawButton(12);
		
		
		
		// Do we want to run the shooter?
		if (newButton4) {
			shooterOn = true;
			if (!pastButton4) {
				speedSumA = 0;
				speedSumB = 0;
				startLoop = teleopLoop;
				setSpeedA = desiredSpeedA;
				setSpeedB = desiredSpeedB;
				// setSpeedA = SmartDashboard.getNumber("desiredSpeedA");
				// setSpeedB = SmartDashboard.getNumber("desiredSpeedB");
			}

		} else if (funStick.getRawButton(5)) {
			shooterOn = false;
			shooterA.set(0);
			shooterB.set(0);
		}

		// If shooter is "on", set shooter motors to desired speeds
		// then check to see if we want to shoot a boulder?
		if (shooterOn) {
			double zValue = funStick.getZ();
			if (zValue  > 0.5) {
				shooterA.set(-1.0);
				shooterB.set(1.0);
			}
			else if (zValue < -0.5){
	//			testShooterSpeeds();
				/*
				 * if (newButton9D && !pastButton9D) { desiredSpeedA = desiredSpeedA
				 * + 100.0; desiredSpeedB = desiredSpeedB + 100.0;
				 * 
				 * } else if (newButton10D && !pastButton10D) { desiredSpeedA =
				 * desiredSpeedA - 100.0; desiredSpeedB = desiredSpeedB - 100.0; }
				 */
				setSpeedA = desiredSpeedA;
				setSpeedB = desiredSpeedB;

							controlShooter();

				if (funStick.getTrigger()) {
					ballControl.set(1.0);
				} else
					ballControl.set(0);
			}
			else {
				shooterA.set(0);
				shooterB.set(0);
			}
		}

		// If we are not running the shooter we can run the collector
		if (!shooterOn) {
			controlCollector();
		} else
			collector.set(0);

		// Raise and lower arm as desired
		controlArm();

		// Change the shooter angle
		controlShooterAngle();

		// Move scaling arms
		controlScalingArms();
		/*
		if (funStick.getRawButton(11)&&teleopLoop%20==0)
		{
			pictureAnalysis();
		}
		*/
		/*
		if (pidContOn) {

			if (driveStick.getTrigger()) {
				// if (newButton2D && !pastButton2D) {
				if (angleController.isEnabled())
					angleController.disable();
				if (driveStraight.isEnabled())
					driveStraight.disable();
				pidContOn = false;
			}
			else {
				overDefenses();
			}
		}
		
		else if (newButton6D && !pastButton6D) { // drive straight
			startYaw = navX.getYaw();
			// turnToAngle = startYaw + 3;
			loopCount = 0;
			direction = -0.5;
			floorCount = 0;
			minRoll = 0;
			maxRoll = 0;
			
//			driveStraight.setPID(.04, 0, .03);
			driveStraight.setSetpoint(startYaw);
			driveStraight.enable();
			pidContOn = true;
			defenseStep = 1;
		} else if (newButton8D && !pastButton8D) { // drive straight
			startYaw = navX.getYaw();
			// turnToAngle = startYaw + 3;
			loopCount = 0;
			direction = 0.6;
			floorCount = 0;
			minRoll = 0;
			maxRoll = 0;
			// double PIDKP = SmartDashboard.getNumber("KP");
			// double PIDKI = SmartDashboard.getNumber("KI");
			// double PIDKD = SmartDashboard.getNumber("KD");
//			driveStraight.setPID(.04, 0, .03);
			driveStraight.setSetpoint(startYaw);
			driveStraight.enable();
			pidContOn = true;
			defenseStep = 1;
		} else if (newButton10D && !pastButton10D) { // drive straight
			startYaw = navX.getYaw();
			// turnToAngle = startYaw + 3;
			loopCount = 0;
			direction = 0.7;
			floorCount = 0;
			minRoll = 0;
			maxRoll = 0;
			// double PIDKP = SmartDashboard.getNumber("KP");
			// double PIDKI = SmartDashboard.getNumber("KI");
			// double PIDKD = SmartDashboard.getNumber("KD");
//			driveStraight.setPID(.04, 0, .03);
			driveStraight.setSetpoint(startYaw);
			driveStraight.enable();
			pidContOn = true;
			defenseStep = 1;
			
		}
		*/
		
//		 else 
			 if (newButton3D) { // programmed turn routine to 0 degrees
			if (!pastButton3D) {
				lastOffYaw = 0;
				turnSum = 0;
			}

			turnRobot(0.0);
		} else if (newButton2D) { // programmed to turn 4 degrees left
			if (!pastButton2D) {
				lastOffYaw = 0;
				turnSum = 0;
				turnToAngle = navX.getYaw() - 6.0;
			}
			turnRobot(turnToAngle);
		} else if (newButton4D) { // programmed to turn 4 degrees right
			if (!pastButton4D) {
				lastOffYaw = 0;
				turnSum = 0;
				turnToAngle = navX.getYaw() + 6.0;
			}
			turnRobot(turnToAngle);
		}
		else if (newButton11D) {
			if (!pastButton11D) {		
				startYaw = navX.getYaw();
				if (startYaw < -100) startYaw = startYaw + 360;
					
				defenseStep = 1;
			}
			sallyPort();
		}
		else if (newButton12D) { // turns robot around 180 degrees
			if (!pastButton12D) {
				lastOffYaw = 0;
				turnSum = 0;
				turnToAngle = 179.8;
			}
			turnRobot(turnToAngle);
		}
		else if (newButton5D) {
			if(!pastButton5D) {
				startYaw=navX.getYaw();
				turnSum = 0;
				lastOffYaw = 0;
			}
			if(analyzePicture) {

				double deltaAngle = pictureAnalysis(140);
				SmartDashboard.putNumber("deltaAngle",deltaAngle);
				if (foundTarget) {
					if (deltaAngle > 0) {
						turnToAngle = startYaw - deltaAngle + 0.75;
						
					}
					else {
						turnToAngle = startYaw - deltaAngle - 0.75;
					}
				}
				else turnToAngle = startYaw;
			}
			System.out.println(turnToAngle);
			turnRobot(turnToAngle);
		}
		else {
			// Drive robot with one joystick
			double joyY = -driveStick.getY();
			double joyX = driveStick.getX();
			double powerLeft = joyY + joyX;
			double powerRight = joyY - joyX;

			if (driveStick.getTrigger()) {
				powerLeft = joyY;
				powerRight = joyY;
			}
			// double nowRoll = navX.getRoll();
			// if (nowRoll > maxRoll) maxRoll = nowRoll;
			// else if (nowRoll < minRoll) minRoll = nowRoll;

			driveRobot(powerLeft, powerRight);
			onTape = 0;
			analyzePicture=true;
		}
		if (teleopLoop % 25 == 0) {
			SmartDashboard.putNumber("powerA", powerA);
			SmartDashboard.putNumber("speedA", 60. / shootSpeedA.getPeriod());
			SmartDashboard.putNumber("powerB", powerB);
			SmartDashboard.putNumber("speedB", 60. / shootSpeedB.getPeriod());

			SmartDashboard.putBoolean("ballSensor", ballSensor.get());
			SmartDashboard.putBoolean("tapeSensor", tapeSensor.get());
			SmartDashboard.putNumber("armPot", armPot.getAverageVoltage());
			SmartDashboard.putNumber("shooterAngle", launchAngle.getVoltage());
			SmartDashboard.putNumber("distL", distanceL.getRaw());
			SmartDashboard.putNumber("distR", distanceR.getRaw());
			SmartDashboard.putBoolean("armLimit", armLimit.get());
			SmartDashboard.putBoolean("armUpLimit", armUpLimit.get());
			SmartDashboard.putNumber("yaw", navX.getYaw());
			SmartDashboard.putNumber("pitch", navX.getPitch());
			SmartDashboard.putNumber("roll", navX.getRoll());
			SmartDashboard.putBoolean("shooterOn", shooterOn);
//			SmartDashboard.putNumber("minRoll", minRoll);
//			SmartDashboard.putNumber("maxRoll", maxRoll);
//			SmartDashboard.putNumber("defenseDistance", defenseDist);
//			SmartDashboard.putNumber("sonar", moeSonar.getAverageVoltage());
			SmartDashboard.putBoolean("foundTarget", foundTarget);

		}

		pastButton4 = newButton4;
		pastButton2D = newButton2D;
		pastButton3D = newButton3D;
		pastButton4D = newButton4D;
		pastButton5D = newButton5D;
		pastButton6D = newButton6D;
		pastButton7D = newButton7D;
		pastButton8D = newButton8D;
		pastButton9D = newButton9D;
		pastButton10D = newButton10D;
		pastButton11D = newButton11D;
		pastButton12D = newButton12D;

	}

	public void autonomousInit() {
		autoLoop = 0;
		autoStep = 1;
		floorCount = 0;
		autoTimer.start();
		navX.zeroYaw();
		distanceL.reset();
		distanceR.reset();
		rollOffset = navX.getRoll();
		onTape = 0;
		analyzePicture = true;
		
//		autoRoutine=(int)SmartDashboard.getNumber("autoChoice");
//		if(autoRoutine==0)autoRoutine=autoFileChoice;
//		SmartDashboard.putNumber("ranRoutne", autoRoutine);
	}

	public void autonomousPeriodic() {
 
		autoLoop++;
		loopCount++;
		// autoTestRoutine();
//autoRoutine = 1;
		switch (autoRoutine) {
		case 1:
			lowBarAutoHigh();
			break;
		case 2:
			newRoughTerrain2High();
//			newWallRT2High();
			break;
		case 3:
//			roughTerrain3High();
			newRT3High();
			break;
		case 4:
//			roughTerrain4High();
			newRT4High();
			break;
		case 5:
			newRoughTerrain5High();
//			newWallRT5High();
			break;
		case 6:
			oldRoughTerrain2Low();
//			lowBarAutoLow();
			break;
		case 7:
			oldRoughTerrain5Low();
//			roughTerrain2Low();
//			newWallRT2Low();
			break;
		case 8:
			lowBarNoShoot();
//			roughTerrain5Low();
//			newWallRT5Low();
			break;
		case 9:
			roughTerrainNoShoot();
			break;
		case 10:
			autoTestRoutine();
			
//			outerWorks3();
//			lowBarFarShot();
//			roughTerrainNoShoot();
			
			break;
		case 11:
//			lowBarWallRoutine();
//		lowBarFarShot();
			break;
		case 12:
			oldRoughTerrain2High();
//			alternateRT2High();
			break;
		case 13:
			oldRoughTerrain5High();
//			alternateRT5High();
			break;
		
		default:
			break;
		}
		
		//serialize();
	}

	public void testInit() {
		testLoop = 0;
	}

	public void testPeriodic() {
		testLoop++;
		reverseWinch();
		 testTalons();
		// testShooterSpeeds();
	}

	public void pidWrite(double output) {
		double right = direction - output;
		double left = direction + output;
		driveRobot(left, right);
	}

	void driveRobot(double leftSide, double rightSide) {

		if (leftSide > 1)
			leftSide = 1.0;
		else if (leftSide < -1)
			leftSide = -1.0;
		if (rightSide > 1)
			rightSide = 1.0;
		else if (rightSide < -1)
			rightSide = -1.0;

		driveLA.set(leftSide);
		driveLB.set(leftSide);
		driveLC.set(leftSide);
		driveRA.set(rightSide);
		driveRB.set(rightSide);
		driveRC.set(rightSide);

	}

	void testTalons() {
		if (driveStick.getRawButton(5))
			driveLA.set(1.0);
		else if (driveStick.getRawButton(6))
			driveLA.set(-1.);
		else
			driveLA.set(0);
		if (driveStick.getRawButton(7))
			driveLB.set(1.0);
		else if (driveStick.getRawButton(8))
			driveLB.set(-1.0);
		else
			driveLB.set(0);
		if (driveStick.getRawButton(9))
			driveLC.set(1.0);
		else if (driveStick.getRawButton(10))
			driveLC.set(-1.0);
		else
			driveLC.set(0);
		// if (driveStick.getRawButton(11)) shootAngle.set(1.0);
		// else if (driveStick.getRawButton(12)) shootAngle.set(-1.0);
		// else shootAngle.set(0);

		if (funStick.getRawButton(5))
			driveRA.set(1.0);
		else if (funStick.getRawButton(6))
			driveRA.set(-1.0);
		else
			driveRA.set(0);
		if (funStick.getRawButton(7))
			driveRB.set(1.0);
		else if (funStick.getRawButton(8))
			driveRB.set(-1.0);
		else
			driveRB.set(0);
		if (driveStick.getRawButton(11))
			driveRC.set(1.0);
		else if (driveStick.getRawButton(12))
			driveRC.set(-1.0);
		else
			driveRC.set(0);
		// if (funStick.getRawButton(11)) arm.set(1.0);
		// else if (funStick.getRawButton(12)) arm.set(-1.0);
		// else arm.set(0);

	}

	void testShooterSpeeds() {
		// mainPower = SmartDashboard.getNumber("powerA");
		powerA = (driveStick.getRawAxis(2) + 1.0) / 2.0;
		shooterA.set(powerA);
		// secondPower = SmartDashboard.getNumber("powerB");
		powerB = (driveStick.getRawAxis(4) + 1.0) / 2.0;
		shooterB.set(-powerB);
		// shooterA.set(1.0);
		// shooterB.set(-1.0);
	}

	void controlShooter() {
		/*
		 * boolean newButton4 = funStick.getRawButton(4); if (newButton4 &&
		 * !pastButton4) { speedSumA = 0; speedSumB = 0; startLoop = teleopLoop;
		 * setSpeedA = SmartDashboard.getNumber("desiredSpeedA"); setSpeedB =
		 * SmartDashboard.getNumber("desiredSpeedB"); shooterOn = true; } else
		 * if (funStick.getRawButton(5)) { setSpeedA = 0; setSpeedB = 0;
		 * shooterOn = false; }
		 */
		double currentSpeedA = 60. / shootSpeedA.getPeriod();
		double currentSpeedB = 60. / shootSpeedB.getPeriod();

		if (shooterOn) {
			double startPowerA = setSpeedA / 4800.;
			double startPowerB = setSpeedB / 5200.;
			double offSpeedA = setSpeedA - currentSpeedA;
			double offSpeedB = setSpeedB - currentSpeedB;
			double KPA = 0;  //KPspeedA;
			double KPB = 0;  //KPspeedB;
			if (teleopLoop - startLoop > 75) {
				if (currentSpeedA < 100 || currentSpeedA > 5200) {
					offSpeedA = 0;
					speedSumA = 0;
					KPA = 0;
				} else if (currentSpeedA <= 5200) {
					
					if (offSpeedA > 50)
						speedSumA = speedSumA + .001;
					else if (offSpeedA < -50)
						speedSumA = speedSumA - .001;
//				if (Math.abs(offSpeedA) > 50) 
				// speedSumA = KIspeedA*offSpeedA + speedSumA;
				}
				if (currentSpeedB < 100 || currentSpeedB > 5200) {
					offSpeedB = 0;
					speedSumB = 0;
					KPB = 0;
				} else if (currentSpeedB <= 5200) {
					if (offSpeedB > 50)
						speedSumB = speedSumB + .001;
					else if (offSpeedB < -50)
						speedSumB = speedSumB - .001;
				
//					if (Math.abs(offSpeedB) > 50)
				// speedSumB = KIspeedB*offSpeedB + speedSumB;
				}
			}
			double newPowerA = startPowerA + KPA * offSpeedA + speedSumA;
			double newPowerB = startPowerB + KPB * offSpeedB + speedSumB;
			if (offSpeedA > 300) newPowerA = 1.0;
			if (offSpeedB > 300) newPowerB = 1.0;

			if (newPowerA > 1)
				newPowerA = 1.0;
			else if (newPowerA < 0)
				newPowerA = 0;
			if (newPowerB > 1)
				newPowerB = 1.0;
			else if (newPowerB < 0)
				newPowerB = 0;

			shooterA.set(newPowerA);
			shooterB.set(-newPowerB);

			powerA = newPowerA;
			powerB = newPowerB;
		} else {
			shooterA.set(0);
			shooterB.set(0);
			powerA = 0;
			powerB = 0;
		}
		// pastButton4 = newButton4;

	}
	
	boolean autoShooterSpeedControl() {
		double currentSpeedA = 60. / shootSpeedA.getPeriod();
		double currentSpeedB = 60. / shootSpeedB.getPeriod();
		setSpeedA = 4200.;
		setSpeedB = 4000.;
		
	

	//	if (shooterOn) {
			double startPowerA = setSpeedA / 4800.;
			double startPowerB = setSpeedB / 5200.;
			double offSpeedA = setSpeedA - currentSpeedA;
			double offSpeedB = setSpeedB - currentSpeedB;
			double KPA = 0;  //KPspeedA;
			double KPB = 0;  //KPspeedB;
			if (loopCount > 75) {
				if (currentSpeedA < 100 || currentSpeedA > 5200) {
					offSpeedA = 0;
					speedSumA = 0;
					KPA = 0;
				} else if (currentSpeedA <= 5200) {
					if (offSpeedA > 50)
						speedSumA = speedSumA + .001;
					else if (offSpeedA < -50)
						speedSumA = speedSumA - .001;
					
				// speedSumA = KIspeedA*offSpeedA + speedSumA;
				}
				if (currentSpeedB < 100 || currentSpeedB > 5200) {
					offSpeedB = 0;
					speedSumB = 0;
					KPB = 0;
				} else if (currentSpeedB <= 5200) {
					if (offSpeedB > 50)
						speedSumB = speedSumB + .001;
					else if (offSpeedB < -50)
						speedSumB = speedSumB - .001;
				}

				// speedSumB = KIspeedB*offSpeedB + speedSumB;
			}
			double newPowerA = startPowerA + KPA * offSpeedA + speedSumA;
			double newPowerB = startPowerB + KPB * offSpeedB + speedSumB;

			if (newPowerA > 1)
				newPowerA = 1.0;
			else if (newPowerA < 0)
				newPowerA = 0;
			if (newPowerB > 1)
				newPowerB = 1.0;
			else if (newPowerB < 0)
				newPowerB = 0;

			shooterA.set(newPowerA);
			shooterB.set(-newPowerB);
			
			if (Math.abs(currentSpeedA - 4200.) < 50.0) {
				return true;
			}
			
			else {
				return false;
			}

	//		powerA = newPowerA;
	//		powerB = newPowerB;
		
	}

	void controlCollector() {
		if (funStick.getRawButton(2)) { // run collector to bring balls in
			collector.set(0.8);
			if (ballSensor.get()) {
				ballControl.set(1.0); // use 0.5 on real bot
			} else
				ballControl.set(0);
		} else if (funStick.getRawButton(3)) { // run collector to push balls out
			if (funStick.getTrigger()) {
				collector.set(-1.0);
				ballControl.set(0);
			}
			else {
				collector.set(-1.0);
				ballControl.set(-1.0);
			}
		} else {
			collector.set(0);
			ballControl.set(0);
		}
	}

	void controlArm() {
		double moveArm = -funStick.getY(); // use joystick to move arm up and
											// down
		if (moveArm > 0) { // check for bottom limit switch
			if (armLimit.get()) {
				arm.set(0);
			} else
				arm.set(moveArm);
		}
		/*
		else if (moveArm < 0) { //don't go past climbing position
			if (armUpLimit.get()&&driveStick.getRawButton(1)) {
				arm.set(0);
			}
			else arm.set(moveArm);
		}
		*/
		else
			arm.set(moveArm);
	}

	void controlShooterAngle() {
//		if (funStick.getRawButton(11))
//			setShooterAngle(shooterClose);
//		else if (funStick.getRawButton(10))
//			setShooterAngle(shooterOuterWorks);

//		else 
			if (funStick.getRawButton(6)) { // want steeper shoot angle
		// if (launchAngle.getAverageVoltage() < 2.3) shootAngle.set(0);
		// else
			shootAngle.set(-0.5);
		} else if (funStick.getRawButton(7)) { // want shallower shoot angle
		// if (launchAngle.getAverageVoltage() > 4.0) shootAngle.set(0);
		// else
			shootAngle.set(0.5);
		} else
			shootAngle.set(0);
	}

	void setShooterAngle(double setAngle) {
		double currentAngle = launchAngle.getAverageVoltage();

		if (currentAngle - setAngle > 0.2) {
			// if (currentAngle < 2.3) shootAngle.set(0);
			// else
			shootAngle.set(-0.8);
		} else if (currentAngle - setAngle > 0.02) {
			// if (currentAngle < 2.3) shootAngle.set(0);
			// else
			shootAngle.set(-0.35);
		} else if (currentAngle - setAngle < -0.2) {
			// if (currentAngle > 4.0) shootAngle.set(0);
			// else
			shootAngle.set(0.8);
		} else if (currentAngle - setAngle < -0.02) {
			// if (currentAngle > 4.0) shootAngle.set(0);
			// else
			shootAngle.set(0.35);
		} else
			shootAngle.set(0.0);
	}

	void controlScalingArms() {
		if (funStick.getRawButton(8)&&(!tapeSensor.get()||funStick.getRawButton(1))){
			scaleL.set(0);
			scaleR.set(-1.0);
		} else if (funStick.getRawButton(9)) { // lift robot
			scaleL.set(-1.0);
			scaleR.set(0);
		} else {
			scaleL.set(0);
			scaleR.set(0);
		}
	}

	void reverseWinch() {
		if (funStick.getRawButton(10)) {
			scaleL.set(0.4);
		}
		else if(funStick.getRawButton(11)){
			scaleL.set(-0.4);
		}else
			scaleL.set(0);
	}

	boolean turnRobot(double setBearing) {
		double currentYaw = navX.getYaw();
		if (setBearing > 160.0 && currentYaw < 0)
			currentYaw = currentYaw + 360;
		if (setBearing < -160.0) {
			setBearing = setBearing + 360.;
			if (currentYaw < 0) 
				currentYaw = currentYaw + 360;
		}
		// if (currentYaw < 0) currentYaw = currentYaw + 360.0;
		double offYaw = setBearing - currentYaw;

		if (offYaw * lastOffYaw <= 0) {
			if (offYaw > 0) turnSum = 0.0;
			else if (offYaw < 0) turnSum = 0.0;
			else turnSum = 0;
		}
		if (offYaw > 1.0 || offYaw < -1.0) {

			if (offYaw < 20 && offYaw > -20) {
				if (offYaw > 0)
					turnSum = turnSum + .01;
				else
					turnSum = turnSum - .01;
			}

			double newPower = .017 * offYaw + turnSum;

			if (Math.abs(offYaw) > 150.)
				maxTurnPower = 0.5;
			else
				maxTurnPower = 0.45;
			if (newPower > maxTurnPower)
				newPower = maxTurnPower;
			else if (newPower < -maxTurnPower)
				newPower = -maxTurnPower;

			driveRobot(newPower, -newPower);
			lastOffYaw = offYaw;
			return false;
		} else {
			driveRobot(0, 0);
			lastOffYaw = offYaw;
			return true;
		}

	}
	
	void overDefenses() {
		loopCount++;
		double currentRoll = navX.getRoll();
		if (currentRoll > maxRoll)
			maxRoll = currentRoll;
		else if (currentRoll < minRoll)
			minRoll = currentRoll;
		switch(defenseStep) {
		case 1:
			double startP = loopCount*.06;
			if (startP < direction) {
				driveRobot(startP,startP);
			}
			else {
				driveStraight.setSetpoint(startYaw);
				driveStraight.enable();
				defenseStep = 2;
			}
			break;
		case 2:
			if (currentRoll >=12) {
				distanceL.reset();
				distanceR.reset();
				defenseDist = 0;
				defenseStep = 3;
			}
			break;
		case 3:
			if (currentRoll < -17.0 ||( currentRoll < -11.0 && (distanceR.getRaw() > 2300 || distanceL.getRaw() > 2300))) {
//				if (defenseDist < 5.0)
					defenseDist = distanceR.getRaw();

				direction = 0.4;
				defenseStep = 4;
			}
			break;
		case 4:
			if (currentRoll > (rollOffset - 0.7)) {
				floorCount++;
				if (floorCount == 2) {
					defenseStep = 5;
					distanceL.reset();
					distanceR.reset();
				}
			}
			break;
		case 5:
			if (!isAutonomous()) {
				if (driveStraight.isEnabled())
					driveStraight.disable();
				pidContOn = false;
				driveRobot(0,0);
			}
			break;
			default:
				driveRobot(0,0);
		}
	}
	
	void sallyPort() {
		double currentYaw = navX.getYaw();
		if (currentYaw < -100) currentYaw = currentYaw + 360;
		switch(defenseStep) {
		case 1: 
			if (currentYaw < (startYaw - 30)) {
				defenseStep = 2;
				driveRobot(0,0);
			}
			else
				driveRobot(-0.6,0.6);
			break;
		case 2:
			defenseStep = 3;
			break;
		case 3:
			if (currentYaw > (startYaw + 15)) {
				defenseStep = 4;
				driveRobot(0,0);
			}
			else 
				driveRobot(0.6,-0.6);
			break;
		case 4:
			defenseStep = 5;
			break;
		case 5:
			if (currentYaw < (startYaw + 7)) {
				defenseStep = 6;
				driveRobot(0,0);
			}
			else
				driveRobot(-0.45,0.45);
			break;
		case 6:
			driveRobot(0,0);
		}
	}
	
	boolean backUpFromBatter(int backDist) {
		if (!tapeSensor.get()) {
			//			onTape = 1;
			driveRobot(-0.4, -0.4);
			return false;
		}
		else if (onTape < 4) {
			onTape++;
			driveRobot(-0.4,-0.4);
			return false;
		}
		else if (onTape == 4) {
			distanceL.reset();
			distanceR.reset();
			onTape = 5;
			driveRobot(-0.4,-0.4);
			return false;

		}
		else if (distanceR.getRaw() < -backDist) {
			driveRobot(0, 0);
			return true;
		} 
		else {
			driveRobot(-0.4, -0.4);
			return false;
		}
		//				onTape = false;
	}

	boolean moveArmUp(double setPointUp) {
		double currentArm = armPot.getAverageVoltage();
		if (setPointUp > currentArm) {
			arm.set(-1.0);
			return false;
		} else {
			arm.set(0);
			return true;
		}
	}

	boolean moveArmDown() {
		if (armLimit.get()) {
			arm.set(0);
			return true;
//		} else if (armPot.getAverageVoltage() < 2.15) {
//			arm.set(0.75);
//			return false;
		} else {
			arm.set(1.0);
			return false;
		}
	}
	
	double pictureAnalysis(double xTarget)
	{
		//return gripAnalysis(xTarget);
		return udpAnalysisX(xTarget);
		
	}
	
	void pictureAnalysisDual(double x, double y){
		udpAnalysisX(x);
		udpAnalysisY(y);
	}
	
	static double picutreAnalyzeX;
	static double pictureAnalyzeY;
	void updAnalysis(double xTarget, double yTarget){
		udpAnalysisX=udpAnalysisX(xTarget);
		udpAnalysisY=udpAnalysisY(yTarget);
	}
	double udpAnalysisY(double yTarget){
		final int scalingFactor=1;
		Box box=udpGetBiggestBox();
		if(box!=null){
			//analyzePicture=false;
			//foundTarget=true;
			return (yTarget-box.getY())/scalingFactor;
		}else{
			//foundTarget=false;
			return 0;
		}
	}
	double udpAnalysisX(double xTarget)//640x480
	{
		final int scalingFactor=5;
		Box box=udpGetBiggestBox();
		if(box!=null){
			analyzePicture=false;
			foundTarget = true;
			return (xTarget-box.getX())/scalingFactor;
		}else{
			foundTarget=false;
			return 0;
		}
	}
	Box udpGetBiggestBox()
	{
		Box[]boxes=udpVision.getData();
		double pastSize=-1;int index=-1;
		for(int n=0;n<boxes.length;n++)
		{
			if(boxes[n]!=null)
			{
				if(pastSize<boxes[n].getS())
				{
					pastSize=boxes[n].getS();
					index=n;
				}
			}
		}
		return index>0?boxes[index]:null;
	}
	
	double gripAnalysis(double xTarget)
	{
		ITable it=table.getSubTable("myContoursReport");
		double []areas=it.getNumberArray("area", new double[]{-1});
		double []xVals=it.getNumberArray("centerX",new double[]{});
		double pastArea = -1;
		int indx = -1;
		//find greatest area
		if(areas.length>0)
		{
			for (int n=0;n<areas.length;n++) 
			{
				if (pastArea < areas[n])
				{
					pastArea = areas[n];
					indx = n;
				}
			}
		}
		//if the area is greater than zero, a target was found
		if (pastArea > 0)
		{
			analyzePicture=false;
			foundTarget = true;
			return (xTarget-xVals[indx])/5;
		}
		foundTarget = false;
		return 0;
	}
	
	public int getAuto()
	{
		int choice=0;
		choice=(int)SmartDashboard.getNumber("autoChoice");
		if(choice==NAK) choice = autoFile.readAutoFile();
		return choice;
	}
	void autoTestRoutine() {
		double aimPixels = 130.;
	
		switch (autoStep) {
		case 1:
			double power = loopCount * .05;
			if (power > 0.5) {
				driveStraight.setSetpoint(0);
				direction = 0.5;
				driveStraight.enable();
				autoStep = 2;
				distanceL.reset();
				distanceR.reset();
			} else
				driveRobot(power, power);
			break;
		case 2:
			if (distanceL.getRaw() > 4600.0) {
				autoStep = 3;
				driveStraight.disable();
				loopCount = 0;
				driveRobot(0, 0);
				autoTimer.reset();
				turnSum = 0;
				turnCount = 0;
				lastOffYaw = 0;
			}
			break;
		case 3:
			if (autoTimer.get() > 0.5) {
				if(analyzePicture) {
					double deltaAngle = pictureAnalysis(aimPixels);
					if (foundTarget) {
						if (deltaAngle > 0) turnToAngle = navX.getYaw() + deltaAngle + 0.5;
						else turnToAngle = navX.getYaw() + deltaAngle - 0.5;
					}
					else turnToAngle = navX.getYaw();
				}

				if(turnRobot(turnToAngle))
				{
					turnCount++;
					if(turnCount>4) {
						autoStep = 5;
						loopCount = 0;
						autoTimer.reset();
					}
				}
			}
				break;
		case 4:
			if (angleController.onTarget()) {
				autoStep = 5;
				angleController.disable();
			}
			break;
		case 5:
			driveRobot(0, 0);

		}

	}
	
	void lowBarAutoHigh() {
		double aimPixels = 140;
		double aimAngle = 3.53;
		switch (autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(aimAngle);
			 if (autoTimer.get() > 1.0) {
			//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
			}
			break;
		case 2:
			moveArmDown();
			setShooterAngle(aimAngle);
			collector.set(1.0);
			double power = loopCount * .05;
			if (power > 0.5) {
				driveStraight.setPID(.04, 0.00005, .03);
				driveStraight.setSetpoint(0);
				direction = 0.5;
				shootAngle.set(0);
				driveStraight.enable();
				autoStep = 3;
			} else
				driveRobot(power, power);
			break;
		case 3:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > 9) {
				autoStep = 4;
			}
			double currentRoll = navX.getRoll();
			if (currentRoll > maxRoll)
				maxRoll = currentRoll;
			else if (currentRoll < minRoll)
				minRoll = currentRoll;
			break;
		case 4:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() < -8) {
				autoStep = 5;
				direction = 0.35;
			}
			double nowRoll = navX.getRoll();
			if (nowRoll > maxRoll)
				maxRoll = nowRoll;
			else if (nowRoll < minRoll)
				minRoll = nowRoll;
			break;
		case 5:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > rollOffset - 0.7) {
				floorCount++;
				if (floorCount == 2) {
					distanceL.reset();
					distanceR.reset();
					direction = 0.45;
					autoStep = 6;
					autoTimer.reset();
				}
			}
			break;
		case 6:
			if (autoTimer.get() > 1.5) arm.set(0);
			else	arm.set(-1);
			collector.set(0);
			if (distanceR.getRaw() > 9300) {
				driveStraight.disable();
				turnSum = 0;
				lastOffYaw = 0;
				autoTimer.reset();
				arm.set(0);
				autoStep = 7;
				loopCount = 0;
			}
			break;
		case 7:
			//moveArmUp(3.0);
			setShooterAngle(aimAngle);
			if (loopCount == 10) {
				// double sonarDist = moeSonar.getAverageVoltage();
				// double newAngle = sonarDist/0.15 + 0.2;
				// turnToAngle = 90.0 - Math.atan(newAngle/12.5);
				turnToAngle = 60.;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 6) {
						autoStep = 8;
						turnSum = 0;
						turnCount = 0;
						loopCount = 0;
						lastOffYaw = 0;
						autoTimer.reset();
						arm.set(0);
						shootAngle.set(0);
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 8:
			onSpeed = autoShooterSpeedControl();
			if (onSpeed && autoTimer.get()>2.0) {
				autoStep = 9;
			}
			else if (autoTimer.get() > 4.0) autoStep = 9;
	//		shooterA.set(0.87);
	//		shooterB.set(-0.76);
	//		if (autoTimer.get() > 2.0) {
	//			autoStep = 9;
	//		}
			driveRobot(0, 0);
			break;
		case 9:
			autoShooterSpeedControl();
			if (autoTimer.get() > 2.5) {
			ballControl.set(1.0);
			}
			if (autoTimer.get() > 7.0) {
				autoStep = 10;
			}
			driveRobot(0, 0);
			break;
		case 10:

			ballControl.set(0);
			shooterA.set(0);
			shooterB.set(0);
			autoStep = 11;
			driveRobot(0, 0);
			break;
		case 11:
			driveRobot(0, 0);
			break;
		case 12:
			if (autoTimer.get() > 1.0) {
				if(analyzePicture) {
					double deltaAngle = pictureAnalysis(aimPixels);
					if (foundTarget) {
						if (deltaAngle > 0) turnToAngle = navX.getYaw() - deltaAngle + 0.5;
						else turnToAngle = navX.getYaw() - deltaAngle - 0.5;
					}
					else turnToAngle = navX.getYaw();
				}

				if(turnRobot(turnToAngle))
				{
					turnCount++;
					if(turnCount>4) {
						autoStep = 8;
						loopCount = 0;
						autoTimer.reset();
					}
				}
				
			}
			else driveRobot(0,0);
			break;
		}
	}
	
	void newRoughTerrain2High() {
		double aimPixels = 135;
		double aimAngle = 3.3;
		switch(autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(aimAngle);
			if (autoTimer.get() > 1.0) {
				//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
				defenseStep = 1;
				if (moat > 0.5) direction = 0.7;
				else direction = 0.6;
				startYaw = 0;			
				loopCount = 0;				
				floorCount = 0;
				minRoll = 0;
				maxRoll = 0;
			}
			break;
		case 2:
			moveArmDown();
			setShooterAngle(aimAngle);
			collector.set(1.0);
			overDefenses();
			if (defenseStep == 5) {
				autoStep = 3;
				direction = 0.5;
			}
			break;
		case 3:
			collector.set(0.0);
			if (distanceR.getRaw() > 12850) {
				driveStraight.disable();
				turnSum = 0;
				lastOffYaw = 0;
				autoStep = 4;
				loopCount = 0;
				arm.set(0);
				driveRobot(0, 0);
			}
			break;
		case 4:
			setShooterAngle(aimAngle);
			if (loopCount == 10) {
				// double sonarDist = moeSonar.getAverageVoltage();
				// double newAngle = sonarDist/0.15 + 0.2;
				// turnToAngle = 90.0 - Math.atan(newAngle/12.5);
				turnToAngle = 61.0;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 2) {
						autoStep = 9;
						loopCount = 0;
						autoTimer.reset();
						shootAngle.set(0);
						turnSum = 0;
						turnCount = 0;
						lastOffYaw = 0;
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 5:
			//		shooterA.set(0.87);
			//		shooterB.set(-0.76);
			onSpeed = autoShooterSpeedControl();
			if (onSpeed && autoTimer.get()>2.0 )   {
				//		if (autoTimer.get() > 2.0) {
				autoStep = 6;
			}
			else if (autoTimer.get()>4.0) autoStep = 6;
			driveRobot(0, 0);
			break;
		case 6:
			autoShooterSpeedControl();
			ballControl.set(1.0);
			if (autoTimer.get() > 7.0) {
				autoStep = 7;
			}
			driveRobot(0, 0);
			break;
		case 7:

			ballControl.set(0);
			shooterA.set(0);
			shooterB.set(0);
			autoStep = 8;
			driveRobot(0, 0);
			break;
		case 8:
			driveRobot(0, 0);
break;
		case 9:
			if (autoTimer.get() > 1.0) {
				if(analyzePicture) {
					double deltaAngle = pictureAnalysis(aimPixels);
					if (foundTarget) {
						if (deltaAngle > 0) turnToAngle = navX.getYaw() - deltaAngle + 0.75;
						else turnToAngle = navX.getYaw() - deltaAngle - 0.75;
					}
					else turnToAngle = navX.getYaw();
					analyzePicture = false;
				}

				if(turnRobot(turnToAngle))
				{
					turnCount++;
					if(turnCount>4) {
						autoStep = 5;
						loopCount = 0;
						autoTimer.reset();
					}
				}
				
			}
			else driveRobot(0,0);
			break;
		}
		
			
		

	}
	
	void newRT3High() {
		double aimPixels = 145.;
		double aimAngle = 3.59;
		switch(autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(aimAngle);
			if (autoTimer.get() > 1.0) {
				//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
				defenseStep = 1;
				if (moat > 0.5) direction = 0.7;
				else direction = 0.6;
				startYaw = 0;			
				loopCount = 0;				
				floorCount = 0;
				minRoll = 0;
				maxRoll = 0;
			}
			break;
		case 2:
			moveArmDown();
			setShooterAngle(aimAngle);
			collector.set(1.0);
			overDefenses();
			if (defenseStep == 5) {
				autoStep = 3;
				direction = 0.5;
			}
			break;
		case 3:
			collector.set(0.0);
			if (distanceR.getRaw() > 6600) {
				driveStraight.disable();
				turnSum = 0;
				lastOffYaw = 0;
				autoStep = 4;
				loopCount = 0;
				arm.set(0);
				driveRobot(0, 0);
			}
			break;
		case 4:
			setShooterAngle(aimAngle);
			if (loopCount == 10) {
				// double sonarDist = moeSonar.getAverageVoltage();
				// double newAngle = sonarDist/0.15 + 0.2;
				// turnToAngle = 90.0 - Math.atan(newAngle/12.5);
				turnToAngle = 25.5;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 2) {
						autoStep = 9;
						loopCount = 0;
						autoTimer.reset();
						shootAngle.set(0);
						turnCount = 0;
						turnSum = 0;
						lastOffYaw = 0;
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 5:
			//		shooterA.set(0.87);
			//		shooterB.set(-0.76);
			onSpeed = autoShooterSpeedControl();
			if (onSpeed && autoTimer.get()>2.0) {
				//		if (autoTimer.get() > 2.0) {
				autoStep = 6;
			}
			else if (autoTimer.get()>4.0) autoStep = 6;
			driveRobot(0, 0);
			break;
		case 6:
			autoShooterSpeedControl();
			ballControl.set(1.0);
			if (autoTimer.get() > 7.0) {
				autoStep = 7;
			}
			driveRobot(0, 0);
			break;
		case 7:

			ballControl.set(0);
			shooterA.set(0);
			shooterB.set(0);
			autoStep = 8;
			driveRobot(0, 0);
			break;
		case 8:
			driveRobot(0, 0);
			break;
		case 9:
			if (autoTimer.get() > 1.0) {
				if(analyzePicture) {
					double deltaAngle = pictureAnalysis(aimPixels);
					if (foundTarget) {
						if (deltaAngle > 0) turnToAngle = navX.getYaw() - deltaAngle + 0.75;
						else turnToAngle = navX.getYaw() - deltaAngle - 0.75;
					}
					else turnToAngle = navX.getYaw();
				}

				if(turnRobot(turnToAngle))
				{
					turnCount++;
					if(turnCount>4) {
						autoStep = 5;
						loopCount = 0;
						autoTimer.reset();
					}
				}
				
			}
			else driveRobot(0,0);
			break;
		}

	}

	void alternateRT3High() {
		double aimAngle = 2.94;
		switch (autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(aimAngle);
			 if (autoTimer.get() > 2.5) {
		//	if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
			}
			break;
		case 2:
			moveArmDown();
			setShooterAngle(aimAngle);
			double power = loopCount * .05;
			if (power > 0.6) {
				driveStraight.setPID(.04, 0.00005, .03);
				driveStraight.setSetpoint(0);
				direction = 0.6;
				driveStraight.enable();
				shootAngle.set(0);
				autoStep = 3;
			} else
				driveRobot(power, power);
			break;
		case 3:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > 12) {
				autoStep = 4;
			}
			double currentRoll = navX.getRoll();
			if (currentRoll > maxRoll)
				maxRoll = currentRoll;
			else if (currentRoll < minRoll)
				minRoll = currentRoll;
			break;
		case 4:
			moveArmDown();
			collector.set(1.0);
			
			double nowRoll = navX.getRoll();
			if (nowRoll > maxRoll)
				maxRoll = nowRoll;
			else if (nowRoll < minRoll)
				minRoll = nowRoll;
			if (nowRoll < -17.0 ||( nowRoll < -12.0 && (distanceR.getRaw() > 2300 || distanceL.getRaw() > 2300))) {	
//			if (navX.getRoll() < -10) {
				autoStep = 5;
				direction = 0.35;
			}
			break;
		case 5:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > rollOffset - 0.7) {
				floorCount++;
				if (floorCount == 1) {
					driveStraight.disable();
					turnSum = 0;
					lastOffYaw = 0;
					autoStep = 6;
					loopCount = 0;
					distanceL.reset();
					distanceR.reset();
					driveRobot(0, 0);
				}
			}

			break;
		case 6:
			collector.set(0.0);
			setShooterAngle(aimAngle);
			arm.set(0);
			if (loopCount == 10) {
				turnToAngle = 25.0;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 7) {
						autoStep = 7;
						loopCount = 0;
						distanceL.reset();
						distanceR.reset();
						direction = 0.5;
						driveStraight.setSetpoint(25.0);
						driveStraight.enable();
						shootAngle.set(0);
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 7:
			if (distanceR.getRaw() > 10000) {
				driveStraight.disable();
				turnSum = 0;
				lastOffYaw = 0;
				autoStep = 8;
				loopCount = 0;
				driveRobot(0, 0);
			}
			break;
		case 8:
			setShooterAngle(2.94);
			if (loopCount == 10) {
				turnToAngle = 0.0;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 7) {
						autoStep = 9;
						loopCount = 0;
						autoTimer.reset();
						shootAngle.set(0);
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 9:
			onSpeed = autoShooterSpeedControl();
			if (onSpeed || autoTimer.get() > 4.0) {
//			shooterA.set(0.87);
//			shooterB.set(-0.76);
//			if (autoTimer.get() > 2.0) {
				autoStep = 10;
			}
			driveRobot(0, 0);
			break;
		case 10:
			autoShooterSpeedControl();
			ballControl.set(1.0);
			if (autoTimer.get() > 6.0) {
				autoStep = 11;
			}
			driveRobot(0, 0);
			break;
		case 11:

			ballControl.set(0);
			shooterA.set(0);
			shooterB.set(0);
			autoStep = 12;
			driveRobot(0, 0);
			break;
		case 12:
			driveRobot(0, 0);
		}

	}


	void newRT4High() {
		double aimPixels = 135.;
		double aimAngle = 3.35;
		switch(autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(aimAngle);
			if (autoTimer.get() > 1.0) {
				//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
				defenseStep = 1;
				if (moat > 0.5) direction = 0.7;
				else direction = 0.6;
				startYaw = 0;			
				loopCount = 0;				
				floorCount = 0;
				minRoll = 0;
				maxRoll = 0;
			}
			break;
		case 2:
			moveArmDown();
			setShooterAngle(aimAngle);
			collector.set(1.0);
			overDefenses();
			if (defenseStep == 5) {
				autoStep = 3;
				direction = 0.5;
			}
			break;
		case 3:
			collector.set(0.0);
			if (distanceR.getRaw() > 8400) {
				driveStraight.disable();
				turnSum = 0;
				lastOffYaw = 0;
				autoStep = 4;
				loopCount = 0;
				arm.set(0);
				driveRobot(0, 0);
			}
			break;
		case 4:
			setShooterAngle(aimAngle);
			if (loopCount == 10) {
				// double sonarDist = moeSonar.getAverageVoltage();
				// double newAngle = sonarDist/0.15 + 0.2;
				// turnToAngle = 90.0 - Math.atan(newAngle/12.5);
				turnToAngle = -14.0;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 2) {
						autoStep = 9;
						loopCount = 0;
						autoTimer.reset();
						shootAngle.set(0);
						turnCount = 0;
						turnSum = 0;
						lastOffYaw = 0;
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 5:
			//		shooterA.set(0.87);
			//		shooterB.set(-0.76);
			onSpeed = autoShooterSpeedControl();
			if (onSpeed && autoTimer.get()>2.0) {
				//		if (autoTimer.get() > 2.0) {
				autoStep = 6;
			}
			else if (autoTimer.get()>4.0) autoStep = 6;
			driveRobot(0, 0);
			break;
		case 6:
			autoShooterSpeedControl();
			if (autoTimer.get() > 2.5) {
			ballControl.set(1.0);
			}
			if (autoTimer.get() > 7.0) {
				autoStep = 7;
			}
			driveRobot(0, 0);
			break;
		case 7:

			ballControl.set(0);
			shooterA.set(0);
			shooterB.set(0);
			autoStep = 8;
			driveRobot(0, 0);
			break;
		case 8:
			driveRobot(0, 0);
			break;
		case 9:
			if (autoTimer.get() > 1.0) {
				if(analyzePicture) {
					double deltaAngle = pictureAnalysis(aimPixels);
					if (foundTarget) {
						if (deltaAngle > 0) turnToAngle = navX.getYaw() - deltaAngle + 0.75;
						else turnToAngle = navX.getYaw() - deltaAngle - 0.75;
					}
					else turnToAngle = navX.getYaw();
				}

				if(turnRobot(turnToAngle))
				{
					turnCount++;
					if(turnCount>4) {
						autoStep = 5;
						loopCount = 0;
						autoTimer.reset();
					}
				}
				
			}
			else driveRobot(0,0);
			break;
		}

	}
	
	void newRoughTerrain5High() {
		double aimPixels = 135.;
		double aimAngle = 2.83;
		switch(autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(aimAngle);
			if (autoTimer.get() > 1.0) {
				//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
				defenseStep = 1;
				if (moat > 0.5) direction = 0.7;
				else direction = 0.6;
				startYaw = 0;			
				loopCount = 0;				
				floorCount = 0;
				minRoll = 0;
				maxRoll = 0;
			}
			break;
		case 2:
			moveArmDown();
			setShooterAngle(aimAngle);
			collector.set(1.0);
			overDefenses();
			if (defenseStep == 5) {
				autoStep = 3;
				direction = 0.5;
			}
			break;
		case 3:
			collector.set(0.0);
			if (distanceR.getRaw() > 14900) {
				driveStraight.disable();
				turnSum = 0;
				lastOffYaw = 0;
				autoStep = 4;
				loopCount = 0;
				arm.set(0);
				driveRobot(0, 0);
			}
			break;
		case 4:
			setShooterAngle(aimAngle);
			if (loopCount == 10) {
				// double sonarDist = moeSonar.getAverageVoltage();
				// double newAngle = sonarDist/0.15 + 0.2;
				// turnToAngle = 90.0 - Math.atan(newAngle/12.5);
				turnToAngle = -61.0;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 7) {
						autoStep = 9;
						loopCount = 0;
						autoTimer.reset();
						shootAngle.set(0);
						lastOffYaw = 0;
						turnSum = 0;
						turnCount = 0;
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 5:
			//		shooterA.set(0.87);
			//		shooterB.set(-0.76);
			onSpeed = autoShooterSpeedControl();
			if (onSpeed && autoTimer.get()>2.0) {
				//		if (autoTimer.get() > 2.0) {
				autoStep = 6;
			}
			else if (autoTimer.get()>4.0) autoStep = 6;
			driveRobot(0, 0);
			break;
		case 6:
			autoShooterSpeedControl();
			ballControl.set(1.0);
			if (autoTimer.get() > 7.0) {
				autoStep = 7;
			}
			driveRobot(0, 0);
			break;
		case 7:

			ballControl.set(0);
			shooterA.set(0);
			shooterB.set(0);
			autoStep = 8;
			driveRobot(0, 0);
			break;
		case 8:
			driveRobot(0, 0);
			break;
		case 9:
			if (autoTimer.get() > 1.0) {
				if(analyzePicture) {
					double deltaAngle = pictureAnalysis(aimPixels);
					if (foundTarget) {
						if (deltaAngle > 0) turnToAngle = navX.getYaw() - deltaAngle + 0.75;
						else turnToAngle = navX.getYaw() - deltaAngle - 0.75;
					}
					else turnToAngle = navX.getYaw();
				}

				if(turnRobot(turnToAngle))
				{
					turnCount++;
					if(turnCount>4) {
						autoStep = 5;
						loopCount = 0;
						autoTimer.reset();
					}
				}
				
			}
			else driveRobot(0,0);
			break;
		}

	}

	void lowBarAutoLow() {
		switch (autoStep) {
		case 1:
			moveArmDown();
			if (autoTimer.get() > 2.5) {
			//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
			}
			break;
		case 2:
			moveArmDown();
			collector.set(1.0);
			double power = loopCount * .05;
			if (power > 0.5) {
				driveStraight.setPID(.04, 0.00005, .03);
				driveStraight.setSetpoint(0);
				direction = 0.5;
				driveStraight.enable();
				autoStep = 3;
			} else
				driveRobot(power, power);
			break;
		case 3:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > 9) {
				autoStep = 4;
			}
			double currentRoll = navX.getRoll();
			if (currentRoll > maxRoll)
				maxRoll = currentRoll;
			else if (currentRoll < minRoll)
				minRoll = currentRoll;
			break;
		case 4:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() < -8) {
				autoStep = 5;
				direction = 0.35;
			}
			double nowRoll = navX.getRoll();
			if (nowRoll > maxRoll)
				maxRoll = nowRoll;
			else if (nowRoll < minRoll)
				minRoll = nowRoll;
			break;
		case 5:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > rollOffset - 0.6) {
				floorCount++;
				if (floorCount == 2) {
					distanceL.reset();
					distanceR.reset();
					direction = 0.5;
					autoStep = 6;
				}
			}
			break;
		case 6:
			// moveArmUp(armCheval);
			collector.set(0);
			if (distanceR.getRaw() > 10000) {
				driveStraight.disable();
				turnSum = 0;
				lastOffYaw = 0;
				autoStep = 7;
				loopCount = 0;
			}
			break;
		case 7:
			// moveArmUp(armCheval);
			if (loopCount == 10) {
				// double sonarDist = moeSonar.getAverageVoltage();
				// double newAngle = sonarDist/0.15 + 0.2;
				// turnToAngle = 90.0 - Math.atan(newAngle/12.5);
				turnToAngle = -120.0;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 7) {
						autoStep = 8;
						// turnSum = 0;
						distanceR.reset();
						distanceL.reset();
						driveStraight.setPID(.04, 0.00005, .03);
						driveStraight.setSetpoint(0);
						direction = -0.7;
						driveStraight.enable();
						loopCount = 0;
						arm.set(0);
						autoTimer.reset();
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 8:
			// if (navX.getRoll() < -7.5) {
			if (distanceR.getRaw() < -15000) {
				autoStep = 10;
				loopCount = 0;
				driveStraight.disable();
				driveRobot(0, 0);
				// distanceR.reset();
				// distanceL.reset();
				autoTimer.reset();
				// driveRobot(0,0);
			}
			// else driveRobot(-0.7,-0.7);
			break;
		case 9:
			if (distanceR.getRaw() < -3000) {
				// if (autoTimer.get() > 1.0 ) {
				autoStep = 10;
				loopCount = 0;
			}
			break;
		case 10:
			collector.set(-1.0);
			ballControl.set(-1.0);
			driveRobot(0, 0);
			if (loopCount == 100) {
				autoStep = 11;
				loopCount = 0;
			}
			break;
		case 11:
			collector.set(0);
			ballControl.set(0);
			driveRobot(0, 0);
			break;
		case 12:
			driveRobot(0, 0);

		}
	}
	
	void newWallRT2Low() {
		switch (autoStep) {
		case 1:
			moveArmDown();
//			setShooterAngle(2.9);
			 if (autoTimer.get() > 2.5) {
			//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
				defenseStep = 1;
				if (moat > 0.5) direction = 0.7;
				else direction = 0.6;
				startYaw = 0;			
				loopCount = 0;				
				floorCount = 0;
				minRoll = 0;
				maxRoll = 0;
			}
			break;
		case 2:
			moveArmDown();
//			setShooterAngle(2.9);
			collector.set(1.0);
			overDefenses();
			if (defenseStep == 5) {
				autoStep = 3;
				direction = 0.5;
			}
			break;
		case 3:
			collector.set(0.0);
			if (distanceR.getRaw() > 15000) {
				driveStraight.disable();
//				turnSum = 0;
				autoStep = 4;
				loopCount = 0;
				arm.set(0);
				autoTimer.reset();
	//			driveRobot(0, 0);
			}
			break;
		case 4:
			if (autoTimer.get() > 1.0) {
				loopCount++;
				if (loopCount > 4) {
				autoStep = 5;
				distanceL.reset();
				distanceR.reset();
				}
				driveRobot(0,0);
			}
			break;
		case 5:
			if (distanceR.getRaw() < -2250) {
				autoStep = 6;
				turnSum = 0;
				lastOffYaw = 0;
				loopCount = 0;
				driveRobot(0,0);
			}
			else driveRobot(-0.45,-0.45);
			break;
		case 6:
//			setShooterAngle(2.9);
			if (loopCount == 10) {
//				sonarDist = moeSonar.getAverageVoltage();
//				double newAngle = sonarDist/0.15 + 0.2;
//				turnToAngle = 90.0 - Math.atan((newAngle + 0.22)/5.83);
				turnToAngle = -120.0;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 7) {
						autoStep = 7;
						shootAngle.set(0);
						autoTimer.reset();
						distanceL.reset();
						distanceR.reset();
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 7:
			// if (navX.getRoll() < -7.5) {
			if (distanceR.getRaw() < -7000) {
				autoStep = 8;
				loopCount = 0;
				driveRobot(0, 0);
			} else
				driveRobot(-0.7, -0.7);
			break;
		
		case 8:
			collector.set(-1.0);
			ballControl.set(-1.0);
			driveRobot(0, 0);
			if (loopCount == 100) {
				autoStep = 9;
				loopCount = 0;
			}
			break;
		case 9:
			collector.set(0);
			ballControl.set(0);
			driveRobot(0, 0);
			break;
		default:
			driveRobot(0, 0);

		}
	}
	
	void newWallRT5Low() {
		switch (autoStep) {
		case 1:
			moveArmDown();
//			setShooterAngle(2.9);
			 if (autoTimer.get() > 2.5) {
			//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
				defenseStep = 1;
				if (moat > 0.5) direction = 0.7;
				else direction = 0.6;
				startYaw = 0;			
				loopCount = 0;				
				floorCount = 0;
				minRoll = 0;
				maxRoll = 0;
			}
			break;
		case 2:
			moveArmDown();
//			setShooterAngle(2.9);
			collector.set(1.0);
			overDefenses();
			if (defenseStep == 5) {
				autoStep = 3;
				direction = 0.5;
			}
			break;
		case 3:
			collector.set(0.0);
			if (distanceR.getRaw() > 15000) {
				driveStraight.disable();
//				turnSum = 0;
				autoStep = 4;
				loopCount = 0;
				arm.set(0);
				autoTimer.reset();
	//			driveRobot(0, 0);
			}
			break;
		case 4:
			if (autoTimer.get() > 1.0) {
				loopCount++;
				if (loopCount > 4) {
				autoStep = 5;
				distanceL.reset();
				distanceR.reset();
				}
				driveRobot(0,0);
			}
			break;
		case 5:
			if (distanceR.getRaw() < -2250) {
				autoStep = 6;
				turnSum = 0;
				lastOffYaw = 0;
				loopCount = 0;
				driveRobot(0,0);
			}
			else driveRobot(-0.45,-0.45);
			break;
		case 6:
//			setShooterAngle(2.9);
			if (loopCount == 10) {
//				sonarDist = moeSonar.getAverageVoltage();
//				double newAngle = sonarDist/0.15 + 0.2;
//				turnToAngle = 90.0 - Math.atan((newAngle + 0.22)/5.83);
				turnToAngle = 120.0;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 7) {
						autoStep = 7;
						shootAngle.set(0);
						autoTimer.reset();
						distanceL.reset();
						distanceR.reset();
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 7:
			// if (navX.getRoll() < -7.5) {
			if (distanceR.getRaw() < -3700) {
				autoStep = 8;
				loopCount = 0;
				driveRobot(0, 0);
			} else
				driveRobot(-0.7, -0.7);
			break;
		
		case 8:
			collector.set(-1.0);
			ballControl.set(-1.0);
			driveRobot(0, 0);
			if (loopCount == 100) {
				autoStep = 9;
				loopCount = 0;
			}
			break;
		case 9:
			collector.set(0);
			ballControl.set(0);
			driveRobot(0, 0);
			break;
		default:
			driveRobot(0, 0);

		}
	}
	
	
	
	void lowBarNoShoot() {
		switch (autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(3.78);
			 if (autoTimer.get() > 2.5) {
			//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
			}
			break;
		case 2:
			moveArmDown();
			setShooterAngle(3.78);
			collector.set(1.0);
			double power = loopCount * .05;
			if (power > 0.5) {
				driveStraight.setPID(.04, 0.00005, .03);
				driveStraight.setSetpoint(0);
				direction = 0.5;
				shootAngle.set(0);
				driveStraight.enable();
				autoStep = 3;
			} else
				driveRobot(power, power);
			break;
		case 3:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > 9) {
				autoStep = 4;
			}
			double currentRoll = navX.getRoll();
			if (currentRoll > maxRoll)
				maxRoll = currentRoll;
			else if (currentRoll < minRoll)
				minRoll = currentRoll;
			break;
		case 4:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() < -8) {
				autoStep = 5;
				direction = 0.35;
			}
			double nowRoll = navX.getRoll();
			if (nowRoll > maxRoll)
				maxRoll = nowRoll;
			else if (nowRoll < minRoll)
				minRoll = nowRoll;
			break;
		case 5:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > rollOffset - 0.6) {
				floorCount++;
				if (floorCount == 2) {
					distanceL.reset();
					distanceR.reset();
					direction = 0.5;
					autoStep = 6;
				}
			}
			break;
		case 6:
			moveArmUp(3.0);
			collector.set(0);
			if (distanceR.getRaw() > 10000) {
				driveStraight.disable();
				turnSum = 0;
				autoStep = 7;
				loopCount = 0;
			}
			break;
		case 7:
			driveRobot(0,0);
	}
	
	}
	
	
	
	void roughTerrainNoShoot() {
		switch (autoStep) {
		case 1:
			moveArmDown();
//			setShooterAngle(2.94);
			 if (autoTimer.get() > 2.0) {
			//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
				defenseStep = 1;
				if (moat > 0.5) direction = 0.7;
				else direction = 0.6;
				startYaw = 0;			
				loopCount = 0;				
				floorCount = 0;
				minRoll = 0;
				maxRoll = 0;
			}
			break;
		case 2:
			moveArmDown();
//			setShooterAngle(2.94);
			collector.set(1.0);
			overDefenses();
			if (defenseStep == 5) {
				autoStep = 3;
				direction = 0.5;
			}
			break;
		case 3:
			collector.set(0.0);
			if (distanceR.getRaw() > 7000) {
				driveStraight.disable();
				//				turnSum = 0;
				autoStep = 4;
				//				loopCount = 0;
				arm.set(0);
				autoTimer.reset();
				driveRobot(0, 0);
			}
			break;
		case 4:
			driveRobot(0,0);
		default:
			driveRobot(0,0);
		}
	}
	
	void oldRoughTerrain2Low() {
		switch (autoStep) {
		case 1:
			moveArmDown();
			 if (autoTimer.get() > 2.5) {
		//	if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
			}
			break;
		case 2:
			moveArmDown();
			double power = loopCount * .05;
			if (power > 0.6) {
				driveStraight.setPID(.04, 0.00005, .03);
				driveStraight.setSetpoint(0);
				direction = 0.6;
				driveStraight.enable();
				autoStep = 3;
			} else
				driveRobot(power, power);
			break;
		case 3:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > 12) {
				autoStep = 4;
			}
			double currentRoll = navX.getRoll();
			if (currentRoll > maxRoll)
				maxRoll = currentRoll;
			else if (currentRoll < minRoll)
				minRoll = currentRoll;
			break;
		case 4:
			moveArmDown();
			collector.set(1.0);
			
			double nowRoll = navX.getRoll();
			if (nowRoll > maxRoll)
				maxRoll = nowRoll;
			else if (nowRoll < minRoll)
				minRoll = nowRoll;
			if (nowRoll < -17.0 ||(nowRoll < -11.0 && (distanceR.getRaw() > 2300 || distanceL.getRaw() > 2300))) {
//				if (navX.getRoll() < -10) {
					autoStep = 5;
					direction = 0.35;
				}
				break;
		case 5:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > rollOffset - 0.7) {
				floorCount++;
				if (floorCount == 2) {
					distanceL.reset();
					distanceR.reset();
					direction = 0.5;
					autoStep = 6;
				}
			}
			break;
		case 6:
			collector.set(0.0);
			if (distanceR.getRaw() > 13500) {
				driveStraight.disable();
				turnSum = 0;
				lastOffYaw = 0;
				autoStep = 7;
				loopCount = 0;
				arm.set(0);
				driveRobot(0, 0);
			}
			break;
		case 7:

			if (loopCount == 10) {
				// double sonarDist = moeSonar.getAverageVoltage();
				// double newAngle = sonarDist/0.15 + 0.2;
				// turnToAngle = 90.0 - Math.atan(newAngle/12.5);
				turnToAngle = -120.0;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 7) {
						autoStep = 8;
						loopCount = 0;
						distanceR.reset();
						distanceL.reset();
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 8:
			// if (navX.getRoll() < -7.5) {
			if (distanceR.getRaw() < -7000) {
				autoStep = 10;
				loopCount = 0;
				driveRobot(0, 0);
			} else
				driveRobot(-0.7, -0.7);
			break;
		case 9:

			break;
		case 10:
			collector.set(-1.0);
			ballControl.set(-1.0);
			driveRobot(0, 0);
			if (loopCount == 100) {
				autoStep = 11;
				loopCount = 0;
			}
			break;
		case 11:
			collector.set(0);
			ballControl.set(0);
			driveRobot(0, 0);
			break;
		case 12:
			driveRobot(0, 0);

		}
	}
	
	void oldRoughTerrain5Low() {
		switch (autoStep) {
		case 1:
			moveArmDown();
			 if (autoTimer.get() > 2.5) {
			//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
			}
			break;
		case 2:
			moveArmDown();
			double power = loopCount * .05;
			if (power > 0.6) {
				driveStraight.setPID(.04, 0.00005, .03);
				driveStraight.setSetpoint(0);
				direction = 0.6;
				driveStraight.enable();
				autoStep = 3;
			} else
				driveRobot(power, power);
			break;
		case 3:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > 12) {
				autoStep = 4;
			}
			double currentRoll = navX.getRoll();
			if (currentRoll > maxRoll)
				maxRoll = currentRoll;
			else if (currentRoll < minRoll)
				minRoll = currentRoll;
			break;
		case 4:
			moveArmDown();
			collector.set(1.0);
			
			double nowRoll = navX.getRoll();
			if (nowRoll > maxRoll)
				maxRoll = nowRoll;
			else if (nowRoll < minRoll)
				minRoll = nowRoll;
			if (nowRoll < -17.0 ||( nowRoll < -11.0 && (distanceR.getRaw() > 2300 || distanceL.getRaw() > 2300))) {	
//			if (navX.getRoll() < -10) {
				autoStep = 5;
				direction = 0.35;
			}
			break;
		case 5:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > rollOffset - 0.7) {
				floorCount++;
				if (floorCount == 1) {
					distanceL.reset();
					distanceR.reset();
					direction = 0.5;
					autoStep = 6;
				}
			}
			break;
		case 6:
			collector.set(0.0);
			if (distanceR.getRaw() > 15600) {
				driveStraight.disable();
				turnSum = 0;
				lastOffYaw = 0;
				autoStep = 7;
				loopCount = 0;
				driveRobot(0, 0);
				arm.set(0);
			}
			break;
		case 7:

			if (loopCount == 10) {
				// double sonarDist = moeSonar.getAverageVoltage();
				// double newAngle = sonarDist/0.15 + 0.2;
				// turnToAngle = 90.0 - Math.atan(newAngle/12.5);
				turnToAngle = 120.0;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 7) {
						autoStep = 8;
						loopCount = 0;
						distanceL.reset();
						distanceR.reset();
					}
				}
			} else
				driveRobot(0, 0);
			break;

		case 8:
			// if (navX.getRoll() < -7.5) {
			if (distanceR.getRaw() < -3000) {
				autoStep = 10;
				loopCount = 0;
				driveRobot(0, 0);
			} else
				driveRobot(-0.7, -0.7);
			break;
		case 9:

			break;
		case 10:
			collector.set(-1.0);
			ballControl.set(-1.0);
			driveRobot(0, 0);
			if (loopCount == 100) {
				autoStep = 11;
				loopCount = 0;
			}
			break;
		case 11:
			collector.set(0);
			ballControl.set(0);
			driveRobot(0, 0);
			break;
		case 12:
			driveRobot(0, 0);

		}
	}
	
	void oldRoughTerrain2High() {
		double aimAngle = 3.13;
		switch (autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(aimAngle);
			 if (autoTimer.get() > 2.5) {
			//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
			}
			break;
		case 2:
			moveArmDown();
			setShooterAngle(aimAngle);
			double power = loopCount * .05;
			if (power > 0.6) {
				driveStraight.setPID(.04, 0.00005, .03);
				driveStraight.setSetpoint(0);
				direction = 0.6;
				driveStraight.enable();
				autoStep = 3;
				shootAngle.set(0);
			} else
				driveRobot(power, power);
			break;
		case 3:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > 12) {
				autoStep = 4;
			}
			double currentRoll = navX.getRoll();
			if (currentRoll > maxRoll)
				maxRoll = currentRoll;
			else if (currentRoll < minRoll)
				minRoll = currentRoll;
			break;
		case 4:
			moveArmDown();
			collector.set(1.0);
			
			double nowRoll = navX.getRoll();
			if (nowRoll > maxRoll)
				maxRoll = nowRoll;
			else if (nowRoll < minRoll)
				minRoll = nowRoll;
			if (nowRoll < -17.0 ||( nowRoll < -11.0 && (distanceR.getRaw() > 2300 || distanceL.getRaw() > 2300))) {	
//			if (navX.getRoll() < -10) {
				autoStep = 5;
				direction = 0.35;
			}
			break;
		case 5:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > rollOffset - 0.7) {
				floorCount++;
				if (floorCount == 1) {
					distanceL.reset();
					distanceR.reset();
					direction = 0.5;
					autoStep = 6;
				}
			}
			break;
		case 6:
			collector.set(0.0);
			if (distanceR.getRaw() > 12850) {
				driveStraight.disable();
				turnSum = 0;
				lastOffYaw = 0;
				autoStep = 7;
				loopCount = 0;
				arm.set(0);
				driveRobot(0, 0);
			}
			break;
		case 7:
			setShooterAngle(aimAngle);
			if (loopCount == 10) {
	//			sonarDist = moeSonar.getAverageVoltage();
	//			double newAngle = sonarDist/0.15 + 0.2;
	//			turnToAngle = 90.0 - Math.atan((newAngle + 0.22)/5.83);
				turnToAngle = 60.0;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 7) {
						autoStep = 8;
						loopCount = 0;
						shootAngle.set(0);
						autoTimer.reset();
						distanceL.reset();
						distanceR.reset();
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 8:
			onSpeed = autoShooterSpeedControl();
			if (onSpeed || autoTimer.get() > 4.0) {
				autoStep = 9;
			}
//			shooterA.set(0.87);
//			shooterB.set(-0.76);
			
		
		case 9:
			if (autoTimer.get() > 2.0) {
				ballControl.set(1.0);
			}
			if (autoTimer.get() > 7.0) {
				autoStep = 10;
			}
			driveRobot(0, 0);
			break;
		case 10:

			ballControl.set(0);
			shooterA.set(0);
			shooterB.set(0);
			autoStep = 11;
			driveRobot(0, 0);
			break;
		case 11:
			driveRobot(0, 0);

		}
	}
	
	void oldRoughTerrain5High() {
			double aimAngle = 2.94;
			switch (autoStep) {
			case 1:
				moveArmDown();
				setShooterAngle(aimAngle);
				 if (autoTimer.get() > 2.5) {
				//if (armPot.getAverageVoltage() < 2.3) {
					autoStep = 2;
				}
				break;
			case 2:
				moveArmDown();
				setShooterAngle(aimAngle);
				double power = loopCount * .05;
				if (power > 0.6) {
					driveStraight.setPID(.04, 0.00005, .03);
					driveStraight.setSetpoint(0);
					direction = 0.6;
					driveStraight.enable();
					autoStep = 3;
					shootAngle.set(0);
				} else
					driveRobot(power, power);
				break;
			case 3:
				moveArmDown();
				collector.set(1.0);
				if (navX.getRoll() > 12) {
					autoStep = 4;
				}
				double currentRoll = navX.getRoll();
				if (currentRoll > maxRoll)
					maxRoll = currentRoll;
				else if (currentRoll < minRoll)
					minRoll = currentRoll;
				break;
			case 4:
				moveArmDown();
				collector.set(1.0);
				
				double nowRoll = navX.getRoll();
				if (nowRoll > maxRoll)
					maxRoll = nowRoll;
				else if (nowRoll < minRoll)
					minRoll = nowRoll;
				if (nowRoll < -17.0 ||( nowRoll < -11.0 && (distanceR.getRaw() > 2300 || distanceL.getRaw() > 2300))) {	
	//			if (navX.getRoll() < -10) {
					autoStep = 5;
					direction = 0.35;
				}
				break;
			case 5:
				moveArmDown();
				collector.set(1.0);
				if (navX.getRoll() > rollOffset - 0.7) {
					floorCount++;
					if (floorCount == 1) {
						distanceL.reset();
						distanceR.reset();
						direction = 0.5;
						autoStep = 6;
					}
				}
				break;
			case 6:
				collector.set(0.0);
				if (distanceR.getRaw() > 15700) {
					driveStraight.disable();
					turnSum = 0;
					lastOffYaw = 0;
					autoStep = 7;
					loopCount = 0;
					arm.set(0);
					driveRobot(0, 0);
				}
				break;
			case 7:
				setShooterAngle(aimAngle);
				if (loopCount == 10) {
					// double sonarDist = moeSonar.getAverageVoltage();
					// double newAngle = sonarDist/0.15 + 0.2;
					// turnToAngle = 90.0 - Math.atan(newAngle/12.5);
					turnToAngle = -60.0;
					turnRobot(turnToAngle);
					turnCount = 0;
				} else if (loopCount > 10) {
					if (turnRobot(turnToAngle)) {
						turnCount++;
						if (turnCount > 7) {
							autoStep = 8;
							loopCount = 0;
							arm.set(0);
							autoTimer.reset();
							shootAngle.set(0);
							distanceL.reset();
							distanceR.reset();
						}
					}
				} else
					driveRobot(0, 0);
				break;
	
			case 8:
				onSpeed = autoShooterSpeedControl();
				if (onSpeed | autoTimer.get() > 4.0) {
					autoStep = 9;
				}
	//			shooterA.set(0.87);
	//			shooterB.set(-0.76);
				if (distanceR.getRaw() < -600) {
					driveRobot(0, 0);
	//				autoStep = 9;
				} else
					driveRobot(-0.4, -0.4);
				break;
				
				
			case 9:
				if (autoTimer.get() > 2.0) {
					
				ballControl.set(1.0);
				}
				if (autoTimer.get() > 6.0) {
					autoStep = 10;
				}
				driveRobot(0, 0);
				break;
			case 10:
	
				ballControl.set(0);
				shooterA.set(0);
				shooterB.set(0);
				autoStep = 12;
				driveRobot(0, 0);
				break;
			case 12:
				driveRobot(0, 0);
			}
	
		}

	void outerWorks3() {
		double aimAngle = 3.2;
		switch(autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(aimAngle);
			if (autoTimer.get() > 2.0) {
				//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
				defenseStep = 1;
				if (moat > 0.5) direction = 0.7;
				else direction = 0.6;
				startYaw = 0;			
				loopCount = 0;				
				floorCount = 0;
				minRoll = 0;
				maxRoll = 0;
			}
			break;
		case 2:
			moveArmDown();
			setShooterAngle(aimAngle);
			collector.set(1.0);
			overDefenses();
			if (defenseStep == 5) {
				driveStraight.disable();
				turnSum = 0;
				lastOffYaw = 0;
				driveRobot(0,0);
				loopCount = 0;
				autoStep = 3;
				collector.set(0.0);
				turnCount = 0;
			}
			break;

		case 3:
			setShooterAngle(aimAngle);
			arm.set(0);
			if (loopCount == 10) {
				// double sonarDist = moeSonar.getAverageVoltage();
				// double newAngle = sonarDist/0.15 + 0.2;
				// turnToAngle = 90.0 - Math.atan(newAngle/12.5);
				turnToAngle = 14.5;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 5) {
						autoStep = 4;
						loopCount = 0;
						autoTimer.reset();
						shootAngle.set(0);
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 4:
		
			//		shooterA.set(0.87);
			//		shooterB.set(-0.76);
			onSpeed = autoShooterSpeedControl();
			if (onSpeed || autoTimer.get()>4.0) {
				//		if (autoTimer.get() > 2.0) {
				autoStep = 5;
			}
			driveRobot(0, 0);
			break;
		case 5:
			autoShooterSpeedControl();
			if (autoTimer.get() > 2.3) {
				ballControl.set(1.0);
			}
			if (ballSensor.get()) {
				autoStep = 6;
				autoTimer.reset();
				turnCount = 0;
				turnSum = 0;
				lastOffYaw = 0;
			}
			break;
		case 6:
			if (autoTimer.get() > 0.8) {
			shooterA.set(0);
			shooterB.set(0);
			ballControl.set(0);
			autoStep = 7;
			}
			break;
		case 7:
			turnToAngle = 0;
			if (turnRobot(turnToAngle)) {
				
				autoStep = 8;
				distanceL.reset();
				distanceR.reset();
				direction = 0.4;
				driveStraight.enable();
				
			}
			break;
		case 8:
			if (distanceR.getRaw() > 2700) {
				driveStraight.disable();
				driveRobot(0,0);
				autoStep = 9;
				turnSum = 0;
				lastOffYaw = 0;
				turnCount = 0;
			}
			break;
		case 9:
			turnToAngle = 179.8;
			if (turnRobot(turnToAngle)) {
				autoStep = 10;
				driveRobot(0,0);
				loopCount = 0;
			}
			break;
		case 10:
			if (loopCount == 10) {
				startYaw = 180.0;
				defenseStep = 1;
				if (moat > 0.5) direction = 0.7;
				else direction = 0.6;
				loopCount = 0;				
				floorCount = 0;
				minRoll = 0;
				maxRoll = 0;
				autoStep = 11;
			}
			break;
		case 11:
			overDefenses();
			if (defenseStep == 5) {
				driveStraight.disable();				
				driveRobot(0,0);				
				autoStep = 12;
				collector.set(0.0);
				
				
			}
			break;
		case 12:
			driveRobot(0,0);
			
			
			
			
		}
	}
	
	void outerWorks4() {
		
	}
	
	void lowBarWallRoutine() {
		double aimAngle = 3.52;
		switch (autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(aimAngle);
			 if (autoTimer.get() > 2.2) {
			//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
			}
			break;
		case 2:
			moveArmDown();
			setShooterAngle(aimAngle);
			collector.set(1.0);
			double power = loopCount * .05;
			if (power > 0.5) {
				driveStraight.setPID(.04, 0.00005, .03);
				driveStraight.setSetpoint(0);
				direction = 0.5;
				shootAngle.set(0);
				driveStraight.enable();
				autoStep = 3;
			} else
				driveRobot(power, power);
			break;
		case 3:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > 9) {
				autoStep = 4;
			}
			double currentRoll = navX.getRoll();
			if (currentRoll > maxRoll)
				maxRoll = currentRoll;
			else if (currentRoll < minRoll)
				minRoll = currentRoll;
			break;
		case 4:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() < -8) {
				autoStep = 5;
				direction = 0.35;
			}
			double nowRoll = navX.getRoll();
			if (nowRoll > maxRoll)
				maxRoll = nowRoll;
			else if (nowRoll < minRoll)
				minRoll = nowRoll;
			break;
		case 5:
			moveArmDown();
			collector.set(1.0);
			if (navX.getRoll() > rollOffset - 0.6) {
				floorCount++;
				if (floorCount == 2) {
					distanceL.reset();
					distanceR.reset();
					direction = 0.6;
					autoStep = 6;
					autoTimer.reset();
				}
			}
			break;
		case 6:
			if (autoTimer.get() > 1.5) arm.set(0);
			else	arm.set(-1);
			collector.set(0);
			if (distanceR.getRaw() > 15000) {
				driveStraight.disable();
				turnSum = 0;
				autoTimer.reset();
				arm.set(0);
				autoStep = 7;
				loopCount = 0;
			}
			break;
		case 7:
			if (autoTimer.get() > 1.0) {
				loopCount = 0;
				autoStep = 8;
				distanceL.reset();
				distanceR.reset();
				
				driveRobot(0,0);
			}
			break;
		case 8:
			if (distanceR.getRaw() < -3500) {
				autoStep = 9;
				turnSum = 0;
				loopCount = 0;
				driveRobot(0,0);
			}
			else driveRobot(-0.45,-0.45);
			break;
		case 9:
			setShooterAngle(aimAngle);
			if (loopCount == 10) {
//				sonarDist = moeSonar.getAverageVoltage();
//				double newAngle = sonarDist/0.15 + 0.2;
//				turnToAngle = 90.0 - Math.atan((newAngle + 0.22)/5.83);
				turnToAngle = 89.0;
				turnRobot(turnToAngle);
				turnCount = 0;
			} else if (loopCount > 10) {
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 7) {
						autoStep = 10;
						shootAngle.set(0);
						loopCount = 0;
						autoTimer.reset();
						distanceL.reset();
						distanceR.reset();
					}
				}
			} else
				driveRobot(0, 0);
			break;
		
			
	
		case 10:
			onSpeed = autoShooterSpeedControl();
			if (onSpeed || autoTimer.get()>4.0) {
				autoStep = 11;
			}
			break;
		case 11:
			if (autoTimer.get() > 2.5) {
				ballControl.set(1.0);
			}
			if (autoTimer.get() > 7.0) {
				autoStep = 12;
			}
			driveRobot(0, 0);
			break;
		case 12:

			ballControl.set(0);
			shooterA.set(0);
			shooterB.set(0);
			autoStep = 13;
			driveRobot(0, 0);
			break;
		case 13:
			driveRobot(0, 0);

		}
	}

	void newWallRT2High() {
			double aimAngle = 3.1;
			switch (autoStep) {
			case 1:
				moveArmDown();
				setShooterAngle(aimAngle);
				 if (autoTimer.get() > 2.1) {
						autoStep = 2;
						defenseStep = 1;
						if (moat > 0.5) direction = 0.7;
						else direction = 0.6;
						startYaw = 0;			
						loopCount = 0;				
						floorCount = 0;
						minRoll = 0;
						maxRoll = 0;
				}
				break;
			case 2:
				moveArmDown();
				setShooterAngle(aimAngle);
				collector.set(1.0);
				overDefenses();
				if (defenseStep == 5) {
					autoStep = 3;
					direction = 0.6;
				}
				break;
			case 3:
				collector.set(0.0);
				if (distanceR.getRaw() > 15000) {
					driveStraight.disable();
	//				turnSum = 0;
					autoStep = 4;
					loopCount = 0;
					arm.set(0);
					autoTimer.reset();
		//			driveRobot(0, 0);
				}
				break;
			case 4:
				if (autoTimer.get() > 1.0) {
					loopCount++;
					if (loopCount > 4) {
					autoStep = 5;
					distanceL.reset();
					distanceR.reset();
					}
					driveRobot(0,0);
				}
				break;
			case 5:
				if (distanceR.getRaw() < -4100) {
					autoStep = 6;
					turnSum = 0;
					loopCount = 0;
					driveRobot(0,0);
					lastOffYaw = 0;
				}
				else driveRobot(-0.45,-0.45);
				break;
			case 6:
				setShooterAngle(aimAngle);
				if (loopCount == 10) {
	//				sonarDist = moeSonar.getAverageVoltage();
	//				double newAngle = sonarDist/0.15 + 0.2;
	//				turnToAngle = 90.0 - Math.atan((newAngle + 0.22)/5.83);
					turnToAngle = 60.0;
					turnRobot(turnToAngle);
					turnCount = 0;
				} else if (loopCount > 10) {
					if (turnRobot(turnToAngle)) {
						turnCount++;
						if (turnCount > 7) {
							autoStep = 7;
							shootAngle.set(0);
							loopCount = 0;
							autoTimer.reset();
							distanceL.reset();
							distanceR.reset();
						}
					}
				} else
					driveRobot(0, 0);
				break;
			case 7:
				onSpeed = autoShooterSpeedControl();
				if (onSpeed || autoTimer.get()>4.0) {
	//			shooterA.set(0.87);
	//			shooterB.set(-0.76);
				autoStep = 8;
				}
				break;
			case 8:
				autoShooterSpeedControl();
				if (autoTimer.get() > 2.5) {
					ballControl.set(1.0);
				}
				if (autoTimer.get() > 7.0) {
					autoStep = 9;
				}
				driveRobot(0, 0);
				break;
			case 9:
	
				ballControl.set(0);
				shooterA.set(0);
				shooterB.set(0);
				autoStep = 10;
				driveRobot(0, 0);
				break;
			case 10:
				driveRobot(0, 0);
	
			}
		}

	void moatRampNoShoot() {
			switch (autoStep) {
			case 1:
				moveArmDown();
	//			setShooterAngle(2.94);
				 if (autoTimer.get() > 2.5) {
				//if (armPot.getAverageVoltage() < 2.3) {
					autoStep = 2;
					direction = 0.7;
					defenseStep = 1;
					startYaw = 0;			
					loopCount = 0;				
					floorCount = 0;
					minRoll = 0;
					maxRoll = 0;
				}
				break;
			case 2:
				moveArmDown();
	//			setShooterAngle(2.94);
				collector.set(1.0);
				overDefenses();
				if (defenseStep == 5) {
					autoStep = 3;
					direction = 0.5;
				}
				break;
			case 3:
				collector.set(0.0);
				if (distanceR.getRaw() > 7000) {
					driveStraight.disable();
					//				turnSum = 0;
					autoStep = 4;
					//				loopCount = 0;
					arm.set(0);
					autoTimer.reset();
					driveRobot(0, 0);
				}
				break;
			case 4:
				driveRobot(0,0);
			default:
				driveRobot(0,0);
			}
			
		}

	void lowBarFarShot() {
			double aimAngle = 3.52; 
			switch (autoStep) {
			case 1:
				moveArmDown();
				setShooterAngle(aimAngle);
				 if (autoTimer.get() > 2.0) {
				//if (armPot.getAverageVoltage() < 2.3) {
					autoStep = 2;
				}
				break;
			case 2:
				moveArmDown();
				setShooterAngle(aimAngle);
				collector.set(1.0);
				double power = loopCount * .05;
				if (power > 0.5) {
					driveStraight.setPID(.04, 0.00005, .03);
					driveStraight.setSetpoint(0);
					direction = 0.5;
					shootAngle.set(0);
					driveStraight.enable();
					autoStep = 3;
				} else
					driveRobot(power, power);
				break;
			case 3:
				moveArmDown();
				collector.set(1.0);
				if (navX.getRoll() > 9) {
					autoStep = 4;
				}
				double currentRoll = navX.getRoll();
				if (currentRoll > maxRoll)
					maxRoll = currentRoll;
				else if (currentRoll < minRoll)
					minRoll = currentRoll;
				break;
			case 4:
	//			moveArmDown();
				collector.set(1.0);
				if (navX.getRoll() < -8) {
					autoStep = 5;
					direction = 0.35;
					autoTimer.reset();
				}
				double nowRoll = navX.getRoll();
				if (nowRoll > maxRoll)
					maxRoll = nowRoll;
				else if (nowRoll < minRoll)
					minRoll = nowRoll;
				break;
			case 5:
				arm.set(-1.0);
				collector.set(1.0);
				if (navX.getRoll() > rollOffset - 0.6) {
					floorCount++;
					if (floorCount == 1) {
						driveStraight.disable();
						driveRobot(0,0);
						shootAngle.set(0);
						distanceL.reset();
						distanceR.reset();
						direction = 0.5;
						autoStep = 6;
						autoTimer.reset();
					}
				}
				break;
			
			case 6:
				collector.set(0);
				if (autoTimer.get() > 1.0) {
					autoStep = 7;
					turnSum = 0.0;
					turnCount = 0;
					lastOffYaw = 0;
					turnCount = 0;
				}
				break;
			case 7:
				turnToAngle = 38.0;
	//			turnRobot(turnToAngle);
	//			turnCount = 0;
			
				if (turnRobot(turnToAngle)) {
					turnCount++;
					if (turnCount > 5) {
						autoStep = 8;
						loopCount = 0;
						autoTimer.reset();
						arm.set(0);
						shootAngle.set(0);
					}
				}
			
			break;
			case 8:
				onSpeed = autoShooterSpeedControl();
				if (onSpeed || autoTimer.get() > 4.0) {
					autoStep = 9;
				}
		//		shooterA.set(0.87);
		//		shooterB.set(-0.76);
		//		if (autoTimer.get() > 2.0) {
		//			autoStep = 9;
		//		}
				driveRobot(0, 0);
				break;
			case 9:
				autoShooterSpeedControl();
				if (autoTimer.get() > 2.2) {
				ballControl.set(1.0);
				}
				if (autoTimer.get() > 7.0) {
					autoStep = 10;
				}
				driveRobot(0, 0);
				break;
			case 10:
	
				ballControl.set(0);
				shooterA.set(0);
				shooterB.set(0);
				autoStep = 11;
				driveRobot(0, 0);
				break;
			case 11:
				driveRobot(0, 0);
	
			
				
		}
	}

	void newWallRT5High() {
			double aimAngle = 2.9;
			switch (autoStep) {
			case 1:
				moveArmDown();
				setShooterAngle(aimAngle);
				 if (autoTimer.get() > 2.5) {
						autoStep = 2;
						defenseStep = 1;
						if (moat > 0.5) direction = 0.7;
						else direction = 0.6;
						startYaw = 0;			
						loopCount = 0;				
						floorCount = 0;
						minRoll = 0;
						maxRoll = 0;
				}
				break;
			case 2:
				moveArmDown();
				setShooterAngle(aimAngle);
				collector.set(1.0);
				overDefenses();
				if (defenseStep == 5) {
					autoStep = 3;
					direction = 0.5;
				}
				break;
			case 3:
				collector.set(0.0);
				if (tapeSensor.get()) {
					onTape++;
					if (onTape == 4) {
						seeTapeDist = distanceR.getRaw();
					}
				}
				if (distanceR.getRaw() > 15000) {
					driveStraight.disable();
	//				turnSum = 0;
					autoStep = 4;
					loopCount = 0;
					arm.set(0);
					autoTimer.reset();
		//			driveRobot(0, 0);
				}
				break;
			case 4:
				if (autoTimer.get() > 1.0) {
					loopCount++;
					if (loopCount > 4) {
						wallDist = distanceR.getRaw();
					autoStep = 5;
					distanceL.reset();
					distanceR.reset();
					}
					driveRobot(0,0);
				}
				break;
			case 5:
				if (distanceR.getRaw() < -2250) {
					autoStep = 6;
					turnSum = 0;
					lastOffYaw = 0;
					loopCount = 0;
					driveRobot(0,0);
				}
				else driveRobot(-0.45,-0.45);
				break;
			case 6:
				setShooterAngle(aimAngle);
				if (loopCount == 10) {
	//				sonarDist = moeSonar.getAverageVoltage();
	//				double newAngle = sonarDist/0.15 + 0.2;
	//				turnToAngle = 90.0 - Math.atan((newAngle + 0.22)/5.83);
					turnToAngle = -60.0;
					turnRobot(turnToAngle);
					turnCount = 0;
				} else if (loopCount > 10) {
					if (turnRobot(turnToAngle)) {
						turnCount++;
						if (turnCount > 7) {
							autoStep = 7;
							shootAngle.set(0);
							loopCount = 0;
							autoTimer.reset();
							distanceL.reset();
							distanceR.reset();
						}
					}
				} else
					driveRobot(0, 0);
				break;
			case 7:
				autoShooterSpeedControl();
				if (backUpFromBatter(800)) {
					autoStep = 8;
				}			
	//			shooterA.set(0.87);
	//			shooterB.set(-0.76);
				
				
				break;
			case 8:
				onSpeed = autoShooterSpeedControl();
				if (onSpeed || autoTimer.get()>4.0) {
					autoStep = 9;
				}
				break;
			case 9:
				if (autoTimer.get() > 2.5) {
					ballControl.set(1.0);
				}
				if (autoTimer.get() > 7.0) {
					autoStep = 10;
				}
				driveRobot(0, 0);
				break;
			case 10:
	
				ballControl.set(0);
				shooterA.set(0);
				shooterB.set(0);
				autoStep = 11;
				driveRobot(0, 0);
				break;
			case 11:
				driveRobot(0, 0);
	
			}
		}
	
	


}
