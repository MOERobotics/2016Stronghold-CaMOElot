package org.usfirst.frc.team365.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogInput;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

	int disabledLoop;
	int teleopLoop;
	int autoLoop;
	int testLoop;
	int startLoop;
	int autoStep;
	int loopCount;
	int autoRoutine;
	int autoRoutineDash, autoRoutineFile;
	int turnCount;
	int floorCount;
	boolean shooterOn;
	int onTape;
	boolean pastButton1F;
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
	double sonarDist;
	// double PIDKP;
	// double PIDKI;
	// double PIDKD;
	double desiredSpeedA = 4200.0;
	double desiredSpeedB = 4000.0;

	final double KPspeedA = 0.0001; // .0004;
	final double KPspeedB = 0.0001; // .0004;
	final double KIspeedA = 0.00001;
	final double KIspeedB = 0.00001;
	
	
	final double armDown = 1.9;
	final double armCheval = 2.6;
	final double armBumper = 3.4;
	final double arm90 = 4.4;
	final double armClimb = 4.6;
	
	
//	final double farAngleLimit = 3.8;
//	final double closeAngleLimit = 2.9;
	final double shooterClose = 2.9; // 32.8 degrees
	final double shooterOuterWorks = 3.8; // 50.7 degrees
	
	GripRoutine grip;
	int cameraOffsetX,cameraOffsetY;
	double closeEnoughX;
	double closeEnoughY;
	
	final static int NAK=0;
	AutoData autoData;
	AutoFile autoFile;
	
	public Robot() {
		
//		  server = CameraServer.getInstance(); 
//		  server.setSize(1);
//		  server.setQuality(50); 
//		  server.startAutomaticCapture("cam1");
		
		grip=new GripRoutine(cameraOffsetX,cameraOffsetY);
		 
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
		SmartDashboard.putNumber("AutoFileChoice", 0);

		angleController = new PIDController(0.03, 0.0005, 0.5, navX, this);
		angleController.setContinuous();
		angleController.setInputRange(-180.0, 180.0);
		angleController.setOutputRange(-0.6, 0.6);
		angleController.setAbsoluteTolerance(1.0);

		driveStraight = new PIDController(0.04, 0.00005, 0.03, navX, this);
		driveStraight.setContinuous();
		driveStraight.setInputRange(-180.0, 180.0);
		driveStraight.setOutputRange(-1.0, 1.0);

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
		
		if(!funStick.getTrigger()&&pastButton1F)
		{
			autoFile.writeAutoFile((int)SmartDashboard.getNumber("AutoFileChoice"));
			autoRoutineFile=autoFile.readAutoFile();
		}

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

			//double chooseAuto = SmartDashboard.getNumber("autoChoice");
			//autoRoutine = (int) chooseAuto;
			autoRoutineDash=(int)SmartDashboard.getNumber("autoChoice");
			
			moat=(int)(SmartDashboard.getNumber("Moat"));
			
			SmartDashboard.putNumber("AutoFile", autoRoutineFile);
			SmartDashboard.putString("AutoAck/Nak", autoRoutineFile != 0 ? "Ack" : "Nak");
			
			//autoRoutine=getAuto();

		}
		pastButton1F=funStick.getRawButton(1);
	}

	public void teleopInit() {
		AutoData.safeSave(autoData);// the nominal delay shouldn't be an issue here
		
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
//			 testShooterSpeeds();
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

		// Testing PID controllers for going straight and turning

		
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
		
		 else if (newButton3D) { // programmed turn routine to 0 degrees
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
			backUpFromBatter(1000);
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
		autoData=new AutoData(autoRoutine);
		
		if(autoRoutineDash!=0) autoRoutine=autoRoutineDash;
		else if(autoRoutineFile!=0) autoRoutine=autoRoutineFile;
		else autoRoutine=0;
		
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
//			roughTerrain2High();
			newWallRT2High();
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
//			roughTerrain5High();
			newWallRT5High();
			break;
		case 6:
			lowBarAutoLow();
			break;
		case 7:
//			roughTerrain2Low();
			newWallRT2Low();
			break;
		case 8:
//			roughTerrain5Low();
			newWallRT5Low();
			break;
		case 9:
			lowBarNoShoot();
			break;
		case 10:
//			roughTerrainNoShoot();
			RTerrainRWNoShoot();
			break;
		case 11:
//			moatAutoNoShoot();
			moatRampNoShoot();
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
		
		serializeAutoData();
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

	void testTalons()
	{
		if(driveStick.getRawButton(11))
		{
			if(driveStick.getRawButton(5))
				driveRA.set(1);
			else if(driveStick.getRawButton(6))
				driveRA.set(-1);
			else
				driveRA.set(0);
			if(driveStick.getRawButton(7))
				driveRA.set(1);
			else if(driveStick.getRawButton(8))
				driveRA.set(-1);
			else
				driveRA.set(0);
			if(driveStick.getRawButton(9))
				driveRA.set(1);
			else if(driveStick.getRawButton(10))
				driveRA.set(-1);
			else
				driveRA.set(0);
		}
		else if(driveStick.getRawButton(12))
		{
			if(driveStick.getRawButton(5))
				driveLA.set(1);
			else if(driveStick.getRawButton(6))
				driveLA.set(-1);
			else
				driveLA.set(0);
			if(driveStick.getRawButton(7))
				driveLA.set(1);
			else if(driveStick.getRawButton(8))
				driveLA.set(-1);
			else
				driveLA.set(0);
			if(driveStick.getRawButton(9))
				driveLA.set(1);
			else if(driveStick.getRawButton(10))
				driveLA.set(-1);
			else
				driveLA.set(0);
		}
		
		
		
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
		} else if (funStick.getRawButton(3)) { // run collector to push balls
												// out
			collector.set(-1.0);
			ballControl.set(-1.0);
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
		if (funStick.getRawButton(11))
			setShooterAngle(shooterClose);
		else if (funStick.getRawButton(10))
			setShooterAngle(shooterOuterWorks);

		else if (funStick.getRawButton(6)) { // want steeper shoot angle
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
		if (funStick.getRawButton(8)){
				//&&(armPot.getAverageVoltage()<4.8&&armPot.getAverageVoltage()>4.2)) { // move scaling arm out
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
		// if (currentYaw < 0) currentYaw = currentYaw + 360.0;
		double offYaw = setBearing - currentYaw;

		if (offYaw * lastOffYaw < 0)
			turnSum = 0;
		if (offYaw > 1 || offYaw < -1) {

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
				if (floorCount == 1) {
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
		switch(defenseStep) {
		case 1: 
			if (navX.getYaw() < (startYaw - 30)) {
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
			if (navX.getYaw() > (startYaw + 15)) {
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
			if (navX.getYaw() < (startYaw + 7)) {
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
	
	public void gripAdjust()
	{
		DualTransfer<Double, Double>delta2d=grip.analyze();
		double dx=delta2d.getArg1();	
		double dy=delta2d.getArg2();
		if(Math.abs(dx)>closeEnoughX)
			driveRobot(dx,-dx);
		else	driveRobot(0,0);
		if(Math.abs(dy)>closeEnoughY)
			shootAngle.set(dy);
		else	shootAngle.set(0);
	}
	
	public void serializeAutoData()
	{
		autoData.armAngle=armPot.getAverageVoltage();
		autoData.shooterAngle=launchAngle.getAverageVoltage();
		autoData.powerA=powerA;
		autoData.powerB=powerB;
		autoData.yaw=navX.getYaw();
		autoData.pitch=navX.getPitch();
		autoData.roll=navX.getRoll();
	}
	
	void autoTestRoutine() {
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
			if (distanceL.getRaw() > 5600.0) {
				autoStep = 5;
				driveStraight.disable();
				loopCount = 0;
				driveRobot(0, 0);
			}
			break;
		case 3:
			if (loopCount > 30) {
				autoStep = 4;
				angleController.setAbsoluteTolerance(1);
				angleController.setOutputRange(-0.5, 0.5);
				angleController.setSetpoint(60.0);
				direction = 0;
				angleController.enable();
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
		switch (autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(3.52);
			 if (autoTimer.get() > 2.2) {
			//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
			}
			break;
		case 2:
			moveArmDown();
			setShooterAngle(3.52);
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
				if (floorCount == 1) {
					distanceL.reset();
					distanceR.reset();
					direction = 0.5;
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
				autoTimer.reset();
				arm.set(0);
				autoStep = 7;
				loopCount = 0;
			}
			break;
		case 7:
			//moveArmUp(3.0);
			setShooterAngle(3.52);
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
					if (turnCount > 7) {
						autoStep = 8;
						loopCount = 0;
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
	
	void newWallRT2High() {
		switch (autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(3.1);
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
			setShooterAngle(3.1);
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
			}
			else driveRobot(-0.45,-0.45);
			break;
		case 6:
			setShooterAngle(3.1);
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
	


	

	
	void newRT3High() {
		switch(autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(3.2);
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
			setShooterAngle(3.2);
			collector.set(1.0);
			overDefenses();
			if (defenseStep == 5) {
				autoStep = 3;
				direction = 0.5;
			}
			break;
		case 3:
			collector.set(0.0);
			if (distanceR.getRaw() > 7590) {
				driveStraight.disable();
				turnSum = 0;
				autoStep = 4;
				loopCount = 0;
				arm.set(0);
				driveRobot(0, 0);
			}
			break;
		case 4:
			setShooterAngle(3.2);
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
					if (turnCount > 7) {
						autoStep = 5;
						loopCount = 0;
						autoTimer.reset();
						shootAngle.set(0);
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 5:
			//		shooterA.set(0.87);
			//		shooterB.set(-0.76);
			onSpeed = autoShooterSpeedControl();
			if (onSpeed || autoTimer.get()>4.0) {
				//		if (autoTimer.get() > 2.0) {
				autoStep = 6;
			}
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
		}

	}

	void alternateRT3High() {
		switch (autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(2.94);
			 if (autoTimer.get() > 2.5) {
		//	if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
			}
			break;
		case 2:
			moveArmDown();
			setShooterAngle(2.94);
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
			setShooterAngle(2.94);
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
		switch(autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(2.92);
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
			setShooterAngle(2.92);
			collector.set(1.0);
			overDefenses();
			if (defenseStep == 5) {
				autoStep = 3;
				direction = 0.5;
			}
			break;
		case 3:
			collector.set(0.0);
			if (distanceR.getRaw() > 9300) {
				driveStraight.disable();
				turnSum = 0;
				autoStep = 4;
				loopCount = 0;
				arm.set(0);
				driveRobot(0, 0);
			}
			break;
		case 4:
			setShooterAngle(2.92);
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
					if (turnCount > 7) {
						autoStep = 5;
						loopCount = 0;
						autoTimer.reset();
						shootAngle.set(0);
					}
				}
			} else
				driveRobot(0, 0);
			break;
		case 5:
			//		shooterA.set(0.87);
			//		shooterB.set(-0.76);
			onSpeed = autoShooterSpeedControl();
			if (onSpeed || autoTimer.get()>4.0) {
				//		if (autoTimer.get() > 2.0) {
				autoStep = 6;
			}
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
		}

	}

	void newWallRT5High() {
		switch (autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(2.9);
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
			setShooterAngle(2.9);
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
				loopCount = 0;
				driveRobot(0,0);
			}
			else driveRobot(-0.45,-0.45);
			break;
		case 6:
			setShooterAngle(2.9);
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
	
	
	
	void RTerrainRWNoShoot() {
		switch (autoStep) {
		case 1:
			moveArmDown();
//			setShooterAngle(2.94);
			 if (autoTimer.get() > 2.5) {
			//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
				defenseStep = 1;
				direction = 0.6;
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
	
	void oldRoughTerrain5High() {
		switch (autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(2.94);
			 if (autoTimer.get() > 2.5) {
			//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
			}
			break;
		case 2:
			moveArmDown();
			setShooterAngle(2.94);
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
				autoStep = 7;
				loopCount = 0;
				arm.set(0);
				driveRobot(0, 0);
			}
			break;
		case 7:
			setShooterAngle(2.94);
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
		switch (autoStep) {
		case 1:
			moveArmDown();
			setShooterAngle(3.13);
			 if (autoTimer.get() > 2.5) {
			//if (armPot.getAverageVoltage() < 2.3) {
				autoStep = 2;
			}
			break;
		case 2:
			moveArmDown();
			setShooterAngle(3.13);
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
				autoStep = 7;
				loopCount = 0;
				arm.set(0);
				driveRobot(0, 0);
			}
			break;
		case 7:
			setShooterAngle(3.13);
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
	
	


}
