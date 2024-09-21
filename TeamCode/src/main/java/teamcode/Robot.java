/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 */

package teamcode;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcDigitalInput;
import TrcCommonLib.trclib.TrcMotor;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcServo;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import ftclib.FtcDashboard;
import ftclib.FtcDcMotor;
import ftclib.FtcMatchInfo;
import ftclib.FtcOpMode;
import ftclib.FtcRobotBattery;
import ftclib.FtcMatchInfo;
import teamcode.autotasks.TaskAutoPlacePixel;
import teamcode.drivebases.MecanumDrive;
import teamcode.drivebases.RobotDrive;
import teamcode.drivebases.SwerveDrive;
import teamcode.subsystems.BlinkinLEDs;
import teamcode.subsystems.ElevatorArm;
import teamcode.subsystems.Intake;
import teamcode.subsystems.PixelTray;
import teamcode.vision.Vision;

/**
 * This class creates the robot object that consists of sensors, indicators, drive base and all the subsystems.
 */
public class Robot
{
    private static final String moduleName = Robot.class.getSimpleName();
    private static final double STATUS_UPDATE_INTERVAL = 0.1;   // 100 msec
    //
    // Global objects.
    //
    public final FtcOpMode opMode;
    public final TrcDbgTrace globalTracer;
    public final FtcDashboard dashboard;
    public static FtcMatchInfo matchInfo = null;
    private static TrcPose2D endOfAutoRobotPose = null;
    private static double nextStatusUpdateTime = 0.0;
    //
    // Vision subsystems.
    //
    public Vision vision;
    //
    // Sensors and indicators.
    //
    public BlinkinLEDs blinkin;
    public FtcRobotBattery battery;
    //
    // Subsystems.
    //
    public RobotDrive robotDrive;
    public ElevatorArm elevatorArm;
    public Intake intake;
    public PixelTray pixelTray;
    public FtcDcMotor launcher;

    public TaskAutoPlacePixel placePixelTask;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param runMode specifies robot running mode (Auto, TeleOp, Test), can be used to create and initialize mode
     *        specific sensors and subsystems if necessary.
     */
    public Robot(TrcRobot.RunMode runMode)
    {
        //
        // Initialize global objects.
        //
        opMode = FtcOpMode.getInstance();
        globalTracer = TrcDbgTrace.getGlobalTracer();
        dashboard = FtcDashboard.getInstance();
        nextStatusUpdateTime = TrcTimer.getCurrentTime();

        speak("Init starting");
        //
        // Initialize vision subsystems.
        //
        if (RobotParams.Preferences.tuneColorBlobVision ||
            RobotParams.Preferences.useAprilTagVision ||
            RobotParams.Preferences.useColorBlobVision ||
            RobotParams.Preferences.useTensorFlowVision)
        {
            vision = new Vision(this);
        }
        //
        // If robotType is NoRobot, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        //
        if (RobotParams.Preferences.robotType != RobotParams.RobotType.NoRobot)
        {
            //
            // Create and initialize sensors and indicators.
            //
            if (RobotParams.Preferences.useBlinkin)
            {
                blinkin = new BlinkinLEDs(RobotParams.HWNAME_BLINKIN);
            }

            if (RobotParams.Preferences.useBatteryMonitor)
            {
                battery = new FtcRobotBattery();
            }
            //
            // Create and initialize RobotDrive.
            //
            robotDrive =
                RobotParams.Preferences.robotType == RobotParams.RobotType.SwerveRobot?
                    new SwerveDrive(): new MecanumDrive();
            //
            // Create and initialize other subsystems.
            //
            if (RobotParams.Preferences.useSubsystems)
            {
                if (RobotParams.Preferences.useElevatorArm)
                {
                    elevatorArm = new ElevatorArm();
                    // Code review: Should this init be in Robot.startMode?
                    if (runMode == TrcRobot.RunMode.TELEOP_MODE)
                    {
                        elevatorArm.wristSetPosition(RobotParams.WRIST_DOWN_POS);
                    }
                }

                if (RobotParams.Preferences.useIntake)
                {
                    intake = new Intake(RobotParams.HWNAME_INTAKE);
                }

                if (RobotParams.Preferences.usePixelTray)
                {
                    pixelTray = new PixelTray(RobotParams.HWNAME_PIXELTRAY);
                    // Code review: Should this init be in Robot.startMode?
                    if (runMode == TrcRobot.RunMode.TELEOP_MODE)
                    {
                        pixelTray.setUpperGateOpened(true, null);
                        pixelTray.setLowerGateOpened(true, null);
                    }
                    else
                    {
                        pixelTray.setUpperGateOpened(false, null);
                        pixelTray.setLowerGateOpened(false, null);
                    }
                }

                if (RobotParams.Preferences.useLauncher)
                {
                    launcher = new FtcDcMotor(RobotParams.HWNAME_LAUNCHER);
                    launcher.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
                    launcher.setMotorInverted(RobotParams.LAUNCHER_MOTOR_INVERTED);
                }
                //
                // Create auto tasks here.
                //
                placePixelTask = new TaskAutoPlacePixel("PlacePixelTask", this);
            }
        }

        speak("Init complete");
    }   //Robot

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return RobotParams.ROBOT_NAME;
    }   //toString

    /**
     * This method is call when the robot mode is about to start. It contains code to initialize robot hardware
     * necessary for running the robot mode.
     *
     * @param runMode specifies the robot mode it is about to start, can be used to initialize mode specific hardware.
     */
    public void startMode(TrcRobot.RunMode runMode)
    {
        if (robotDrive != null)
        {
            //
            // Since the IMU gyro is giving us cardinal heading, we need to enable its cardinal to cartesian converter.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(true);
                // The following are performance counters, could be disabled for competition if you want.
                // But it might give you some insight if somehow autonomous wasn't performing as expected.
                robotDrive.gyro.setElapsedTimerEnabled(true);
            }
            //
            // Enable odometry for all opmodes. We may need odometry in TeleOp for auto-assist drive.
            //
            robotDrive.driveBase.setOdometryEnabled(true);
            if (runMode == TrcRobot.RunMode.TELEOP_MODE)
            {
                if (endOfAutoRobotPose != null)
                {
                    // We had a previous autonomous run that saved the robot position at the end, use it.
                    robotDrive.driveBase.setFieldPosition(endOfAutoRobotPose);
                    globalTracer.traceInfo(moduleName, "Restore saved RobotPose=" + endOfAutoRobotPose);
                }
            }
            // Consume it so it's no longer valid for next run.
            endOfAutoRobotPose = null;
        }
        TrcDigitalInput.setElapsedTimerEnabled(true);
        TrcMotor.setElapsedTimerEnabled(true);
        TrcServo.setElapsedTimerEnabled(true);
    }   //startMode

    /**
     * This method is call when the robot mode is about to end. It contains code to cleanup robot hardware before
     * exiting the robot mode.
     *
     * @param runMode specifies the robot mode it is about to stop, can be used to cleanup mode specific hardware.
     */
    public void stopMode(TrcRobot.RunMode runMode)
    {
        //
        // Print all performance counters if there are any.
        //
        if (robotDrive != null && robotDrive.gyro != null)
        {
            robotDrive.gyro.printElapsedTime(globalTracer);
            robotDrive.gyro.setElapsedTimerEnabled(false);
        }
        TrcDigitalInput.printElapsedTime(globalTracer);
        TrcDigitalInput.setElapsedTimerEnabled(false);
        TrcMotor.printElapsedTime(globalTracer);
        TrcMotor.setElapsedTimerEnabled(false);
        TrcServo.printElapsedTime(globalTracer);
        TrcServo.setElapsedTimerEnabled(false);
        //
        // Disable vision.
        //
        if (vision != null)
        {
            if (vision.rawColorBlobVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling RawColorBlobVision.");
                vision.setRawColorBlobVisionEnabled(false);
            }

            if (vision.aprilTagVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling AprilTagVision.");
                vision.setAprilTagVisionEnabled(false);
            }

            if (vision.purplePixelVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling PurplePixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.PurplePixel, false);
            }

            if (vision.greenPixelVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling GreenPixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.GreenPixel, false);
            }

            if (vision.yellowPixelVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling YellowPixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.YellowPixel, false);
            }

            if (vision.whitePixelVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling WhitePixelVision.");
                vision.setPixelVisionEnabled(Vision.PixelType.WhitePixel, false);
            }

            if (vision.redBlobVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling RedBlobVision.");
                vision.setRedBlobVisionEnabled(false);
            }

            if (vision.blueBlobVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling BlueBlobVision.");
                vision.setBlueBlobVisionEnabled(false);
            }

            if (vision.tensorFlowVision != null)
            {
                globalTracer.traceInfo(moduleName, "Disabling TensorFlowVision.");
                vision.setTensorFlowVisionEnabled(false);
            }
       }

        if (robotDrive != null)
        {
            if (runMode == TrcRobot.RunMode.AUTO_MODE)
            {
                // Save current robot location at the end of autonomous so subsequent teleop run can restore it.
                endOfAutoRobotPose = robotDrive.driveBase.getFieldPosition();
                globalTracer.traceInfo(moduleName, "Saved robot pose=" + endOfAutoRobotPose);
            }
            //
            // Disable odometry.
            //
            robotDrive.driveBase.setOdometryEnabled(false);
            //
            // Disable gyro task.
            //
            if (robotDrive.gyro != null)
            {
                robotDrive.gyro.setEnabled(false);
            }
        }
    }   //stopMode

    /**
     * This method update all subsystem status on the dashboard.
     */
    public void updateStatus()
    {
        if (TrcTimer.getCurrentTime() > nextStatusUpdateTime)
        {
            int lineNum = 2;

            nextStatusUpdateTime += STATUS_UPDATE_INTERVAL;
            if (robotDrive != null)
            {
                dashboard.displayPrintf(lineNum++, "DriveBase: Pose=%s", robotDrive.driveBase.getFieldPosition());
            }

            if (elevatorArm != null)
            {
                if (elevatorArm.elevator != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "Elevator: power=%.3f, pos=%.1f, target=%.1f, lowerLimitSw=%s",
                        elevatorArm.elevator.getPower(), elevatorArm.elevator.getPosition(),
                        elevatorArm.elevator.getPidTarget(), elevatorArm.elevator.isLowerLimitSwitchActive());
                }

                if (elevatorArm.arm != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "Arm: power=%.3f, pos=%.1f/%f, target=%.1f",
                        elevatorArm.arm.getPower(), elevatorArm.arm.getPosition(),
                        elevatorArm.arm.getEncoderRawPosition(), elevatorArm.arm.getPidTarget());
                }

                if (elevatorArm.wrist != null)
                {
                    if (elevatorArm.wristSensor != null)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "Wrist: pos=%.1f, distance=%.1f",
                            elevatorArm.wrist.getPosition(), elevatorArm.wristGetDistance());
                    }
                    else
                    {
                        dashboard.displayPrintf(lineNum++, "Wrist: pos=%.1f", elevatorArm.wrist.getPosition());
                    }
                }
            }

            if (intake != null)
            {
                dashboard.displayPrintf(
                    lineNum++, "Intake: power=%.1f", intake.getIntakeMotor().getPower());
            }

            if (pixelTray != null)
            {
                dashboard.displayPrintf(
                    lineNum++, "PixelTray: lowerGateOpened=%s, upperGateOpened=%s",
                    pixelTray.isLowerGateOpened(), pixelTray.isUpperGateOpened());
            }
        }
    }   //updateStatus

    /**
     * This method zero calibrates all subsystems.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     */
    public void zeroCalibrate(String owner)
    {
        if (elevatorArm != null)
        {
            elevatorArm.zeroCalibrate(owner);
        }
    }   //zeroCalibrate

    /**
     * This method zero calibrates all subsystems.
     */
    public void zeroCalibrate()
    {
        zeroCalibrate(null);
    }   //zeroCalibrate

    /**
     * This method sets the robot's starting position according to the autonomous choices.
     *
     * @param autoChoices specifies all the auto choices.
     */
    public void setRobotStartPosition(FtcAuto.AutoChoices autoChoices)
    {
        robotDrive.driveBase.setFieldPosition(
            adjustPoseByAlliance(
                autoChoices.startPos == FtcAuto.StartPos.AUDIENCE?
                    RobotParams.STARTPOS_BLUE_AUDIENCE: RobotParams.STARTPOS_BLUE_BACKSTAGE,
                autoChoices.alliance, false));
    }   //setRobotStartPosition

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param x specifies x position in the blue alliance in tile unit.
     * @param y specifies y position in the blue alliance in tile unit.
     * @param heading specifies heading in the blue alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if pose is in tile units, false otherwise.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(
        double x, double y, double heading, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        TrcPose2D newPose = new TrcPose2D(x, y, heading);

        if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
        {
            double angleDelta = (newPose.angle - 90.0) * 2.0;
            newPose.angle -= angleDelta;
            newPose.y = -newPose.y;
        }

        if (isTileUnit)
        {
            newPose.x *= RobotParams.FULL_TILE_INCHES;
            newPose.y *= RobotParams.FULL_TILE_INCHES;
        }

        return newPose;
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param x specifies x position in the blue alliance in tile unit.
     * @param y specifies y position in the blue alliance in tile unit.
     * @param heading specifies heading in the blue alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(double x, double y, double heading, FtcAuto.Alliance alliance)
    {
        return adjustPoseByAlliance(x, y, heading, alliance, true);
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param pose specifies pose in the blue alliance in tile unit.
     * @param alliance specifies the alliance to be converted to.
     * @param isTileUnit specifies true if pose is in tile units, false otherwise.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, FtcAuto.Alliance alliance, boolean isTileUnit)
    {
        return adjustPoseByAlliance(pose.x, pose.y, pose.angle, alliance, isTileUnit);
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param pose specifies pose in the blue alliance in tile unit.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, FtcAuto.Alliance alliance)
    {
        return adjustPoseByAlliance(pose, alliance, true);
    }   //adjustPoseByAlliance

    /**
     * This method sends the text string to the Driver Station to be spoken using text to speech.
     *
     * @param sentence specifies the sentence to be spoken by the Driver Station.
     */
    public void speak(String sentence)
    {
        opMode.telemetry.speak(sentence);
    }   //speak

}   //class Robot
