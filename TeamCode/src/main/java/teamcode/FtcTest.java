/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 */

package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.Arrays;
import java.util.Locale;

import TrcCommonLib.command.CmdDriveMotorsTest;
import TrcCommonLib.command.CmdPidDrive;
import TrcCommonLib.command.CmdTimedDrive;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcElapsedTimer;
import TrcCommonLib.trclib.TrcGameController;
import TrcCommonLib.trclib.TrcPidController;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;
import ftclib.FtcChoiceMenu;
import ftclib.FtcGamepad;
import ftclib.FtcMenu;
import ftclib.FtcPidCoeffCache;
import ftclib.FtcValueMenu;
import teamcode.drivebases.RobotDrive;
import teamcode.drivebases.SwerveDrive;
import teamcode.vision.Vision;

/**
 * This class contains the Test Mode program. It extends FtcTeleOp so that we can teleop control the robot for
 * testing purposes. It provides numerous tests for diagnosing problems with the robot. It also provides tools
 * for tuning and calibration. FTC3543
 */
@TeleOp(name="FtcTest", group="Ftc16607")
public class FtcTest extends FtcTeleOp
{
    private static final String moduleName = FtcTest.class.getSimpleName();
    private static final boolean logEvents = true;
    private static final boolean debugPid = true;
    private static final double LAUNCHER_POWER_STEP = 0.05;

    private enum Test
    {
        SENSORS_TEST,
        SUBSYSTEMS_TEST,
        VISION_TEST,
        TUNE_COLORBLOB_VISION,
        DRIVE_SPEED_TEST,
        DRIVE_MOTORS_TEST,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        PID_DRIVE,
        TUNE_X_PID,
        TUNE_Y_PID,
        TUNE_TURN_PID,
        PURE_PURSUIT_DRIVE,
        CALIBRATE_SWERVE_STEERING,
        TUNE_LAUNCHER_POWER
    }   //enum Test

    /**
     * This class stores the test menu choices.
     */
    private static class TestChoices
    {
        Test test = Test.SENSORS_TEST;
        double xTarget = 0.0;
        double yTarget = 0.0;
        double turnTarget = 0.0;
        double driveTime = 0.0;
        double drivePower = 0.0;
        TrcPidController.PidCoefficients tunePidCoeff = null;
        double tuneDistance = 0.0;
        double tuneHeading = 0.0;
        double tuneDrivePower = 0.0;

        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "test=\"%s\" " +
                "xTarget=%.1f " +
                "yTarget=%.1f " +
                "turnTarget=%1f " +
                "driveTime=%.1f " +
                "drivePower=%.1f " +
                "tunePidCoeff=%s " +
                "tuneDistance=%.1f " +
                "tuneHeading=%.1f " +
                "tuneDrivePower=%.1f",
                test, xTarget, yTarget, turnTarget, driveTime, drivePower, tunePidCoeff, tuneDistance, tuneHeading,
                tuneDrivePower);
        }   //toString

    }   //class TestChoices

    private final FtcPidCoeffCache pidCoeffCache = new FtcPidCoeffCache(RobotParams.TEAM_FOLDER_PATH);
    private final TestChoices testChoices = new TestChoices();
    private TrcElapsedTimer elapsedTimer = null;
    private FtcChoiceMenu<Test> testMenu = null;

    private TrcRobot.RobotCommand testCommand = null;
    // Drive Speed Test.
    private double maxDriveVelocity = 0.0;
    private double maxDriveAcceleration = 0.0;
    private double prevTime = 0.0;
    private double prevVelocity = 0.0;
    // Swerve Steering Calibration.
    private boolean steerCalibrating = false;
    // Color Blob Vision Turning.
    private static final double[] COLOR_THRESHOLD_LOW_RANGES = {0.0, 0.0, 0.0};
    private static final double[] COLOR_THRESHOLD_HIGH_RANGES = {255.0, 255.0, 255.0};
    private double[] colorThresholds = null;
    private int colorThresholdIndex = 0;
    private double colorThresholdMultiplier = 1.0;
    private boolean teleOpControlEnabled = true;
    private long exposure;
    private WebcamName frontWebcam = null;
    private WebcamName rearWebcam = null;

    //
    // Overrides FtcOpMode abstract method.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station is pressed.
     */
    @Override
    public void robotInit()
    {
        //
        // TeleOp initialization.
        //
        super.robotInit();
        if (robot.vision != null)
        {
            frontWebcam = robot.vision.getFrontWebcam();
            rearWebcam = robot.vision.getRearWebcam();
        }

        if (RobotParams.Preferences.useLoopPerformanceMonitor)
        {
            elapsedTimer = new TrcElapsedTimer("TestLoopMonitor", 2.0);
        }
        //
        // Test menus.
        //
        doTestMenus();
        //
        // Create the robot command for the tests that need one.
        //
        switch (testChoices.test)
        {
            case DRIVE_MOTORS_TEST:
                if (robot.robotDrive != null)
                {
                    testCommand = new CmdDriveMotorsTest(robot.robotDrive.driveMotors, 5.0, 0.5);
                }
                break;

            case X_TIMED_DRIVE:
                if (robot.robotDrive != null)
                {
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, testChoices.driveTime, testChoices.drivePower, 0.0, 0.0);
                }
                break;

            case Y_TIMED_DRIVE:
                if (robot.robotDrive != null)
                {
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, testChoices.driveTime, 0.0, testChoices.drivePower, 0.0);
                }
                break;

            case PID_DRIVE:
                if (robot.robotDrive != null)
                {
                    // Distance targets are in feet, so convert them into inches.
                    testCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.drivePower, null,
                        new TrcPose2D(testChoices.xTarget*12.0, testChoices.yTarget*12.0, testChoices.turnTarget));
                }
                break;

            case TUNE_X_PID:
                if (robot.robotDrive != null)
                {
                    // Distance target is in feet, so convert it into inches.
                    testCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.tuneDrivePower,
                        testChoices.tunePidCoeff, new TrcPose2D(testChoices.tuneDistance*12.0, 0.0, 0.0));
                }
                break;

            case TUNE_Y_PID:
                if (robot.robotDrive != null)
                {
                    // Distance target is in feet, so convert it into inches.
                    testCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.tuneDrivePower,
                        testChoices.tunePidCoeff, new TrcPose2D(0.0, testChoices.tuneDistance*12.0, 0.0));
                }
                break;

            case TUNE_TURN_PID:
                if (robot.robotDrive != null)
                {
                    testCommand = new CmdPidDrive(
                        robot.robotDrive.driveBase, robot.robotDrive.pidDrive, 0.0, testChoices.tuneDrivePower,
                        testChoices.tunePidCoeff, new TrcPose2D(0.0, 0.0, testChoices.tuneHeading));
                }
                break;
        }
        //
        // Only VISION_TEST needs TensorFlow, shut it down for all other tests.
        //
        if (robot.vision != null && robot.vision.tensorFlowVision != null && testChoices.test != Test.VISION_TEST)
        {
            robot.globalTracer.traceInfo(moduleName, "Disabling TensorFlowVision.");
            robot.vision.setTensorFlowVisionEnabled(false);
        }
    }   //robotInit

    //
    // Overrides TrcRobot.RobotMode methods.
    //

    /**
     * This method is called before test mode is about to start so it can initialize appropriate subsystems for the
     * test.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        super.startMode(prevMode, nextMode);
        switch (testChoices.test)
        {
            case VISION_TEST:
                if (robot.vision != null)
                {
                    exposure = robot.vision.vision.getCurrentExposure();
                    // Vision generally will impact performance, so we only enable it if it's needed.
                    if (robot.vision.aprilTagVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling AprilTagVision.");
                        robot.vision.setAprilTagVisionEnabled(true);
                    }

                    if (robot.vision.purplePixelVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling PurplePixelVision.");
                        robot.vision.setPixelVisionEnabled(Vision.PixelType.PurplePixel, true);
                    }

                    if (robot.vision.greenPixelVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling GreenPixelVision.");
                        robot.vision.setPixelVisionEnabled(Vision.PixelType.GreenPixel, true);
                    }

                    if (robot.vision.yellowPixelVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling YellowPixelVision.");
                        robot.vision.setPixelVisionEnabled(Vision.PixelType.YellowPixel, true);
                    }

                    if (robot.vision.whitePixelVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling WhitePixelVision.");
                        robot.vision.setPixelVisionEnabled(Vision.PixelType.WhitePixel, true);
                    }

                    if (robot.vision.redBlobVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling RedBlobVision.");
                        robot.vision.setRedBlobVisionEnabled(true);
                    }

                    if (robot.vision.blueBlobVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling BlueBlobVision.");
                        robot.vision.setBlueBlobVisionEnabled(true);
                    }

                    if (robot.vision.tensorFlowVision != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Enabling TensorFlowVison.");
                        robot.vision.setTensorFlowVisionEnabled(true);
                    }
                }
                break;

            case TUNE_COLORBLOB_VISION:
                if (robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    robot.globalTracer.traceInfo(moduleName, "Enabling FtcRawEocvVision.");
                    robot.vision.setRawColorBlobVisionEnabled(true);
                    colorThresholds = robot.vision.getRawColorBlobThresholds();
                    colorThresholdIndex = 0;
                    colorThresholdMultiplier = 1.0;
                    updateColorThresholds();
                }
                break;

            case PID_DRIVE:
            case TUNE_X_PID:
            case TUNE_Y_PID:
            case TUNE_TURN_PID:
                if (robot.robotDrive != null)
                {
                    robot.robotDrive.pidDrive.setTraceLevel(TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
                }
                break;

            case PURE_PURSUIT_DRIVE:
                if (robot.robotDrive != null)
                {
                    robot.robotDrive.purePursuitDrive.setTraceLevel(
                        TrcDbgTrace.MsgLevel.INFO, logEvents, debugPid, false);
                    //
                    // Doing a 48x48-inch square box with robot heading always pointing to the center of the box.
                    //
                    // Set the current position as the absolute field origin so the path can be an absolute path.
                    TrcPose2D startPose = new TrcPose2D(0.0, 0.0, 0.0);
                    robot.robotDrive.driveBase.setFieldPosition(startPose);
                    robot.robotDrive.purePursuitDrive.start(startPose, false, new TrcPose2D(0.0, 48.0, 90.0));
                }
                break;
        }
    }   //startMode

    /**
     * This method is called before test mode is about to exit so it can do appropriate cleanup.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
        if (testCommand != null)
        {
            testCommand.cancel();
        }

        if (robot.robotDrive != null)
        {
            robot.robotDrive.cancel();
        }

        super.stopMode(prevMode, nextMode);
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        //
        // Run the testCommand if any.
        //
        if (testCommand != null)
        {
            testCommand.cmdPeriodic(elapsedTime);
        }
        //
        // Display test status.
        //
        int lineNum = 9;
        switch (testChoices.test)
        {
            case DRIVE_SPEED_TEST:
                if (robot.robotDrive != null)
                {
                    double currTime = TrcTimer.getCurrentTime();
                    TrcPose2D velPose = robot.robotDrive.driveBase.getFieldVelocity();
                    double velocity = TrcUtil.magnitude(velPose.x, velPose.y);
                    double acceleration = 0.0;

                    if (prevTime != 0.0)
                    {
                        acceleration = (velocity - prevVelocity)/(currTime - prevTime);
                    }

                    if (velocity > maxDriveVelocity)
                    {
                        maxDriveVelocity = velocity;
                    }

                    if (acceleration > maxDriveAcceleration)
                    {
                        maxDriveAcceleration = acceleration;
                    }

                    prevTime = currTime;
                    prevVelocity = velocity;

                    robot.dashboard.displayPrintf(lineNum++, "Drive Vel: (%.1f/%.1f)", velocity, maxDriveVelocity);
                    robot.dashboard.displayPrintf(
                        lineNum++, "Drive Accel: (%.1f/%.1f)", acceleration, maxDriveAcceleration);
                }
                break;

            case X_TIMED_DRIVE:
            case Y_TIMED_DRIVE:
                if (robot.robotDrive != null)
                {
                    robot.dashboard.displayPrintf(lineNum++, "Timed Drive: %.0f sec", testChoices.driveTime);
                    robot.dashboard.displayPrintf(
                        lineNum++, "RobotPose=%s", robot.robotDrive.driveBase.getFieldPosition());
                    robot.dashboard.displayPrintf(
                        lineNum++, "rawEnc=lf:%.0f,rf:%.0f,lb:%.0f,rb:%.0f",
                        robot.robotDrive.driveMotors[RobotDrive.INDEX_LEFT_FRONT].getPosition(),
                        robot.robotDrive.driveMotors[RobotDrive.INDEX_RIGHT_FRONT].getPosition(),
                        robot.robotDrive.driveMotors[RobotDrive.INDEX_LEFT_BACK].getPosition(),
                        robot.robotDrive.driveMotors[RobotDrive.INDEX_RIGHT_BACK].getPosition());
                }
                break;

            case TUNE_X_PID:
            case TUNE_Y_PID:
            case TUNE_TURN_PID:
                if (robot.robotDrive != null && testChoices.tunePidCoeff != null)
                {
                    robot.dashboard.displayPrintf(7, "TunePid=%s", testChoices.tunePidCoeff);
                }
                //
                // Intentionally falling through.
                //
            case PURE_PURSUIT_DRIVE:
            case PID_DRIVE:
                if (robot.robotDrive != null)
                {
                    TrcPidController xPidCtrl, yPidCtrl, turnPidCtrl;
                    if (testChoices.test == Test.PURE_PURSUIT_DRIVE)
                    {
                        xPidCtrl = robot.robotDrive.purePursuitDrive.getXPosPidCtrl();
                        yPidCtrl = robot.robotDrive.purePursuitDrive.getYPosPidCtrl();
                        turnPidCtrl = robot.robotDrive.purePursuitDrive.getTurnPidCtrl();
                    }
                    else
                    {
                        xPidCtrl = robot.robotDrive.pidDrive.getXPidCtrl();
                        yPidCtrl = robot.robotDrive.pidDrive.getYPidCtrl();
                        turnPidCtrl = robot.robotDrive.pidDrive.getTurnPidCtrl();
                    }

                    robot.dashboard.displayPrintf(
                        lineNum++, "RobotPose=%s,rawEnc=lf:%.0f,rf:%.0f,lb:%.0f,rb:%.0f",
                        robot.robotDrive.driveBase.getFieldPosition(),
                        robot.robotDrive.driveMotors[RobotDrive.INDEX_LEFT_FRONT].getPosition(),
                        robot.robotDrive.driveMotors[RobotDrive.INDEX_RIGHT_FRONT].getPosition(),
                        robot.robotDrive.driveMotors[RobotDrive.INDEX_LEFT_BACK].getPosition(),
                        robot.robotDrive.driveMotors[RobotDrive.INDEX_RIGHT_BACK].getPosition());
                    if (xPidCtrl != null)
                    {
                        xPidCtrl.displayPidInfo(lineNum);
                        lineNum += 2;
                    }
                    yPidCtrl.displayPidInfo(lineNum);
                    lineNum += 2;
                    turnPidCtrl.displayPidInfo(lineNum);
                }
                break;
        }

        if (elapsedTimer != null)
        {
            elapsedTimer.recordPeriodTime();
            robot.dashboard.displayPrintf(
                15, "Period: %.3f(%.3f/%.3f)",
                elapsedTimer.getAverageElapsedTime(), elapsedTimer.getMinElapsedTime(),
                elapsedTimer.getMaxElapsedTime());
        }

        if (slowPeriodicLoop)
        {
            if (allowTeleOp())
            {
                //
                // Allow TeleOp to run so we can control the robot in subsystem test or drive speed test modes.
                //
                super.periodic(elapsedTime, true);
            }

            switch (testChoices.test)
            {
                case SENSORS_TEST:
                case SUBSYSTEMS_TEST:
                    doSensorsTest();
                    break;

                case VISION_TEST:
                case TUNE_COLORBLOB_VISION:
                    doVisionTest();
                    break;

                case CALIBRATE_SWERVE_STEERING:
                    if (robot.robotDrive != null && (robot.robotDrive instanceof SwerveDrive) && steerCalibrating)
                    {
                        SwerveDrive swerveDrive = (SwerveDrive) robot.robotDrive;
                        swerveDrive.runSteeringCalibration();
                        if (swerveDrive.calibrationCount > 0)
                        {
                            robot.dashboard.displayPrintf(lineNum++, "Count = %d", swerveDrive.calibrationCount);
                            robot.dashboard.displayPrintf(
                                lineNum++, "Encoder: lf=%.3f/%f",
                                swerveDrive.steerEncoders[SwerveDrive.INDEX_LEFT_FRONT].getRawPosition(),
                                swerveDrive.zeroPositions[SwerveDrive.INDEX_LEFT_FRONT]/swerveDrive.calibrationCount);
                            robot.dashboard.displayPrintf(
                                lineNum++, "Encoder: rf=%.3f/%f",
                                swerveDrive.steerEncoders[SwerveDrive.INDEX_RIGHT_FRONT].getRawPosition(),
                                swerveDrive.zeroPositions[SwerveDrive.INDEX_RIGHT_FRONT]/swerveDrive.calibrationCount);
                            robot.dashboard.displayPrintf(
                                lineNum++, "Encoder: lb=%.3f/%f",
                                swerveDrive.steerEncoders[SwerveDrive.INDEX_LEFT_BACK].getRawPosition(),
                                swerveDrive.zeroPositions[SwerveDrive.INDEX_LEFT_BACK]/swerveDrive.calibrationCount);
                            robot.dashboard.displayPrintf(
                                lineNum++, "Encoder: rb=%.3f/%f",
                                swerveDrive.steerEncoders[SwerveDrive.INDEX_RIGHT_BACK].getRawPosition(),
                                swerveDrive.zeroPositions[SwerveDrive.INDEX_RIGHT_BACK]/swerveDrive.calibrationCount);
                        }
                    }
                    break;

                case TUNE_LAUNCHER_POWER:
                    robot.dashboard.displayPrintf(lineNum++, "LauncherPower=%.3f", launchPower);
                    break;
            }
        }
    }   //periodic

    //
    // Overrides TrcGameController.ButtonHandler in TeleOp.
    //

    /**
     * This method is called when a driver gamepad button event occurs.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    public void driverButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        boolean passToTeleOp = true;
        //
        // In addition to or instead of the gamepad controls handled by FtcTeleOp, we can add to or override the
        // FtcTeleOp gamepad actions.
        //
        robot.dashboard.displayPrintf(8, "%s: %04x->%s", gamepad, button, pressed ? "Pressed" : "Released");
        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
                if (testChoices.test == Test.CALIBRATE_SWERVE_STEERING)
                {
                    if (pressed && robot.robotDrive != null && robot.robotDrive instanceof SwerveDrive)
                    {
                        SwerveDrive swerveDrive = (SwerveDrive) robot.robotDrive;

                        steerCalibrating = !steerCalibrating;
                        if (steerCalibrating)
                        {
                            // Start steer calibration.
                            swerveDrive.startSteeringCalibration();
                        }
                        else
                        {
                            // Stop steer calibration.
                            swerveDrive.stopSteeringCalibration();
                        }
                    }
                    passToTeleOp = false;
                }
                else if ((testChoices.test == Test.TUNE_COLORBLOB_VISION || testChoices.test == Test.VISION_TEST) &&
                         robot.vision != null)
                {
                    // Can only switch camera if we have two.
                    if (pressed && frontWebcam != null && rearWebcam != null)
                    {
                        robot.vision.setActiveWebcam(
                            robot.vision.getActiveWebcam() != frontWebcam? frontWebcam: rearWebcam);
                    }
                    passToTeleOp = false;
                }
                break;

            case FtcGamepad.GAMEPAD_B:
                if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                    robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed)
                    {
                        // Increment to next color threshold index.
                        colorThresholdIndex++;
                        if (colorThresholdIndex >= colorThresholds.length)
                        {
                            colorThresholdIndex = colorThresholds.length - 1;
                        }
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TUNE_LAUNCHER_POWER && robot.launcher != null)
                {
                    robot.launcher.setPower(pressed ? launchPower : 0.0);
                    passToTeleOp = false;
                }
                break;

            case FtcGamepad.GAMEPAD_X:
                if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                    robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed)
                    {
                        // Decrement to previous color threshold index.
                        colorThresholdIndex--;
                        if (colorThresholdIndex < 0)
                        {
                            colorThresholdIndex = 0;
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            case FtcGamepad.GAMEPAD_Y:
                if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                    robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed)
                    {
                        // Set display to next intermediate Mat in the pipeline.
                        robot.vision.rawColorBlobVision.getPipeline().setNextVideoOutput();
                    }
                    passToTeleOp = false;
                }
                break;

            case FtcGamepad.GAMEPAD_LBUMPER:
                if (testChoices.test == Test.VISION_TEST && robot.vision != null)
                {
                    if (pressed)
                    {
                        exposure -= 100;
                        robot.vision.vision.setManualExposure(exposure, null);
                    }
                    passToTeleOp = false;
                }
                break;

            case FtcGamepad.GAMEPAD_RBUMPER:
                if (testChoices.test == Test.VISION_TEST && robot.vision != null)
                {
                    if (pressed)
                    {
                        exposure += 100;
                        robot.vision.vision.setManualExposure(exposure, null);
                    }
                    passToTeleOp = false;
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    // If we are moving swerve steering, make sure TeleOp doesn't interfere.
                    teleOpControlEnabled = !pressed;
                    if (pressed && robot.robotDrive != null && robot.robotDrive instanceof SwerveDrive)
                    {
                        SwerveDrive swerveDrive = (SwerveDrive) robot.robotDrive;
                        swerveDrive.setSteerAngle(0.0, false, true);
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                         robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed &&
                        colorThresholds[colorThresholdIndex] + colorThresholdMultiplier <=
                        COLOR_THRESHOLD_HIGH_RANGES[colorThresholdIndex/2])
                    {
                        // Increment color threshold value.
                        colorThresholds[colorThresholdIndex] += colorThresholdMultiplier;
                        updateColorThresholds();
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TUNE_LAUNCHER_POWER && robot.launcher != null)
                {
                    if (pressed)
                    {
                        launchPower += LAUNCHER_POWER_STEP;
                        if (launchPower > 1.0) launchPower = 1.0;
                    }
                    passToTeleOp = false;
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    // If we are moving swerve steering, make sure TeleOp doesn't interfere.
                    teleOpControlEnabled = !pressed;
                    if (pressed && robot.robotDrive != null && robot.robotDrive instanceof SwerveDrive)
                    {
                        SwerveDrive swerveDrive = (SwerveDrive) robot.robotDrive;
                        swerveDrive.setSteerAngle(180.0, false, true);
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                         robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed &&
                        colorThresholds[colorThresholdIndex] - colorThresholdMultiplier >=
                        COLOR_THRESHOLD_LOW_RANGES[colorThresholdIndex/2])
                    {
                        // Decrement color threshold value.
                        colorThresholds[colorThresholdIndex] -= colorThresholdMultiplier;
                        updateColorThresholds();
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TUNE_LAUNCHER_POWER && robot.launcher != null)
                {
                    if (pressed)
                    {
                        launchPower -= LAUNCHER_POWER_STEP;
                        if (launchPower < 0.0) launchPower = 0.0;
                    }
                    passToTeleOp = false;
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_LEFT:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    // If we are moving swerve steering, make sure TeleOp doesn't interfere.
                    teleOpControlEnabled = !pressed;
                    if (pressed && robot.robotDrive != null && robot.robotDrive instanceof SwerveDrive)
                    {
                        SwerveDrive swerveDrive = (SwerveDrive) robot.robotDrive;
                        swerveDrive.setSteerAngle(270.0, false, true);
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                         robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed && colorThresholdMultiplier * 10.0 <= 100.0)
                    {
                        // Increment the significant multiplier.
                        colorThresholdMultiplier *= 10.0;
                    }
                    passToTeleOp = false;
                }
                break;

            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                if (testChoices.test == Test.SUBSYSTEMS_TEST)
                {
                    // If we are moving swerve steering, make sure TeleOp doesn't interfere.
                    teleOpControlEnabled = !pressed;
                    if (pressed && robot.robotDrive != null && robot.robotDrive instanceof SwerveDrive)
                    {
                        SwerveDrive swerveDrive = (SwerveDrive) robot.robotDrive;
                        swerveDrive.setSteerAngle(90.0, false, true);
                    }
                    passToTeleOp = false;
                }
                else if (testChoices.test == Test.TUNE_COLORBLOB_VISION &&
                         robot.vision != null && robot.vision.rawColorBlobVision != null)
                {
                    if (pressed && colorThresholdMultiplier / 10.0 >= 1.0)
                    {
                        // Decrement the significant multiplier.
                        colorThresholdMultiplier /= 10.0;
                    }
                    passToTeleOp = false;
                }
                break;
        }
        //
        // If the control was not processed by this method, pass it back to TeleOp.
        //
        if (passToTeleOp && allowTeleOp())
        {
            super.driverButtonEvent(gamepad, button, pressed);
        }
    }   //driverButtonEvent

    /**
     * This method is called when an operator gamepad button event occurs.
     *
     * @param gamepad specifies the game controller object that generated the event.
     * @param button specifies the button ID that generates the event
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    public void operatorButtonEvent(TrcGameController gamepad, int button, boolean pressed)
    {
        boolean passToTeleOp = true;
        //
        // In addition to or instead of the gamepad controls handled by FtcTeleOp, we can add to or override the
        // FtcTeleOp gamepad actions.
        //
        robot.dashboard.displayPrintf(8, "%s: %04x->%s", gamepad, button, pressed ? "Pressed" : "Released");
        switch (button)
        {
            case FtcGamepad.GAMEPAD_A:
                break;

            case FtcGamepad.GAMEPAD_B:
                break;

            case FtcGamepad.GAMEPAD_X:
                break;

            case FtcGamepad.GAMEPAD_Y:
                break;

            case FtcGamepad.GAMEPAD_DPAD_UP:
                break;

            case FtcGamepad.GAMEPAD_DPAD_DOWN:
                break;

            case FtcGamepad.GAMEPAD_DPAD_LEFT:
                break;

            case FtcGamepad.GAMEPAD_DPAD_RIGHT:
                break;
        }
        //
        // If the control was not processed by this method, pass it back to TeleOp.
        //
        if (passToTeleOp && allowTeleOp())
        {
            super.operatorButtonEvent(gamepad, button, pressed);
        }
    }   //operatorButtonEvent

    /**
     * This method displays the current color thresholds on the dashboard.
     */
    private void updateColorThresholds()
    {
        robot.dashboard.displayPrintf(7, "Thresholds: %s", Arrays.toString(colorThresholds));
    }   //updateColorThresholds

    /**
     * This method creates and displays the test menus and record the selected choices.
     */
    private void doTestMenus()
    {
        //
        // Create menus.
        //
        testMenu = new FtcChoiceMenu<>("Tests:", null);
        FtcValueMenu xTargetMenu = new FtcValueMenu(
            "xTarget:", testMenu, -10.0, 10.0, 0.5, 0.0, " %.1f ft");
        FtcValueMenu yTargetMenu = new FtcValueMenu(
            "yTarget:", testMenu, -10.0, 10.0, 0.5, 0.0, " %.1f ft");
        FtcValueMenu turnTargetMenu = new FtcValueMenu(
            "turnTarget:", testMenu, -180.0, 180.0, 5.0, 0.0, " %.0f deg");
        FtcValueMenu driveTimeMenu = new FtcValueMenu(
            "Drive time:", testMenu, 1.0, 10.0, 1.0, 4.0, " %.0f sec");
        FtcValueMenu drivePowerMenu = new FtcValueMenu(
            "Drive power:", testMenu, -1.0, 1.0, 0.1, 0.5, " %.1f");
        //
        // PID Tuning menus.
        //
        FtcValueMenu tuneKpMenu = new FtcValueMenu(
            "Kp:", testMenu, 0.0, 1.0, 0.001, this::getTuneKp, " %f");
        FtcValueMenu tuneKiMenu = new FtcValueMenu(
            "Ki:", tuneKpMenu, 0.0, 1.0, 0.001, this::getTuneKi, " %f");
        FtcValueMenu tuneKdMenu = new FtcValueMenu(
            "Kd:", tuneKiMenu, 0.0, 1.0, 0.001, this::getTuneKd, " %f");
        FtcValueMenu tuneKfMenu = new FtcValueMenu(
            "Kf:", tuneKdMenu, 0.0, 1.0, 0.001, this::getTuneKf, " %f");
        FtcValueMenu tuneDistanceMenu = new FtcValueMenu(
            "PID Tune distance:", tuneKfMenu, -10.0, 10.0, 0.5, 0.0,
            " %.1f ft");
        FtcValueMenu tuneHeadingMenu = new FtcValueMenu(
            "PID Tune heading:", tuneDistanceMenu, -180.0, 180.0, 5.0, 0.0,
            " %.0f deg");
        FtcValueMenu tuneDrivePowerMenu = new FtcValueMenu(
            "PID Tune drive power:", tuneHeadingMenu, -1.0, 1.0, 0.1, 1.0,
            " %.1f");
        //
        // Populate menus.
        //
        testMenu.addChoice("Sensors test", Test.SENSORS_TEST, true);
        testMenu.addChoice("Subsystems test", Test.SUBSYSTEMS_TEST, false);
        testMenu.addChoice("Vision test", Test.VISION_TEST, false);
        testMenu.addChoice("Tune ColorBlob vision", Test.TUNE_COLORBLOB_VISION, false);
        testMenu.addChoice("Drive speed test", Test.DRIVE_SPEED_TEST, false);
        testMenu.addChoice("Drive motors test", Test.DRIVE_MOTORS_TEST, false);
        testMenu.addChoice("X Timed drive", Test.X_TIMED_DRIVE, false, driveTimeMenu);
        testMenu.addChoice("Y Timed drive", Test.Y_TIMED_DRIVE, false, driveTimeMenu);
        testMenu.addChoice("PID drive", Test.PID_DRIVE, false, xTargetMenu);
        testMenu.addChoice("Tune X PID", Test.TUNE_X_PID, false, tuneKpMenu);
        testMenu.addChoice("Tune Y PID", Test.TUNE_Y_PID, false, tuneKpMenu);
        testMenu.addChoice("Tune Turn PID", Test.TUNE_TURN_PID, false, tuneKpMenu);
        testMenu.addChoice("Pure Pursuit Drive", Test.PURE_PURSUIT_DRIVE, false);
        testMenu.addChoice("Calibrate Swerve Steering", Test.CALIBRATE_SWERVE_STEERING, false);
        testMenu.addChoice("Tune Launcher Power", Test.TUNE_LAUNCHER_POWER, false);

        xTargetMenu.setChildMenu(yTargetMenu);
        yTargetMenu.setChildMenu(turnTargetMenu);
        turnTargetMenu.setChildMenu(drivePowerMenu);
        driveTimeMenu.setChildMenu(drivePowerMenu);
        tuneKpMenu.setChildMenu(tuneKiMenu);
        tuneKiMenu.setChildMenu(tuneKdMenu);
        tuneKdMenu.setChildMenu(tuneKfMenu);
        tuneKfMenu.setChildMenu(tuneDistanceMenu);
        tuneDistanceMenu.setChildMenu(tuneHeadingMenu);
        tuneHeadingMenu.setChildMenu(tuneDrivePowerMenu);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(testMenu);
        //
        // Fetch choices.
        //
        testChoices.test = testMenu.getCurrentChoiceObject();
        testChoices.xTarget = xTargetMenu.getCurrentValue();
        testChoices.yTarget = yTargetMenu.getCurrentValue();
        testChoices.turnTarget = turnTargetMenu.getCurrentValue();
        testChoices.driveTime = driveTimeMenu.getCurrentValue();
        testChoices.drivePower = drivePowerMenu.getCurrentValue();
        testChoices.tunePidCoeff = new TrcPidController.PidCoefficients(
            tuneKpMenu.getCurrentValue(), tuneKiMenu.getCurrentValue(),
            tuneKdMenu.getCurrentValue(),tuneKfMenu.getCurrentValue());
        testChoices.tuneDistance = tuneDistanceMenu.getCurrentValue();
        testChoices.tuneHeading = tuneHeadingMenu.getCurrentValue();
        testChoices.tuneDrivePower = tuneDrivePowerMenu.getCurrentValue();

        TrcPidController tunePidCtrl = getTunePidController(testChoices.test);
        if (tunePidCtrl != null)
        {
            //
            // Write the user input PID coefficients to a cache file so tune PID menu can read them as start value
            // next time.
            //
            pidCoeffCache.writeCachedPidCoeff(tunePidCtrl, testChoices.tunePidCoeff);
        }
        //
        // Show choices.
        //
        robot.dashboard.displayPrintf(1, "Test Choices: %s", testChoices);
    }   //doTestMenus

    /**
     * This method returns the PID controller for the tune test.
     *
     * @param test specifies the selected test.
     * @return tune PID controller.
     */
    private TrcPidController getTunePidController(Test test)
    {
        TrcPidController pidCtrl;

        switch (test)
        {
            case TUNE_X_PID:
                pidCtrl = robot.robotDrive.pidDrive.getXPidCtrl();
                break;

            case TUNE_Y_PID:
                pidCtrl = robot.robotDrive.pidDrive.getYPidCtrl();
                break;

            case TUNE_TURN_PID:
                pidCtrl = robot.robotDrive.pidDrive.getTurnPidCtrl();
                break;

            default:
                pidCtrl = null;
        }

        return pidCtrl;
    }   //getTunePidController

    /**
     * This method is called by the tuneKpMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Kp value of the PID controller being tuned.
     */
    private double getTuneKp()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kP;
        }

        return value;
    }   //getTuneKp

    /**
     * This method is called by the tuneKiMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Ki value of the PID controller being tuned.
     */
    private double getTuneKi()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kI;
        }

        return value;
    }   //getTuneKi

    /**
     * This method is called by the tuneKdMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Kd value of the PID controller being tuned.
     */
    private double getTuneKd()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kD;
        }

        return value;
    }   //getTuneKd

    /**
     * This method is called by the tuneKfMenu to get the start value to be displayed as the current value of the menu.
     *
     * @return start Kf value of the PID controller being tuned.
     */
    double getTuneKf()
    {
        double value = 0.0;
        TrcPidController tunePidCtrl = getTunePidController(testMenu.getCurrentChoiceObject());

        if (tunePidCtrl != null)
        {
            value = pidCoeffCache.getCachedPidCoeff(tunePidCtrl).kF;
        }

        return value;
    }   //getTuneKF

    /**
     * This method reads all sensors and prints out their values. This is a very useful diagnostic tool to check
     * if all sensors are working properly. For encoders, since test sensor mode is also teleop mode, you can
     * operate the gamepads to turn the motors and check the corresponding encoder counts.
     */
    private void doSensorsTest()
    {
        int lineNum = 9;
        //
        // Read all sensors and display on the dashboard.
        // Drive the robot around to sample different locations of the field.
        //
        if (robot.robotDrive != null)
        {
            robot.dashboard.displayPrintf(
                lineNum++, "DriveEnc: lf=%.0f,rf=%.0f,lb=%.0f,rb=%.0f",
                robot.robotDrive.driveMotors[RobotDrive.INDEX_LEFT_FRONT].getPosition(),
                robot.robotDrive.driveMotors[RobotDrive.INDEX_RIGHT_FRONT].getPosition(),
                robot.robotDrive.driveMotors[RobotDrive.INDEX_LEFT_BACK].getPosition(),
                robot.robotDrive.driveMotors[RobotDrive.INDEX_RIGHT_BACK].getPosition());

            if (robot.robotDrive instanceof SwerveDrive)
            {
                SwerveDrive swerveDrive = (SwerveDrive) robot.robotDrive;
                robot.dashboard.displayPrintf(
                    lineNum++, "SteerEnc: lf=%.2f, rf=%.2f, lb=%.2f, rb=%.2f",
                    swerveDrive.steerEncoders[RobotDrive.INDEX_LEFT_FRONT].getPosition(),
                    swerveDrive.steerEncoders[RobotDrive.INDEX_RIGHT_FRONT].getPosition(),
                    swerveDrive.steerEncoders[RobotDrive.INDEX_LEFT_BACK].getPosition(),
                    swerveDrive.steerEncoders[RobotDrive.INDEX_RIGHT_BACK].getPosition());
                robot.dashboard.displayPrintf(
                    lineNum++, "SteerRaw: lf=%.2f, rf=%.2f, lb=%.2f, rb=%.2f",
                    swerveDrive.steerEncoders[RobotDrive.INDEX_LEFT_FRONT].getRawPosition(),
                    swerveDrive.steerEncoders[RobotDrive.INDEX_RIGHT_FRONT].getRawPosition(),
                    swerveDrive.steerEncoders[RobotDrive.INDEX_LEFT_BACK].getRawPosition(),
                    swerveDrive.steerEncoders[RobotDrive.INDEX_RIGHT_BACK].getRawPosition());
            }

            if (robot.robotDrive.gyro != null)
            {
                robot.dashboard.displayPrintf(
                    lineNum++, "Gyro(x,y,z): Heading=(%.1f,%.1f,%.1f), Rate=(%.3f,%.3f,%.3f)",
                    robot.robotDrive.gyro.getXHeading().value, robot.robotDrive.gyro.getYHeading().value,
                    robot.robotDrive.gyro.getZHeading().value, robot.robotDrive.gyro.getXRotationRate().value,
                    robot.robotDrive.gyro.getYRotationRate().value, robot.robotDrive.gyro.getZRotationRate().value);
            }
        }
    }   //doSensorsTest

    /**
     * This method calls vision code to detect target objects and display their info.
     */
    private void doVisionTest()
    {
        if (robot.vision != null)
        {
            int lineNum = 9;

//            robot.vision.displayExposureSettings(lineNum++);
            if (robot.vision.rawColorBlobVision != null)
            {
                robot.vision.getDetectedRawColorBlob(lineNum++);
            }

            if (robot.vision.aprilTagVision != null)
            {
                robot.vision.getDetectedAprilTag(null, lineNum++);
            }

            if (robot.vision.redBlobVision != null)
            {
                robot.vision.getDetectedTeamPropPosition(FtcAuto.Alliance.RED_ALLIANCE, lineNum++);
            }

            if (robot.vision.blueBlobVision != null)
            {
                robot.vision.getDetectedTeamPropPosition(FtcAuto.Alliance.BLUE_ALLIANCE, lineNum++);
            }


        }
    }   //doVisionTest

    /**
     * This method is called to determine if Test mode is allowed to do teleop control of the robot.
     *
     * @return true to allow and false otherwise.
     */
    private boolean allowTeleOp()
    {
        return teleOpControlEnabled &&
               (testChoices.test == Test.SUBSYSTEMS_TEST || testChoices.test == Test.DRIVE_SPEED_TEST);
    }   //allowTeleOp

}   //class FtcTest
