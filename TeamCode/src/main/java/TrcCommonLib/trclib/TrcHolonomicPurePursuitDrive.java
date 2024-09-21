/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package TrcCommonLib.trclib;

import org.apache.commons.math3.linear.RealVector;

/**
 * This class implements a platform independent Pure Pursuit drive for holonomic robots.
 * Essentially, a pure pursuit drive navigates the robot to chase a point along the path. The point to chase is
 * chosen by intersecting a proximity circle centered on the robot with a specific radius with the path, and chasing
 * the "furthest" intersection. The smaller the radius is, the more "tightly" the robot will follow a path, but it
 * will be more prone to oscillation and sharp turns. A larger radius will tend to smooth out turns and corners. Note
 * that the error tolerance must be less than the proximity radius, so choose them accordingly.
 * <p>
 * A path consists of an array of waypoints, specifying position, velocity, and optionally heading. All other properties
 * of the TrcWaypoint object may be ignored.The path may be low resolution, as this automatically interpolates between
 * waypoints. If you want the robot to maintain heading, call setMaintainHeading(true) and it will ignore all the
 * heading values. Otherwise, call setMaintainHeading(false), ensure that the heading tolerance and pid coefficients
 * are set, and it will follow the heading values specified by the path.
 * <p>
 * A somewhat similar idea is here:
 * <a href="https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552">...</a>
 * or <a href="https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf">...</a>
 * <p>
 * Note that this paper is for non-holonomic robots. This means that all the turning radius stuff isn't very relevant.
 * Technically, we could impose limits on the turning radius as a function of robot velocity and max rot vel, but that's
 * unnecessarily complicated, in my view. Additionally, it does point injection instead of interpolation, and path
 * smoothing, which we don't do, since a nonzero proximity radius will naturally smooth it anyway.
 */
public class TrcHolonomicPurePursuitDrive
{
    public interface WaypointEventHandler
    {
        /**
         * This method is called when Pure Pursuit crosses a waypoint or the path is completed.
         *
         * @param index specifies the index of the waypoint in the path, -1 if the path is completed or canceled.
         * @param waypoint specifies the current target waypoint.
         */
        void waypointEvent(int index, TrcWaypoint waypoint);
    }   //interface WaypointEventHandler

    public enum InterpolationType
    {
        LINEAR(1), QUADRATIC(2), CUBIC(3), QUARTIC(4), QUADRATIC_INV(2), CUBIC_INV(3), QUARTIC_INV(4);

        private final int value;

        InterpolationType(int value)
        {
            this.value = value;
        }   //InterpolationType

        public int getValue()
        {
            return value;
        }   //getValue

    }   //enum InterpolationType

    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcDriveBase driveBase;
    private final TrcTaskMgr.TaskObject driveTaskObj;
    private final TrcPidController posPidCtrl, turnPidCtrl, velPidCtrl;
    // Tracer config.
    private boolean logRobotPoseEvents = false;
    private boolean tracePidInfo = false;
    private boolean verbosePidInfo = false;
    private TrcRobotBattery battery = null;
    private volatile double posTolerance; // Volatile so it can be changed at runtime
    private volatile double proximityRadius; // Volatile so it can be changed at runtime
    private TrcPath path;
    private int pathIndex = 1;
    private double positionInput;
    private TrcEvent onFinishedEvent;
    private double timedOutTime;
    private final TrcWarpSpace warpSpace;
    private WaypointEventHandler waypointEventHandler = null;
    private InterpolationType interpolationType = InterpolationType.LINEAR;
    private volatile boolean maintainHeading = false;
    private double startHeading;
    private TrcPose2D referencePose;
    private double moveOutputLimit = Double.POSITIVE_INFINITY;
    private double rotOutputLimit = Double.POSITIVE_INFINITY;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveBase specifies the reference to the drive base.
     * @param proximityRadius specifies the distance between the robot and next following point.
     * @param posTolerance specifies the position tolerance.
     * @param turnTolerance specifies the turn tolerance.
     * @param posPidCoeff specifies the position PID coefficients.
     * @param turnPidCoeff specifies the turn PID coefficients.
     * @param velPidCoeff specifies the velocity PID coefficients.
     */
    public TrcHolonomicPurePursuitDrive(String instanceName, TrcDriveBase driveBase, double proximityRadius,
        double posTolerance, double turnTolerance, TrcPidController.PidCoefficients posPidCoeff,
        TrcPidController.PidCoefficients turnPidCoeff, TrcPidController.PidCoefficients velPidCoeff)
    {
        if (driveBase.supportsHolonomicDrive())
        {
            this.driveBase = driveBase;
        }
        else
        {
            throw new IllegalArgumentException(
                "Only holonomic drive bases supported for this pure pursuit implementation!");
        }

        this.tracer = new TrcDbgTrace(instanceName);
        this.instanceName = instanceName;
        warpSpace = new TrcWarpSpace(instanceName + ".warpSpace", 0.0, 360.0);
        setPositionToleranceAndProximityRadius(posTolerance, proximityRadius);

        this.posPidCtrl = new TrcPidController(instanceName + ".posPid", posPidCoeff, 0.0, this::getPositionInput);
        this.turnPidCtrl = new TrcPidController(instanceName + ".turnPid", turnPidCoeff, turnTolerance,
            driveBase::getHeading);
        this.velPidCtrl = new TrcPidController(instanceName + ".velPid", velPidCoeff, 0.0, this::getVelocityInput);

        posPidCtrl.setAbsoluteSetPoint(true);
        turnPidCtrl.setAbsoluteSetPoint(true);
        velPidCtrl.setAbsoluteSetPoint(true);

        turnPidCtrl.setNoOscillation(true);

        this.driveTaskObj = TrcTaskMgr.createTask(instanceName + ".driveTask", this::driveTask);
    }   //TrcHolonomicPurePursuitDrive

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param driveBase specifies the reference to the drive base.
     * @param proximityRadius specifies the distance between the robot and next following point.
     * @param posTolerance specifies the position tolerance.
     * @param posPidCoeff specifies the position PID coefficients.
     * @param turnPidCoeff specifies the turn PID coefficients.
     * @param velPidCoeff specifies the velocity PID coefficients.
     */
    public TrcHolonomicPurePursuitDrive(String instanceName, TrcDriveBase driveBase, double proximityRadius,
        double posTolerance, TrcPidController.PidCoefficients posPidCoeff,
        TrcPidController.PidCoefficients turnPidCoeff, TrcPidController.PidCoefficients velPidCoeff)
    {
        this(instanceName, driveBase, proximityRadius, posTolerance, 5.0, posPidCoeff, turnPidCoeff, velPidCoeff);
        setMaintainHeading(true);
    }   //TrcHolonomicPurePursuitDrive

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method sets the trace level for logging trace messages.
     *
     * @param msgLevel specifies the message level.
     * @param logRobotPoseEvents specifies true to log robot pose events, false otherwise.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     * @param verbosePidInfo specifies true to trace verbose PID info, false otherwise.
     * @param battery specifies the battery object to get battery info for the message.
     */
    public synchronized void setTraceLevel(
        TrcDbgTrace.MsgLevel msgLevel, boolean logRobotPoseEvents, boolean tracePidInfo, boolean verbosePidInfo,
        TrcRobotBattery battery)
    {
        tracer.setTraceLevel(msgLevel);
        this.logRobotPoseEvents = logRobotPoseEvents;
        this.tracePidInfo = tracePidInfo;
        this.verbosePidInfo = verbosePidInfo;
        this.battery = battery;
    }   //setTraceLevel

    /**
     * This method sets the trace level for logging trace messages.
     *
     * @param msgLevel specifies the message level.
     * @param logRobotPoseEvents specifies true to log robot pose events, false otherwise.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     * @param verbosePidInfo specifies true to trace verbose PID info, false otherwise.
     */
    public void setTraceLevel(
        TrcDbgTrace.MsgLevel msgLevel, boolean logRobotPoseEvents, boolean tracePidInfo, boolean verbosePidInfo)
    {
        setTraceLevel(msgLevel, logRobotPoseEvents, tracePidInfo, verbosePidInfo, null);
    }   //setTraceLevel

    /**
     * This method returns the field position of the target waypoint of the path (i.e. the last waypoint in the path).
     *
     * @return field position of the last waypoint in the path.
     */
    public TrcPose2D getTargetFieldPosition()
    {
        TrcPose2D targetPose = null;

        if (referencePose != null && path != null && path.getSize() > 0)
        {
            targetPose = referencePose.addRelativePose(path.getLastWaypoint().pose);
        }

        return targetPose;
    }   //getTargetFieldPosition

    /**
     * This method sets the waypoint event handler that gets called when the robot crosses each waypoint. This allows
     * the caller to perform actions when each waypoint is reached. Waypoint handler is cleared when the start method
     * is called. In other words, this method should only be called after the start method is called and the Waypoint
     * event handler is only valid for the path started by the start method.
     *
     * @param handler specifies the waypoint event handler, can be null to clear the event handler.
     */
    public synchronized void setWaypointEventHandler(WaypointEventHandler handler)
    {
        this.waypointEventHandler = handler;
    }   //setWaypointEventHandler

    /**
     * Maintain heading during path following, or follow the heading values in the path. If not maintaining heading,
     * remember to set the heading tolerance!
     *
     * @param maintainHeading If true, maintain heading. If false, use closed loop to control heading.
     */
    public void setMaintainHeading(boolean maintainHeading)
    {
        this.maintainHeading = maintainHeading;
    }   //setMaintainHeading

    /**
     * Set the turn tolerance for the closed loop control on turning. Only applicable if not maintaining heading.
     *
     * @param turnTolerance The turn tolerance, in degrees. Should be positive.
     */
    public void setTurnTolerance(double turnTolerance)
    {
        turnPidCtrl.setTargetTolerance(turnTolerance);
    }   //setTurnTolerance

    /**
     * Configure the method of interpolating between waypoints. Methods ending with INV will favor the ending point.
     *
     * @param interpolationType The type of interpolation to use.
     */
    public void setInterpolationType(InterpolationType interpolationType)
    {
        this.interpolationType = interpolationType == null ? InterpolationType.LINEAR : interpolationType;
    }   //setInterpolationType

    /**
     * Set both the position tolerance and proximity radius.
     *
     * @param posTolerance    sepcifies the distance at which the controller will stop itself.
     * @param proximityRadius specifies the distance between the robot and next following point.
     */
    public void setPositionToleranceAndProximityRadius(double posTolerance, double proximityRadius)
    {
        if (posTolerance >= proximityRadius)
        {
            throw new IllegalArgumentException("Position tolerance must be less than proximityRadius!");
        }

        this.proximityRadius = proximityRadius;
        this.posTolerance = posTolerance;
    }   //setPositionToleranceAndProximityRadius

    /**
     * Set the position tolerance to end the path. Units need to be consistent.
     *
     * @param posTolerance The distance at which the controller will stop itself.
     */
    public void setPositionTolerance(double posTolerance)
    {
        setPositionToleranceAndProximityRadius(posTolerance, proximityRadius);
    }   //setPositionTolerance

    /**
     * Set the following distance for the pure pursuit controller.
     *
     * @param proximityRadius specifies the distance between the robot and next following point.
     */
    public void setProximityRadius(double proximityRadius)
    {
        setPositionToleranceAndProximityRadius(posTolerance, proximityRadius);
    }   //setProximityRadius

    /**
     * Sets the pid coefficients for the position controller. This will work in the middle of an operation as well.
     *
     * @param pidCoefficients The new PID coefficients for the position controller.
     */
    public void setPositionPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        posPidCtrl.setPidCoefficients(pidCoefficients);
    }   //setPositionPidCoefficients

    /**
     * Sets the pid coefficients for the turn controller. This will work in the middle of an operation as well.
     *
     * @param pidCoefficients The new PID coefficients for the heading controller.
     */
    public void setTurnPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        turnPidCtrl.setPidCoefficients(pidCoefficients);
    }   //setTurnPidCoefficients

    /**
     * Sets the pid coefficients for the position controller. This will work in the middle of an operation as well.
     * Note that velocity controllers should have an F term as well.
     *
     * @param pidCoefficients The new PIDF coefficients for the velocity controller.
     */
    public void setVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoefficients)
    {
        velPidCtrl.setPidCoefficients(pidCoefficients);
    }   //setVelocityPidCoefficients

    /**
     * Sets the movement output power limit.
     *
     * @param limit specifies the output power limit for movement (X and Y).
     */
    public void setMoveOutputLimit(double limit)
    {
        moveOutputLimit = Math.abs(limit);
    }   //setMoveOutputLimit

    /**
     * Sets the rotation output power limit.
     *
     * @param limit specifies the output power limit for rotation.
     */
    public void setRotOutputLimit(double limit)
    {
        rotOutputLimit = Math.abs(limit);
    }   //setRotOutputLimit

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param onFinishedEvent When finished, signal this event.
     * @param timeout         Number of seconds after which to cancel this operation. 0.0 for no timeout.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     */
    public synchronized void start(
        TrcPath path, TrcEvent onFinishedEvent, double timeout, Double maxVel, Double maxAccel)
    {
        if (path == null || path.getSize() == 0)
        {
            throw new IllegalArgumentException("Path cannot be null or empty!");
        }

        cancel();

        if (onFinishedEvent != null)
        {
            onFinishedEvent.clear();
        }
        this.onFinishedEvent = onFinishedEvent;

        this.path = maxVel != null && maxAccel != null? path.trapezoidVelocity(maxVel, maxAccel): path;

        timedOutTime = timeout == 0.0 ? Double.POSITIVE_INFINITY : TrcTimer.getCurrentTime() + timeout;
        pathIndex = 1;
        positionInput = 0;
        startHeading = driveBase.getHeading();

        posPidCtrl.reset();
        turnPidCtrl.reset();
        velPidCtrl.reset();

        posPidCtrl.setTarget(0.0);
        turnPidCtrl.setTarget(startHeading); // Maintain heading to start

        referencePose = driveBase.getFieldPosition();
        driveTaskObj.registerTask(TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param onFinishedEvent When finished, signal this event.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     */
    public void start(TrcPath path, TrcEvent onFinishedEvent, Double maxVel, Double maxAccel)
    {
        start(path, onFinishedEvent, 0.0, maxVel, maxAccel);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path The path to follow. Must start at (0,0).
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     */
    public void start(TrcPath path, Double maxVel, Double maxAccel)
    {
        start(path, null, 0.0, maxVel, maxAccel);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param onFinishedEvent When finished, signal this event.
     * @param timeout         Number of seconds after which to cancel this operation. 0.0 for no timeout.
     */
    public synchronized void start(TrcPath path, TrcEvent onFinishedEvent, double timeout)
    {
        start(path, onFinishedEvent, timeout, null, null);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path            The path to follow. Must start at (0,0).
     * @param onFinishedEvent When finished, signal this event.
     */
    public void start(TrcPath path, TrcEvent onFinishedEvent)
    {
        start(path, onFinishedEvent, 0.0, null, null);
    }   //start

    /**
     * Start following the supplied path using a pure pursuit controller. The velocity must always be positive, and
     * the path must start at (0,0). Heading is absolute and position is relative in the starting robot reference frame.
     *
     * @param path The path to follow. Must start at (0,0).
     */
    public void start(TrcPath path)
    {
        start(path, null, 0.0, null, null);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param onFinishedEvent When finished, signal this event.
     * @param timeout specifies the maximum time allowed for this operation, 0.0 for no timeout.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
            TrcEvent onFinishedEvent, double timeout, TrcPose2D startingPose, boolean incrementalPath,
            Double maxVel, Double maxAccel, TrcPose2D... poses)
    {
        TrcPathBuilder pathBuilder = new TrcPathBuilder(startingPose, incrementalPath);

        for (TrcPose2D pose: poses)
        {
            pathBuilder.append(pose);
        }

        start(pathBuilder.toRelativeStartPath(), onFinishedEvent, timeout, maxVel, maxAccel);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param onFinishedEvent When finished, signal this event.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcEvent onFinishedEvent, TrcPose2D startingPose, boolean incrementalPath, Double maxVel, Double maxAccel,
        TrcPose2D... poses)
    {
        start(onFinishedEvent, 0.0, startingPose, incrementalPath, maxVel, maxAccel, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param maxVel          specifies the maximum velocity if applying trapezoid velocity profile, null if not.
     * @param maxAccel        specifies the maximum acceleration if applying trapezoid velocity profile, null if not.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcPose2D startingPose, boolean incrementalPath, Double maxVel, Double maxAccel, TrcPose2D... poses)
    {
        start(null, 0.0, startingPose, incrementalPath, maxVel, maxAccel, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param onFinishedEvent When finished, signal this event.
     * @param timeout specifies the maximum time allowed for this operation, 0.0 for no timeout.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(
        TrcEvent onFinishedEvent, double timeout, TrcPose2D startingPose, boolean incrementalPath,
        TrcPose2D... poses)
    {
        TrcPathBuilder pathBuilder = new TrcPathBuilder(startingPose, incrementalPath);

        for (TrcPose2D pose: poses)
        {
            pathBuilder.append(pose);
        }

        start(pathBuilder.toRelativeStartPath(), onFinishedEvent, timeout, null, null);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param onFinishedEvent When finished, signal this event.
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(TrcEvent onFinishedEvent, TrcPose2D startingPose, boolean incrementalPath, TrcPose2D... poses)
    {
        start(onFinishedEvent, 0.0, startingPose, incrementalPath, null, null, poses);
    }   //start

    /**
     * This method starts the Pure Pursuit drive with the specified poses in the drive path.
     *
     * @param startingPose specifies the starting pose at the beginning of the path.
     * @param incrementalPath specifies true if appending point is relative to the previous point in the path,
     *                        false if appending point is in the same reference frame as startingPose.
     * @param poses specifies an array of waypoint poses in the drive path.
     */
    public void start(TrcPose2D startingPose, boolean incrementalPath, TrcPose2D... poses)
    {
        start(null, 0.0, startingPose, incrementalPath, null, null, poses);
    }   //start

    /**
     * Checks if the robot is currently following a path.
     *
     * @return True if the pure pursuit controller is active, false otherwise.
     */
    public synchronized boolean isActive()
    {
        return driveTaskObj.isRegistered();
    }   //isActive

    /**
     * If the controller is currently active, cancel the path following operation. Otherwise, do nothing.
     * If there is an event to signal, mark it as cancelled.
     */
    public synchronized void cancel()
    {
        if (isActive())
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.cancel();
            }
            stop();
            //
            // Either the path is done or canceled, call the event handler one last time with index -1 and be done.
            //
            if (waypointEventHandler != null)
            {
                waypointEventHandler.waypointEvent(-1, null);
                waypointEventHandler = null;
            }
        }
    }   //cancel

    /**
     * This method is called by the Position PID controller to get the current position of the robot. Since the
     * position target is always zero, getPositionInput returns the negative distance of the robot from the next
     * target point.
     *
     * @return negative distance of the robot from the next target point.
     */
    private double getPositionInput()
    {
        return positionInput;
    }   //getPositionInput

    /**
     * This method is called by the Velocity PID controller to get the polar magnitude of the robot's velocity.
     *
     * @return robot's velocity magnitude.
     */
    private double getVelocityInput()
    {
        return TrcUtil.magnitude(driveBase.getXVelocity(), driveBase.getYVelocity());
    }   //getVelocityInput

    /**
     * Stops PurePursuit drive.
     */
    private synchronized void stop()
    {
        driveTaskObj.unregisterTask();
        driveBase.stop();
    }   //stop

    /**
     * This task is called periodically to calculate the next target point on the path. The next target point on
     * the path has a distance of followDistance from the current robot position intersecting with the path segment
     * towards the end of the endpoint of the path segment.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is about to end (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private synchronized void driveTask(
        TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        TrcPose2D pose = driveBase.getPositionRelativeTo(referencePose, false);
        double robotX = pose.x;
        double robotY = pose.y;
        TrcWaypoint point = getFollowingPoint(pose);

        if (waypointEventHandler != null)
        {
            waypointEventHandler.waypointEvent(pathIndex - 1, point);
        }

        double dist = TrcUtil.magnitude(robotX - point.pose.x, robotY - point.pose.y);
        positionInput = -dist; // Make this negative so the control effort is positive.
        velPidCtrl.setTarget(point.velocity);
        // Only follow heading if we're not maintaining heading
        if (!maintainHeading)
        {
            turnPidCtrl.setTarget(warpSpace.getOptimizedTarget(point.pose.angle, pose.angle));
        }

        double posPower = posPidCtrl.getOutput();
        double turnPower = turnPidCtrl.getOutput();
        double velPower = velPidCtrl.getOutput();
        turnPower = TrcUtil.clipRange(turnPower, -rotOutputLimit, rotOutputLimit);

        double r = posPower + velPower;
        r = TrcUtil.clipRange(r, -moveOutputLimit, moveOutputLimit);
        double theta = Math.toDegrees(Math.atan2(point.pose.x - robotX, point.pose.y - robotY));

        double velocity = TrcUtil.magnitude(driveBase.getXVelocity(), driveBase.getYVelocity());

        tracer.traceDebug(
            instanceName, "RobotPose=%s, RobotVel=%f, TargetPose=%s, TargetVel=%f, pathIndex=%d, r=%f, theta=%f",
            pose, velocity, point.pose, point.velocity, pathIndex, r, theta);
        // If we have timed out or finished, stop the operation.
        boolean timedOut = TrcTimer.getCurrentTime() >= timedOutTime;
        boolean posOnTarget = dist <= posTolerance;
        boolean headingOnTarget = maintainHeading || (!maintainHeading && turnPidCtrl.isOnTarget());
        if (timedOut || (pathIndex == path.getSize() - 1 && posOnTarget && headingOnTarget))
        {
            if (onFinishedEvent != null)
            {
                onFinishedEvent.signal();
            }
            stop();
        }
        else
        {
            driveBase.holonomicDrive_Polar(r, theta, turnPower, pose.angle - startHeading);
        }

        if (logRobotPoseEvents)
        {
            tracer.logEvent(instanceName, "RobotPose", "pose=\"" + driveBase.getFieldPosition() + "\"");
        }

        if (tracePidInfo)
        {
            posPidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
            velPidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
            turnPidCtrl.printPidInfo(tracer, verbosePidInfo, battery);
        }
    }   //driveTask

    /**
     * Interpolates a waypoint that's weighted between two given waypoints.
     *
     * @param point1 specifies the start point of the path segment.
     * @param point2 specifies the end point of the path segment.
     * @param weight specifies the weight between the two provided points.
     * @return weighted interpolated waypoint.
     */
    private TrcWaypoint interpolate(TrcWaypoint point1, TrcWaypoint point2, double weight)
    {
        double timestep = interpolate(point1.timeStep, point2.timeStep, weight);
        double x = interpolate(point1.pose.x, point2.pose.x, weight);
        double y = interpolate(point1.pose.y, point2.pose.y, weight);
        double position = interpolate(point1.encoderPosition, point2.encoderPosition, weight);
        double velocity = interpolate(point1.velocity, point2.velocity, weight);
        double acceleration = interpolate(point1.acceleration, point2.acceleration, weight);
        double jerk = interpolate(point1.jerk, point2.jerk, weight);
        double heading = interpolate(
                point1.pose.angle, warpSpace.getOptimizedTarget(point2.pose.angle, point1.pose.angle), weight);
        return new TrcWaypoint(timestep, new TrcPose2D(x, y, heading), position, velocity, acceleration, jerk);
    }   //interpolate

    /**
     * Returns a weighted value between given values.
     *
     * @param start specifies the start value.
     * @param end specifies the end value.
     * @param weight specifies the weight between the values.
     * @return weighted value between the given values.
     */
    private double interpolate(double start, double end, double weight)
    {
        if (!TrcUtil.inRange(weight, 0.0, 1.0))
        {
            throw new IllegalArgumentException("Weight must be in range [0,1]!");
        }

        switch (interpolationType)
        {
            case LINEAR:
            case QUADRATIC:
            case CUBIC:
            case QUARTIC:
                weight = Math.pow(weight, interpolationType.getValue());
                break;

            case QUADRATIC_INV:
            case CUBIC_INV:
            case QUARTIC_INV:
                weight = Math.pow(weight, 1.0 / interpolationType.getValue());
                break;
        }

        return (1.0 - weight) * start + weight * end;
    }   //interpolate

    /**
     * This method calculates the waypoint on the path segment that intersects the robot's proximity circle that is
     * closest to the end point of the path segment. The algorithm is based on this article:
     * <a href="https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm">...</a>
     *
     * @param prev specifies the start point of the path segment.
     * @param point specifies the end point of the path segment.
     * @param robotPose specifies the robot's position.
     * @return calculated waypoint.
     */
    private TrcWaypoint getFollowingPointOnSegment(TrcWaypoint prev, TrcWaypoint point, TrcPose2D robotPose)
    {
        // Find intersection of path segment with the proximity circle of the robot.
        RealVector start = prev.getPositionPose().toPosVector();
        RealVector end = point.getPositionPose().toPosVector();
        RealVector robot = robotPose.toPosVector();

        RealVector startToEnd = end.subtract(start);
        RealVector robotToStart = start.subtract(robot);
        // Solve quadratic formula
        double a = startToEnd.dotProduct(startToEnd);
        double b = 2 * robotToStart.dotProduct(startToEnd);
        double c = robotToStart.dotProduct(robotToStart) - proximityRadius * proximityRadius;

        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0)
        {
            // No valid intersection.
            return null;
        }
        else
        {
            // line is a parametric equation, where t=0 is start waypoint, t=1 is end waypoint of the line segment.
            discriminant = Math.sqrt(discriminant);
            //
            // t1 and t2 represent the relative positions of the intersection points on the line segment. If they are
            // in the range of 0.0 and 1.0, they are on the line segment. Otherwise, the intersection points are
            // outside of the line segment. If the relative position is towards 0.0, it is closer to the start
            // waypoint of the line segment. If the relative position is towards 1.0, it is closer to the end
            // waypoint of the line segment.
            //
            // t represents the furthest intersection point (the one closest to the end waypoint of the line segment).
            //
            double t1 = (-b - discriminant) / (2 * a);
            double t2 = (-b + discriminant) / (2 * a);
            double t = Math.max(t1, t2);

            if (!TrcUtil.inRange(t, 0.0, 1.0))
            {
                //
                // The furthest intersection point is not on the line segment, so skip this segment.
                //
                return null;
            }

            return interpolate(prev, point, t);
        }
    }   //getFollowingPointOnSegment

    /**
     * Determines the next target point for Pure Pursuit Drive to follow.
     *
     * @param robotPose specifies the robot's location.
     * @return next target point for the robot to follow.
     */
    private TrcWaypoint getFollowingPoint(TrcPose2D robotPose)
    {
        //
        // Find the next segment that intersects with the proximity circle of the robot.
        // If there are tiny segments that are completely within the proximity circle, we will skip them all.
        //
        for (int i = Math.max(pathIndex, 1); i < path.getSize(); i++)
        {
            // If there is a valid intersection, return it.
            TrcWaypoint interpolated = getFollowingPointOnSegment(
                path.getWaypoint(i - 1), path.getWaypoint(i), robotPose);
            if (interpolated != null)
            {
                pathIndex = i;
                return interpolated;
            }
        }
        //
        // Found no intersection. The robot must be off-path. Just proceed to the immediate next waypoint.
        //
        return path.getWaypoint(pathIndex);
    }   //getFollowingPoint

}   //class TrcHolonomicPurePursuitDrive
