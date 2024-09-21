/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

package ftclib;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import TrcCommonLib.trclib.TrcServo;
import TrcCommonLib.trclib.TrcTimer;

/**
 * This class implements a platform dependent servo extending TrcServo. It provides implementation of the abstract
 * methods in TrcServo.
 */
public class FtcServo extends TrcServo
{
    private final Servo servo;
    private final ServoController controller;
    private final TrcTimer timer;
    private Double logicalPos;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcServo(HardwareMap hardwareMap, String instanceName)
    {
        super(instanceName);
        servo = hardwareMap.get(Servo.class, instanceName);
        controller = servo.getController();
        timer = new TrcTimer(instanceName);
        logicalPos = null;
    }   //FtcServo

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcServo(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName);
    }   //FtcServo

    /**
     * This method returns the servo controller object that this servo is plugged into.
     *
     * @return servo controller object.
     */
    public ServoController getController()
    {
        return controller;
    }   //getController

    /**
     * This method cancels the holdTimer and stops the state machine if it is running.
     */
    public void cancel()
    {
        timer.cancel();
    }   //cancel

    /**
     * This method sets the servo position but will cut power to the servo when done. Since servo motors can't really
     * take a lot of loads, it would stress out and may burn out the servo if it is held against a heavy load for
     * extended period of time. This method allows us to set the position and only hold it long enough for it to
     * reach target position and then we will cut the servo controller power off. Note that by doing so, all servos
     * on the same controller will go limp.
     *
     * @param pos specifies the target position.
     * @param onTime specifies the time in seconds to wait before disabling servo controller.
     */
    public void setPositionWithOnTime(double pos, double onTime)
    {
        cancel();
        setPosition(pos);
        timer.set(onTime, this::onTimeExpired);
    }   //setPositionWithOnTime

    /**
     * This method is called when the onTime has expired. It disables the servo controller.
     *
     * @param context specifies callback context (not used).
     */
    private void onTimeExpired(Object context)
    {
        controller.pwmDisable();
    }   //onTimeExpired

    /**
     * The method enables/disables the servo controller. If the servo controller is disabled, all servos on the
     * controller will go limp. This is useful for preventing the servos from burning up if it is held against
     * a heavy load.
     *
     * @param on specifies true to enable the servo controller, false otherwise.
     */
    public void setControllerOn(boolean on)
    {
        if (on)
        {
            controller.pwmEnable();
        }
        else
        {
            controller.pwmDisable();
        }
    }   //setControllerOn

    //
    // Implements TrcServo abstract methods.
    //

    /**
     * This method inverts the servo direction.
     *
     * @param inverted specifies true to invert the servo direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        servo.setDirection(inverted? Servo.Direction.REVERSE: Servo.Direction.FORWARD);
    }   //setInverted

    /**
     * This method checks if the servo direction is inverted.
     *
     * @return true if the servo direction is inverted, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        return servo.getDirection() == Servo.Direction.REVERSE;
    }   //isInverted

    /**
     * This method sets the logical position of the servo.
     *
     * @param position specifies the logical position of the servo in the range of [0.0, 1.0].
     */
    @Override
    public void setLogicalPosition(double position)
    {
        if (logicalPos == null || position != logicalPos)
        {
            if (servoSetPosElapsedTimer != null) servoSetPosElapsedTimer.recordStartTime();
            servo.setPosition(position);
            if (servoSetPosElapsedTimer != null) servoSetPosElapsedTimer.recordEndTime();
            logicalPos = position;
        }
    }   //setLogicalPosition

    /**
     * This method returns the logical position of the servo. In general, servo do not provide real time position
     * feedback. Therefore, it will return the position set by the last setLogicalPosition call.
     *
     * @return logical position of the servo in the range of [0.0, 1.0].
     */
    @Override
    public double getLogicalPosition()
    {
        return logicalPos != null? logicalPos: 0.0;
    }   //getLogicalPosition

}   //class FtcServo
