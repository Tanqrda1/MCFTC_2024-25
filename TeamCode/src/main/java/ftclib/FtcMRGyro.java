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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import TrcCommonLib.trclib.TrcFilter;
import TrcCommonLib.trclib.TrcGyro;
import TrcCommonLib.trclib.TrcTimer;

/**
 * This class implements the Modern Robotics gyro extending TrcGyro. It provides implementation of the abstract
 * methods in TrcGyro. The Modern Robotics gyro supports 3 axes: x, y and z. It provides rotation rate data for
 * all 3 axes. However, it only provides heading data for the z-axis and the heading data is wrap-around.
 */
public class FtcMRGyro extends TrcGyro
{
    private final ModernRoboticsI2cGyro gyro;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filters to use for filtering sensor noise, one for each axis. Since we
     *                have 3 axes, the array should have 3 elements. If no filters are used, it can be set to null.
     */
    public FtcMRGyro(HardwareMap hardwareMap, String instanceName, TrcFilter[] filters)
    {
        super(instanceName, 3, GYRO_HAS_X_AXIS | GYRO_HAS_Y_AXIS | GYRO_HAS_Z_AXIS, filters);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.get(GyroSensor.class, instanceName);
    }   //FtcMRGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filters to use for filtering sensor noise, one for each axis. Since we
     *                have 3 axes, the array should have 3 elements. If no filters are used, it can be set to null.
     */
    public FtcMRGyro(String instanceName, TrcFilter[] filters)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, filters);
    }   //FtcMRGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcMRGyro(String instanceName)
    {
        this(instanceName, null);
    }   //FtcMRGyro

    /**
     * This method calibrates the sensor.
     */
    public synchronized void calibrate()
    {
        gyro.calibrate();
        while (gyro.isCalibrating())
        {
            TrcTimer.sleep(10);
        }
    }   //calibrate

    //
    // Overriding TrcGyro methods.
    //

    /**
     * This method overrides the TrcGyro class. It doesn't have an x-integrator.
     */
    @Override
    public void resetXIntegrator()
    {
        throw new UnsupportedOperationException("Modern Robotics Gyro does not have an x-integrator.");
    }   //resetXIntegrator

    /**
     * This method overrides the TrcGyro class. It doesn't have an y-integrator.
     */
    @Override
    public void resetYIntegrator()
    {
        throw new UnsupportedOperationException("Modern Robotics Gyro does not have an y-integrator.");
    }   //resetYIntegrator

    /**
     * This method overrides the TrcGyro class and calls its own.
     */
    @Override
    public synchronized void resetZIntegrator()
    {
        gyro.resetZAxisIntegrator();
    }   //resetZIntegrator

    //
    // Implements TrcGyro abstract methods.
    //

    /**
     * This method returns the raw data of the specified type for the x-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the x-axis.
     */
    @Override
    public synchronized SensorData<Double> getRawXData(DataType dataType)
    {
        SensorData<Double> data;
        //
        // MR gyro supports only rotation rate for the x-axis.
        //
        if (dataType == DataType.ROTATION_RATE)
        {
            data = new SensorData<>(TrcTimer.getCurrentTime(), (double)gyro.rawX());
        }
        else
        {
            throw new UnsupportedOperationException("Modern Robotics Gyro only supports rotation rate.");
        }

        return data;
    }   //getRawXData

    /**
     * This method returns the raw data of the specified type for the y-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the y-axis.
     */
    @Override
    public synchronized SensorData<Double> getRawYData(DataType dataType)
    {
        SensorData<Double> data;
        //
        // MR gyro supports only rotation rate for the y-axis.
        //
        if (dataType == DataType.ROTATION_RATE)
        {
            data = new SensorData<>(TrcTimer.getCurrentTime(), (double)gyro.rawY());
        }
        else
        {
            throw new UnsupportedOperationException("Modern Robotics Gyro only supports rotation rate.");
        }

        return data;
    }   //getRawYData

    /**
     * This method returns the raw data of the specified type for the z-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the z-axis.
     */
    @Override
    public synchronized SensorData<Double> getRawZData(DataType dataType)
    {
        double value = 0.0;

        if (dataType == DataType.ROTATION_RATE)
        {
            value = gyro.rawZ();
        }
        else if (dataType == DataType.HEADING)
        {
            value = -gyro.getIntegratedZValue();
        }

        return new SensorData<>(TrcTimer.getCurrentTime(), value);
    }   //getRawZData

}   //class FtcMRGyro
