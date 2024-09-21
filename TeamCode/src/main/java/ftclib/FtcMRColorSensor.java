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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import TrcCommonLib.trclib.TrcSensor;
import TrcCommonLib.trclib.TrcTimer;

/**
 * This class implements the Modern Color sensor extending TrcAnalogInput. It provides implementation of the abstract
 * methods in TrcAnalogInput.
 */
public class FtcMRColorSensor extends TrcSensor<FtcMRColorSensor.DataType>
{
    public enum DataType
    {
        COLOR_NUMBER,
        RED,
        GREEN,
        BLUE,
        WHITE
    }   //enum DataType

    public ModernRoboticsI2cColorSensor sensor;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcMRColorSensor(HardwareMap hardwareMap, String instanceName)
    {
        super(instanceName, 1);
        sensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, instanceName);
    }   //FtcMRColorSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcMRColorSensor(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName);
    }   //FtcMRColorSensor

    //
    // Implements TrcAnalogInput abstract methods.
    //

    /**
     * This method returns the raw sensor data of the specified type.
     *
     * @param index specifies the data index.
     * @param dataType specifies the data type.
     * @return raw sensor data of the specified index and type.
     */
    @Override
    public SensorData<Double> getRawData(int index, DataType dataType)
    {
        SensorData<Double> data = null;

        switch (dataType)
        {
            case COLOR_NUMBER:
                data = new SensorData<>(TrcTimer.getCurrentTime(), (double)sensor.argb());
                break;

            case RED:
                data = new SensorData<>(TrcTimer.getCurrentTime(), (double)sensor.red());
                break;

            case GREEN:
                data = new SensorData<>(TrcTimer.getCurrentTime(), (double)sensor.green());
                break;

            case BLUE:
                data = new SensorData<>(TrcTimer.getCurrentTime(), (double)sensor.blue());
                break;

            case WHITE:
                data = new SensorData<>(TrcTimer.getCurrentTime(), (double)sensor.alpha());
                break;
        }

        return data;
    }   //getRawData

}   //class FtcMRColorSensor
