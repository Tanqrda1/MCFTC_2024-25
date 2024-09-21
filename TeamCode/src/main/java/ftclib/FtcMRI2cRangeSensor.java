/*
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import TrcCommonLib.trclib.TrcSensor;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;

/**
 * This class implements the Modern Robotics Range Sensor extending FtcMRI2cDevice that implements the common features
 * of all Modern Robotics I2C devices.
 */
public class FtcMRI2cRangeSensor extends FtcMRI2cDevice implements TrcSensor.DataSource<FtcMRI2cRangeSensor.DataType>
{
    public enum DataType
    {
        ULTRASONIC_DISTANCE,
        OPTICAL_DISTANCE
    }

    public static final int DEF_I2CADDRESS          = 0x28;     //8-bit address.
    //
    // I2C registers.
    //
    private static final int REG_ULTRSONIC_DISTANCE = 0x04;
    private static final int REG_OPTICAL_DISTANCE   = 0x05;

    private static final int READ_START             = REG_ULTRSONIC_DISTANCE;
    private static final int READ_END               = REG_OPTICAL_DISTANCE;
    private static final int READ_LENGTH            = (READ_END - READ_START + 1);

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcMRI2cRangeSensor(HardwareMap hardwareMap, String instanceName, int i2cAddress, boolean addressIs7Bit)
    {
        super(hardwareMap, instanceName, i2cAddress, addressIs7Bit);
        deviceSynch.setDeviceInfo(HardwareDevice.Manufacturer.ModernRobotics, "MR Range Sensor");
        deviceSynch.setBufferedReadWindow(READ_START, READ_LENGTH, I2cDeviceSynch.ReadMode.REPEAT, READ_LENGTH);
    }   //FtcMRI2cRangeSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param i2cAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcMRI2cRangeSensor(String instanceName, int i2cAddress, boolean addressIs7Bit)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, i2cAddress, addressIs7Bit);
    }   //FtcMRI2cRangeSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcMRI2cRangeSensor(String instanceName)
    {
        this(instanceName, DEF_I2CADDRESS, false);
    }   //FtcMRI2cRangeSensor

    /**
     * This method returns the ultrasonic distance.
     *
     * @return ultrasonic distance.
     */
    public TrcSensor.SensorData<Double> getUltrasonicDistance()
    {
        byte[] regData = readData(READ_START, READ_LENGTH);
        return new TrcSensor.SensorData<>(
            TrcTimer.getCurrentTime(), (double)TrcUtil.bytesToInt(regData[REG_ULTRSONIC_DISTANCE - READ_START]));
    }   //getUltrasonicDistance

    /**
     * This method returns the optical distance.
     *
     * @return optical distance.
     */
    public TrcSensor.SensorData<Double> getOpticalDistance()
    {
        byte[] regData = readData(READ_START, READ_LENGTH);
        return new TrcSensor.SensorData<>(
            TrcTimer.getCurrentTime(), (double)TrcUtil.bytesToInt(regData[REG_OPTICAL_DISTANCE - READ_START]));
    }   //getOpticalDistance

    //
    // Implements TrcSensor.DataSource interface.
    //

    /**
     * This method returns the sensor data of the specified index.
     *
     * @param index specifies the data index.
     * @param dataType specifies the data type.
     * @return sensor data of the specified index and type.
     */
    @Override
    public TrcSensor.SensorData<Double> getRawData(int index, DataType dataType)
    {
        TrcSensor.SensorData<Double> data = null;

        switch (dataType)
        {
            case ULTRASONIC_DISTANCE:
                data = getUltrasonicDistance();
                break;

            case OPTICAL_DISTANCE:
                data = getOpticalDistance();
                break;
        }

        return data;
    }   //getRawData

}   //class FtcMRI2cRangeSensor
