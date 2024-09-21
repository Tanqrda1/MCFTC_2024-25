/*
 * Copyright (c) 2018 Titan Robotics Club (http://www.titanrobotics.com)
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

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPixyCam1;

/**
 * This class implements a platform dependent pixy camera 1 that is connected to an I2C bus.
 * It provides access to the last detected objects reported by the pixy camera asynchronously.
 */
public class FtcPixyCam1 extends TrcPixyCam1
{
    private static final String moduleName = FtcPixyCam1.class.getSimpleName();
    private static final int DEF_I2C_ADDRESS = 0x54;
    private static final boolean USE_BUFFERED_READ = false;
    private final FtcI2cDevice pixyCam;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param devAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcPixyCam1(HardwareMap hardwareMap, String instanceName, int devAddress, boolean addressIs7Bit)
    {
        super(instanceName, false);
        pixyCam = new FtcI2cDevice(hardwareMap, instanceName, devAddress, addressIs7Bit, true);
        pixyCam.deviceSynch.setDeviceInfo(HardwareDevice.Manufacturer.Other, "Pixy Camera v1");
        if (USE_BUFFERED_READ)
        {
            pixyCam.deviceSynch.setBufferedReadWindow(
                    1, I2cDeviceSynch.ReadWindow.READ_REGISTER_COUNT_MAX, I2cDeviceSynch.ReadMode.REPEAT,
                    14);
        }
    }   //FtcPixyCam1

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param devAddress specifies the I2C address of the device.
     * @param addressIs7Bit specifies true if the I2C address is a 7-bit address, false if it is 8-bit.
     */
    public FtcPixyCam1(String instanceName, int devAddress, boolean addressIs7Bit)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, devAddress, addressIs7Bit);
    }   //FtcPixyCam1

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcPixyCam1(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, DEF_I2C_ADDRESS, true);
    }   //FtcPixyCam1

    /**
     * This method checks if the pixy camera is enabled.
     *
     * @return true if pixy camera is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        return pixyCam.isEnabled() && pixyCam.deviceSynch.isEnabled();
    }   //isEnable

    /**
     * This method enables/disables the pixy camera.
     *
     * @param enabled specifies true to enable pixy camera, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        pixyCam.deviceSynch.setEnabled(enabled);
        pixyCam.setEnabled(enabled);
        if (enabled)
        {
            start();
        }
        else
        {
            end();
        }
    }   //setEnabled

    //
    // Implements TrcPixyCam1 abstract methods.
    //

    /**
     * This method issues an asynchronous read of the specified number of bytes from the device.
     *
     * @param requestId specifies the ID to identify the request. Can be null if none was provided.
     * @param length specifies the number of bytes to read.
     */
    @Override
    public void asyncReadData(RequestId requestId, int length)
    {
        TrcEvent callbackEvent = new TrcEvent(moduleName + ".callbackHandler");
        callbackEvent.setCallback(this::requestHandler, null);
        pixyCam.asyncRead(requestId, length, callbackEvent);
    }   //asyncReadData

    /**
     * This method writes the data buffer to the device asynchronously.
     *
     * @param requestId specifies the ID to identify the request. Can be null if none was provided.
     * @param data specifies the data buffer.
     */
    @Override
    public void asyncWriteData(RequestId requestId, byte[] data)
    {
        pixyCam.asyncWrite(requestId, data, data.length, null);
    }   //asyncWriteData

}   //class FtcPixyCam1
