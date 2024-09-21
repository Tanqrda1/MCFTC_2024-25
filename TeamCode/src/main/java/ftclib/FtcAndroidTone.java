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

import android.media.AudioAttributes;
import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioTrack;

import TrcCommonLib.trclib.TrcTone;

/**
 * This class implements a platform dependent sound player that can play a tone with specified waveform, frequency,
 * duration and volume using the Android AudioTrack.
 */
public class FtcAndroidTone extends TrcTone implements AudioTrack.OnPlaybackPositionUpdateListener
{
    private static final Waveform DEF_WAVEFORM = Waveform.TRIANGLE_WAVE;
    private static final int DEF_SAMPLERATE = 16*1024;  //approx. 16kHz

    private final int sampleRate;
    private AudioTrack audioTrack = null;
    private boolean playing = false;
    private double attack = 0.0;        //in seconds
    private double decay = 0.0;         //in seconds
    private double sustain = 1.0;       //in proportion (0.0 to 1.0)
    private double release = 0.0;       //in seconds
    private boolean envelopeEnabled = false;

    /**
     * Constructor: Create and initialize an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param defWaveform specifies the default waveform type.
     * @param sampleRate specifies the sampling rate.
     */
    public FtcAndroidTone(String instanceName, Waveform defWaveform, int sampleRate)
    {
        super(instanceName, defWaveform);
        if (sampleRate <= 0)
        {
            throw new IllegalArgumentException("Invalid sample rate.");
        }

        this.sampleRate = sampleRate;
    }   //FtcAndroidTone

    /**
     * Constructor: Create and initialize an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param defWaveform specifies the default waveform type.
     */
    public FtcAndroidTone(String instanceName, Waveform defWaveform)
    {
        this(instanceName, defWaveform, DEF_SAMPLERATE);
    }   //FtcAndroidTone

    /**
     * Constructor: Create and initialize an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcAndroidTone(String instanceName)
    {
        this(instanceName, DEF_WAVEFORM, DEF_SAMPLERATE);
    }   //FtcAndroidTone

    /**
     * This method sets the sound envelope parameters ADSR (Attack, Decay, Sustain, Release). Attack, Decay and
     * Release are time parameters in seconds. Sustain is a level parameter ranged from 0.0 to 1.0 (i.e. percentage
     * of maximum volume).
     *
     * @param attack specifies attack time in seconds.
     * @param decay specifies decay time in seconds.
     * @param sustain specifies sustain level in the range of 0.0 to 1.0. It is multiplied with the maximum volume
     *                to get the sustain level volume.
     * @param release specifies the release time in seconds.
     */
    public synchronized void setSoundEnvelope(double attack, double decay, double sustain, double release)
    {
        this.attack = attack;
        this.decay = decay;
        this.sustain = sustain;
        this.release = release;
    }   //setSoundEnvelope

    /**
     * This method enables/disables the sound envelope.
     */
    public synchronized void setSoundEnvelopeEnabled(boolean enabled)
    {
        this.envelopeEnabled = enabled;
    }   //setSoundEnvelopeEnabled

    /**
     * This method plays the sound data in the specified buffer.
     *
     * @param buffer specifies the sound data buffer.
     */
    public synchronized void playSound(short[] buffer)
    {
        AudioAttributes.Builder attrBuilder = new AudioAttributes.Builder();
        attrBuilder.setLegacyStreamType(AudioManager.STREAM_MUSIC);

        AudioFormat.Builder formatBuilder = new AudioFormat.Builder();
        formatBuilder.setEncoding(AudioFormat.ENCODING_PCM_16BIT)
                .setSampleRate(sampleRate)
                .setChannelMask(AudioFormat.CHANNEL_OUT_MONO);

        audioTrack = new AudioTrack(
                attrBuilder.build(),
                formatBuilder.build(),
                buffer.length * 2,
                AudioTrack.MODE_STATIC,
                AudioManager.AUDIO_SESSION_ID_GENERATE);

        audioTrack.write(buffer, 0, buffer.length);
        audioTrack.setNotificationMarkerPosition(buffer.length);
        audioTrack.setPlaybackPositionUpdateListener(this);
        audioTrack.play();
        playing = true;
    }   //playSound

    //
    // Implements TrcTone abstract methods.
    //

    /**
     * This method plays a tone with the specified waveform, frequency, duration and volume.
     *
     * @param waveform specifies the waveform type.
     * @param frequency specifies the tone frequency in Hz.
     * @param duration specifies the duration in seconds.
     * @param volume specifies the volume in the range 0.0 to 1.0.
     */
    @Override
    public synchronized void playTone(Waveform waveform, double frequency, double duration, double volume)
    {
        if (volume < 0.0 || volume > 1.0)
        {
            throw new IllegalArgumentException("Volume must be in the range 0.0 to 1.0.");
        }

         if (frequency < 0.0)
        {
            throw new IllegalArgumentException("Frequency cannot be negative.");
        }

        if (duration < 0.0)
        {
            throw new IllegalArgumentException("Duration cannot be negative.");
        }

        short[] buffer = new short[(int)(sampleRate*duration)];

        switch (waveform)
        {
            case SINE_WAVE:
                genSineWave(buffer, sampleRate, frequency, volume);
                break;

            case SQUARE_WAVE:
                genSquareWave(buffer, sampleRate, frequency, volume);
                break;

            case TRIANGLE_WAVE:
                genTriangleWave(buffer, sampleRate, frequency, volume);
                break;
        }

        if (envelopeEnabled)
        {
            applySoundEnvelope(buffer, sampleRate, attack, decay, sustain, release);
        }

        playSound(buffer);
    }   //playTone

    /**
     * This method stops the playing of the sound in progress.
     */
    @Override
    public synchronized void stop()
    {
        if (playing)
        {
            audioTrack.pause();
            audioTrack.flush();
            playing = false;
        }
    }   //stop

    /**
     * This method checks if the sound is still playing.
     *
     * @return true if the sound is still playing, false otherwise.
     */
    @Override
    public synchronized boolean isPlaying()
    {
        return playing;
    }   //isPlaying

    //
    // Implements AudioTrack.OnPlaybackPositionUpdateListener interface.
    //

    /**
     * This method is called when the sample at the set marker has been played. This is used to indicate
     * the completion of the tone played.
     *
     * @param track specifies the AudioTrack object that was playing.
     */
    @Override
    public synchronized void onMarkerReached(AudioTrack track)
    {
        audioTrack.setNotificationMarkerPosition(0);
        playing = false;
    }   //onMarkerReached

    /**
     * This method is called when the period marker has been reached.
     *
     * @param track specifies the AudioTrack object that was playing.
     */
    @Override
    public void onPeriodicNotification(AudioTrack track)
    {
    }   //onPeriodicNotification

}   //class FtcAndroidTone
