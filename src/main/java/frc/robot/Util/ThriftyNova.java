//ThriftyNova API v1.0.0

package frc.robot.Util;

import edu.wpi.first.hal.CANAPIJNI;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;

public class ThriftyNova
{
    public final int THRIFTY_ID = 13;
    public final int MOTOR_CONTROLLER_ID = 2;

    public final int STATUS_API_CLASS = 20;
    public final int CONTORL_API_CLASS = 10;

    public final int PERCENT_OUTPUT = 1;
    public final int POSITION_OUTPUT = 2;
    public final int VELOCITY_OUTPUT = 3;

    int device_id = -1;
    int device_handle = -1;

    ///DEVICE SET UP
    public ThriftyNova(int dev_id)
    {
        device_id = dev_id;
        device_handle = CANAPIJNI.initializeCAN          
        (
            THRIFTY_ID, 
            device_id, 
            MOTOR_CONTROLLER_ID
        );
    }
    
    ///SETTERS
    /** 
     * The speed to set the motor. Value should be between -1.0 and 1.0.
     */
    public void setPercentOutput(double percent_output)
    {   
        setMotor(0, PERCENT_OUTPUT, translateOutput(limitRange(-1, 1, percent_output)));
    }

    /** 
     * Drives the motor towards the given position using the configured P, I, D and F values.
     */
    public void setPosition(int target_position)
    {   
        setPosition(target_position, 0);
    }
    public void setPosition(int target_position, int pid_select)
    {   
        setMotor(pid_select, POSITION_OUTPUT, target_position);
    }

    /** 
     * Drives the motor at the given velocity using the configured P, I, D and F values.
     */
    public void setVelocity(int target_velocity)
    {   
        setVelocity(target_velocity, 0);
    }
    public void setVelocity(int target_velocity, int pid_select)
    {   
        setMotor(pid_select, VELOCITY_OUTPUT, target_velocity);
    }

    ///CONFIGS
    /**
     * If enabled, will drive the motor in reverse when commanded to go forward
     */
    public boolean setInverted(boolean inverted) {return setConfig(0, inverted ? 1 : 0);}
    public boolean setBrakeMode(boolean brake_mode) {return setConfig(1, brake_mode ? 1 : 0);}
    /**
     * Sets the max forward speed of the motor which the motor will not excede, even if instructed to.
     * Input ranges from 0 to 1. Will return true on successful upload.
     */
    public boolean setMaxForward(double max_forward) 
    {
        return setConfig(3, translateOutput(limitRange(0, 1, max_forward)));
    }
    /**
     * Sets the max reverse speed of the motor which the motor will not excede, even if instructed to.
     * Input ranges from 0 to -1. Will return true on successful upload.
     */
    public boolean setMaxReverse(double max_reverse) 
    {
        return setConfig(4, translateOutput(-limitRange(-1, 0, max_reverse)));
    }
    /**
     * Sets the ramp rate in seconds. For example, an input of .5 will ramp the motor from idle to 100%
     * over the course of .5 seconds. Input ranges from 0 to 10. Will return true on successful upload.
     */
    public boolean setRampUp(double ramp_up) 
    {
        ramp_up = limitRange(0, 10, ramp_up);

        double translated_ramp = 1;

        if(ramp_up != 0) translated_ramp = 1 / (ramp_up * 1000);    

        return setConfig(5, translateOutput(translated_ramp));
    }
    /**
     * Sets the ramp rate in seconds. For example, an input of .5 will ramp the motor from 100% to idle
     * over the course of .5 seconds. Input ranges from 0 to 10. Will return true on successful upload.
     */
    public boolean setRampDown(double ramp_down) 
    {
        ramp_down = limitRange(0, 10, ramp_down);

        double translated_ramp = 1;

        if(ramp_down != 0) translated_ramp = 1 / (ramp_down * 1000);   

        return setConfig(6, translateOutput(translated_ramp));
    }
    public boolean setMaxCurrent(double max_current) {return setConfig(7, translateOutput(max_current));}
    public boolean setExtSensorId(int extern_id) {return setConfig(9, extern_id);}

    public boolean setkP(double p) {return setkP(p, 0);}
    public boolean setkP(double p, int profile) 
    {
        return setConfig(profile == 1 ? 16 : 10, translatePIDOutput(p));
    }
    public boolean setkI(double i) {return setkI(i, 0);}
    public boolean setkI(double i, int profile) 
    {
        return setConfig(profile == 1 ? 17 : 11, translatePIDOutput(i));
    }
    public boolean setkD(double d) {return setkD(d, 0);}
    public boolean setkD(double d, int profile) 
    {
        return setConfig(profile == 1 ? 18 : 12, translatePIDOutput(d));
    }
    public boolean setkF(double f) {return setkF(f, 0);}
    public boolean setkF(double f, int profile) 
    {
        return setConfig(profile == 1 ? 19 : 13, translatePIDOutput(f));
    }

    //Status Frame control
    public boolean setSFPeriodFaults(double period) {return setConfig(18, translateOutput(period));}
    public boolean setSFPeriodSensor(double period) {return setConfig(19, translateOutput(period));}
    public boolean setSFPeriodExtSensor(double period) {return setConfig(20, translateOutput(period));}
    public boolean setSFPeriodControl(double period) {return setConfig(21, translateOutput(period));}
    public boolean setSFPeriodCurrent(double period) {return setConfig(22, translateOutput(period));}


    ///GETTERS
    public int getPosition() {return getMotorStatus(1, 4, 7);}
    public int getVelocity() {return getMotorStatus(1, 0, 3);}
    public int getExtPosition() {return getMotorStatus(2, 4, 7);}
    public int getExtVelocity() {return getMotorStatus(2, 0, 3);}
    public int getCurrentDraw() {return getMotorStatus(4, 6, 7);}
    // public int getAppliedOutput() {return getMotorStatus(4, 0, 3);}

    ///UTIL
    private void setMotor(int pid_select, int control_type, int target)
    {
        CANAPIJNI.writeCANPacketNoThrow(
            device_handle, 
            new byte[] {
                0, 
                0, 
                (byte)pid_select, 
                (byte)control_type, 
                (byte)((target >> 24) & 0xFF), 
                (byte)((target >> 16) & 0xFF), 
                (byte)((target >> 8) & 0xFF), 
                (byte)(target & 0xFF)
            },
            getApiId(CONTORL_API_CLASS, 1)
        );
    }

    private boolean setConfig(int index, int value)
    {     
        if(CANAPIJNI.writeCANPacketNoThrow(
            device_handle, 
            new byte[] {
                (byte)((value >> 24) & 0xFF), 
                (byte)((value >> 16) & 0xFF), 
                (byte)((value >> 8) & 0xFF), 
                (byte)(value & 0xFF),
                (byte)((index >> 24) & 0xFF), 
                (byte)((index >> 16) & 0xFF), 
                (byte)((index >> 8) & 0xFF), 
                (byte)(index & 0xFF)
            },
            getApiId(CONTORL_API_CLASS, 0)
        ) != 0) return false;

        CANData data_in = new CANData();
        long rio_time = RobotController.getFPGATime();

        ///Checks if robot disabled because 1. blocking isn't as much of an issue and 2. rio start up times
        while((RobotController.getFPGATime() - rio_time) < (RobotState.isDisabled() ? 50000 : 5000))
        {
            CANAPIJNI.readCANPacketNew(
                device_handle, 
                getApiId(STATUS_API_CLASS, 5), 
                data_in
            );

            if(data_in.data[0] == 0x69) return true;
        }

        System.out.println("\nThriftyWarning: Failed to set configuration " + index + ".\n");

        return false;
    }

    public void setEncoder(int value)
    {
        CANAPIJNI.writeCANPacketNoThrow(
            device_handle, 
            new byte[] {
                (byte)((value >> 24) & 0xFF), 
                (byte)((value >> 16) & 0xFF), 
                (byte)((value >> 8) & 0xFF), 
                (byte)(value & 0xFF),
                (byte)((value >> 24) & 0xFF), 
                (byte)((value >> 16) & 0xFF), 
                (byte)((value >> 8) & 0xFF), 
                (byte)(value & 0xFF),
            },
            getApiId(CONTORL_API_CLASS, 2)
        );
    }

    private int getMotorStatus(int index, int start_byte, int end_byte)
    {
        CANData data_in = new CANData();
        CANAPIJNI.readCANPacketLatest(
            device_handle, 
            getApiId(STATUS_API_CLASS, index), 
            data_in
        );

        int motor_data = 0;

        for(int i = start_byte; i <= end_byte; i++)
        {
            motor_data += (data_in.data[i] & 0xFF) << ((end_byte - i) * 8);
        }

        return motor_data;
    }

    private int getApiId(int api_class, int api_index)
    {
        return ((api_class << 4) + api_index);
    }

    private int translateOutput(double input)
    {
        return (int)(10000 * input);
    }

    private int translatePIDOutput(double input)
    {
        return (int)(1000000 * input);
    }

    private double limitRange(double min, double max, double input)
    {
        if(input < min) return min;
        if(input > max) return max;
        return input;
    }
}
