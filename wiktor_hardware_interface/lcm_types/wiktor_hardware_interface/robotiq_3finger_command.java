/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package wiktor_hardware_interface;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class robotiq_3finger_command implements lcm.lcm.LCMEncodable
{
    public long utime;
    public wiktor_hardware_interface.robotiq_3finger_actuator_command finger_a_command;
    public wiktor_hardware_interface.robotiq_3finger_actuator_command finger_b_command;
    public wiktor_hardware_interface.robotiq_3finger_actuator_command finger_c_command;
    public wiktor_hardware_interface.robotiq_3finger_actuator_command scissor_command;
 
    public robotiq_3finger_command()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x91f5365e8adb6eb4L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(wiktor_hardware_interface.robotiq_3finger_command.class))
            return 0L;
 
        classes.add(wiktor_hardware_interface.robotiq_3finger_command.class);
        long hash = LCM_FINGERPRINT_BASE
             + wiktor_hardware_interface.robotiq_3finger_actuator_command._hashRecursive(classes)
             + wiktor_hardware_interface.robotiq_3finger_actuator_command._hashRecursive(classes)
             + wiktor_hardware_interface.robotiq_3finger_actuator_command._hashRecursive(classes)
             + wiktor_hardware_interface.robotiq_3finger_actuator_command._hashRecursive(classes)
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeLong(this.utime); 
 
        this.finger_a_command._encodeRecursive(outs); 
 
        this.finger_b_command._encodeRecursive(outs); 
 
        this.finger_c_command._encodeRecursive(outs); 
 
        this.scissor_command._encodeRecursive(outs); 
 
    }
 
    public robotiq_3finger_command(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public robotiq_3finger_command(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static wiktor_hardware_interface.robotiq_3finger_command _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        wiktor_hardware_interface.robotiq_3finger_command o = new wiktor_hardware_interface.robotiq_3finger_command();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.utime = ins.readLong();
 
        this.finger_a_command = wiktor_hardware_interface.robotiq_3finger_actuator_command._decodeRecursiveFactory(ins);
 
        this.finger_b_command = wiktor_hardware_interface.robotiq_3finger_actuator_command._decodeRecursiveFactory(ins);
 
        this.finger_c_command = wiktor_hardware_interface.robotiq_3finger_actuator_command._decodeRecursiveFactory(ins);
 
        this.scissor_command = wiktor_hardware_interface.robotiq_3finger_actuator_command._decodeRecursiveFactory(ins);
 
    }
 
    public wiktor_hardware_interface.robotiq_3finger_command copy()
    {
        wiktor_hardware_interface.robotiq_3finger_command outobj = new wiktor_hardware_interface.robotiq_3finger_command();
        outobj.utime = this.utime;
 
        outobj.finger_a_command = this.finger_a_command.copy();
 
        outobj.finger_b_command = this.finger_b_command.copy();
 
        outobj.finger_c_command = this.finger_c_command.copy();
 
        outobj.scissor_command = this.scissor_command.copy();
 
        return outobj;
    }
 
}
