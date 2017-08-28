/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package victor_hardware_interface;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class motion_status implements lcm.lcm.LCMEncodable
{
    public victor_hardware_interface.joint_value_quantity measured_joint_position;
    public victor_hardware_interface.joint_value_quantity commanded_joint_position;
    public victor_hardware_interface.joint_value_quantity measured_joint_velocity;
    public victor_hardware_interface.joint_value_quantity measured_joint_torque;
    public victor_hardware_interface.joint_value_quantity estimated_external_torque;
    public victor_hardware_interface.cartesian_value_quantity estimated_external_wrench;
    public victor_hardware_interface.cartesian_value_quantity measured_cartesian_pose_abc;
    public victor_hardware_interface.cartesian_value_quantity commanded_cartesian_pose_abc;
    public victor_hardware_interface.cartesian_pose measured_cartesian_pose;
    public victor_hardware_interface.cartesian_pose commanded_cartesian_pose;
    public victor_hardware_interface.control_mode active_control_mode;
    public double timestamp;
 
    public motion_status()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x8d25644d75239c66L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(victor_hardware_interface.motion_status.class))
            return 0L;
 
        classes.add(victor_hardware_interface.motion_status.class);
        long hash = LCM_FINGERPRINT_BASE
             + victor_hardware_interface.joint_value_quantity._hashRecursive(classes)
             + victor_hardware_interface.joint_value_quantity._hashRecursive(classes)
             + victor_hardware_interface.joint_value_quantity._hashRecursive(classes)
             + victor_hardware_interface.joint_value_quantity._hashRecursive(classes)
             + victor_hardware_interface.joint_value_quantity._hashRecursive(classes)
             + victor_hardware_interface.cartesian_value_quantity._hashRecursive(classes)
             + victor_hardware_interface.cartesian_value_quantity._hashRecursive(classes)
             + victor_hardware_interface.cartesian_value_quantity._hashRecursive(classes)
             + victor_hardware_interface.cartesian_pose._hashRecursive(classes)
             + victor_hardware_interface.cartesian_pose._hashRecursive(classes)
             + victor_hardware_interface.control_mode._hashRecursive(classes)
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
        this.measured_joint_position._encodeRecursive(outs); 
 
        this.commanded_joint_position._encodeRecursive(outs); 
 
        this.measured_joint_velocity._encodeRecursive(outs); 
 
        this.measured_joint_torque._encodeRecursive(outs); 
 
        this.estimated_external_torque._encodeRecursive(outs); 
 
        this.estimated_external_wrench._encodeRecursive(outs); 
 
        this.measured_cartesian_pose_abc._encodeRecursive(outs); 
 
        this.commanded_cartesian_pose_abc._encodeRecursive(outs); 
 
        this.measured_cartesian_pose._encodeRecursive(outs); 
 
        this.commanded_cartesian_pose._encodeRecursive(outs); 
 
        this.active_control_mode._encodeRecursive(outs); 
 
        outs.writeDouble(this.timestamp); 
 
    }
 
    public motion_status(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public motion_status(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static victor_hardware_interface.motion_status _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        victor_hardware_interface.motion_status o = new victor_hardware_interface.motion_status();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.measured_joint_position = victor_hardware_interface.joint_value_quantity._decodeRecursiveFactory(ins);
 
        this.commanded_joint_position = victor_hardware_interface.joint_value_quantity._decodeRecursiveFactory(ins);
 
        this.measured_joint_velocity = victor_hardware_interface.joint_value_quantity._decodeRecursiveFactory(ins);
 
        this.measured_joint_torque = victor_hardware_interface.joint_value_quantity._decodeRecursiveFactory(ins);
 
        this.estimated_external_torque = victor_hardware_interface.joint_value_quantity._decodeRecursiveFactory(ins);
 
        this.estimated_external_wrench = victor_hardware_interface.cartesian_value_quantity._decodeRecursiveFactory(ins);
 
        this.measured_cartesian_pose_abc = victor_hardware_interface.cartesian_value_quantity._decodeRecursiveFactory(ins);
 
        this.commanded_cartesian_pose_abc = victor_hardware_interface.cartesian_value_quantity._decodeRecursiveFactory(ins);
 
        this.measured_cartesian_pose = victor_hardware_interface.cartesian_pose._decodeRecursiveFactory(ins);
 
        this.commanded_cartesian_pose = victor_hardware_interface.cartesian_pose._decodeRecursiveFactory(ins);
 
        this.active_control_mode = victor_hardware_interface.control_mode._decodeRecursiveFactory(ins);
 
        this.timestamp = ins.readDouble();
 
    }
 
    public victor_hardware_interface.motion_status copy()
    {
        victor_hardware_interface.motion_status outobj = new victor_hardware_interface.motion_status();
        outobj.measured_joint_position = this.measured_joint_position.copy();
 
        outobj.commanded_joint_position = this.commanded_joint_position.copy();
 
        outobj.measured_joint_velocity = this.measured_joint_velocity.copy();
 
        outobj.measured_joint_torque = this.measured_joint_torque.copy();
 
        outobj.estimated_external_torque = this.estimated_external_torque.copy();
 
        outobj.estimated_external_wrench = this.estimated_external_wrench.copy();
 
        outobj.measured_cartesian_pose_abc = this.measured_cartesian_pose_abc.copy();
 
        outobj.commanded_cartesian_pose_abc = this.commanded_cartesian_pose_abc.copy();
 
        outobj.measured_cartesian_pose = this.measured_cartesian_pose.copy();
 
        outobj.commanded_cartesian_pose = this.commanded_cartesian_pose.copy();
 
        outobj.active_control_mode = this.active_control_mode.copy();
 
        outobj.timestamp = this.timestamp;
 
        return outobj;
    }
 
}

