/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package victor_lcm_interface;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class cartesian_value_quantity implements lcm.lcm.LCMEncodable
{
    public double x;
    public double y;
    public double z;
    public double a;
    public double b;
    public double c;
 
    public cartesian_value_quantity()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xf31b4152afd20adaL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(victor_lcm_interface.cartesian_value_quantity.class))
            return 0L;
 
        classes.add(victor_lcm_interface.cartesian_value_quantity.class);
        long hash = LCM_FINGERPRINT_BASE
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
        outs.writeDouble(this.x); 
 
        outs.writeDouble(this.y); 
 
        outs.writeDouble(this.z); 
 
        outs.writeDouble(this.a); 
 
        outs.writeDouble(this.b); 
 
        outs.writeDouble(this.c); 
 
    }
 
    public cartesian_value_quantity(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public cartesian_value_quantity(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static victor_lcm_interface.cartesian_value_quantity _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        victor_lcm_interface.cartesian_value_quantity o = new victor_lcm_interface.cartesian_value_quantity();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.x = ins.readDouble();
 
        this.y = ins.readDouble();
 
        this.z = ins.readDouble();
 
        this.a = ins.readDouble();
 
        this.b = ins.readDouble();
 
        this.c = ins.readDouble();
 
    }
 
    public victor_lcm_interface.cartesian_value_quantity copy()
    {
        victor_lcm_interface.cartesian_value_quantity outobj = new victor_lcm_interface.cartesian_value_quantity();
        outobj.x = this.x;
 
        outobj.y = this.y;
 
        outobj.z = this.z;
 
        outobj.a = this.a;
 
        outobj.b = this.b;
 
        outobj.c = this.c;
 
        return outobj;
    }
 
}
