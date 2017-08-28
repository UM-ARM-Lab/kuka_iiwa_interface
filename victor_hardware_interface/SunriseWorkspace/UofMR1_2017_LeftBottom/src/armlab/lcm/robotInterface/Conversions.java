package armlab.lcm.robotInterface;

import armlab.lcm.msgs.cartesian_pose;
import armlab.lcm.msgs.cartesian_value_quantity;
import armlab.lcm.msgs.joint_value_quantity;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixRotation;
import com.kuka.roboticsAPI.geometricModel.math.MatrixBuilder;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;

public class Conversions
{
	public static void JVQToJointPosition(final joint_value_quantity jvq, JointPosition joint_position)
	{
		joint_position.set(0, jvq.joint_1);
		joint_position.set(1, jvq.joint_2);
		joint_position.set(2, jvq.joint_3);
		joint_position.set(3, jvq.joint_4);
		joint_position.set(4, jvq.joint_5);
		joint_position.set(5, jvq.joint_6);
		joint_position.set(6, jvq.joint_7);
	}
	
	public static JointPosition JVQToJointPosition(final joint_value_quantity jvq)
	{
		return new JointPosition(jvq.joint_1, jvq.joint_2, jvq.joint_3, jvq.joint_4, jvq.joint_5, jvq.joint_6, jvq.joint_7);
	}
	
	public static void JVQInitializer(final double val, joint_value_quantity jvq)
	{
		jvq.joint_1 = val;
		jvq.joint_2 = val;
		jvq.joint_3 = val;
		jvq.joint_4 = val;
		jvq.joint_5 = val;
		jvq.joint_6 = val;
		jvq.joint_7 = val;
	}
	
	public static joint_value_quantity JVQInitializer(final double val)
	{
		joint_value_quantity jvq = new joint_value_quantity();
		JVQInitializer(val, jvq);		
		return jvq;
	}
	
	public static void JointPositionToJVQ(final JointPosition joint_position, joint_value_quantity jvq)
	{
		jvq.joint_1 = joint_position.get(0);
		jvq.joint_2 = joint_position.get(1);
		jvq.joint_3 = joint_position.get(2);
		jvq.joint_4 = joint_position.get(3);
		jvq.joint_5 = joint_position.get(4);
		jvq.joint_6 = joint_position.get(5);
		jvq.joint_7 = joint_position.get(6);
	}
	
	public static joint_value_quantity JointPositionToJVQ(final JointPosition joint_position)
	{
		joint_value_quantity jvq = new joint_value_quantity();
		JointPositionToJVQ(joint_position, jvq);
		return jvq;
	}
	
	public static double[] JVQToVector(final joint_value_quantity jvq)
	{
		double[] vec = new double[7];
		vec[0] = jvq.joint_1;
		vec[1] = jvq.joint_2;
		vec[2] = jvq.joint_3;
		vec[3] = jvq.joint_4;
		vec[4] = jvq.joint_5;
		vec[5] = jvq.joint_6;
		vec[6] = jvq.joint_7;
		return vec;
	}
	
	public static void VectorToJVQ(final double[] vec,joint_value_quantity jvq)
	{
		jvq.joint_1 = vec[0];
		jvq.joint_2 = vec[1];
		jvq.joint_3 = vec[2];
		jvq.joint_4 = vec[3];
		jvq.joint_5 = vec[4];
		jvq.joint_6 = vec[5];
		jvq.joint_7 = vec[6];
	}
	
	public static joint_value_quantity VectorToJVQ(final double[] vec)
	{
		joint_value_quantity jvq = new joint_value_quantity();
		VectorToJVQ(vec, jvq);
		return jvq;
	}
	
	public static void VectorToCVQ(final double[] vec, cartesian_value_quantity cvq)
	{
		cvq.x = vec[0];
		cvq.y = vec[1];
		cvq.z = vec[2];
		cvq.a = vec[3];
		cvq.b = vec[4];
		cvq.c = vec[5];
	}
	
	public static cartesian_value_quantity VectorToCVQ(final double[] vec)
	{
		cartesian_value_quantity cvq = new cartesian_value_quantity();
		VectorToCVQ(vec, cvq);
		return cvq;
	}
	
	public static void CVQInitializer(final double val, cartesian_value_quantity cvq)
	{
		cvq.x = val;
		cvq.y = val;
		cvq.z = val;
		cvq.a = val;
		cvq.b = val;
		cvq.c = val;
	}
	
	public static cartesian_value_quantity CVQInitializer(final double val)
	{
		cartesian_value_quantity cvq = new cartesian_value_quantity();
		CVQInitializer(val, cvq);
		return cvq;
	}

	public static void TransformationToCVQ(final Transformation transformation, cartesian_value_quantity cvq)
	{
		// Need to convert from millimeters to meters
		cvq.x = transformation.getX() / 1000.0;
		cvq.y = transformation.getY() / 1000.0;
		cvq.z = transformation.getZ() / 1000.0;
		cvq.a = transformation.getAlphaRad();
		cvq.b = transformation.getBetaRad();
		cvq.c = transformation.getGammaRad();
	}
	
	public static void TransformationToPose(final Transformation transformation, cartesian_pose pose)
	{
		// Need to convert from millimeters to meters
		pose.xt = transformation.getX() / 1000.0;
		pose.yt = transformation.getY() / 1000.0;
		pose.zt = transformation.getZ() / 1000.0;
		MatrixToPoseQuat(transformation.getRotationMatrix(), pose);
	}
	
	public static cartesian_value_quantity TransformationToCVQ(final Transformation transformation)
	{
		cartesian_value_quantity cvq = new cartesian_value_quantity();
		TransformationToCVQ(transformation, cvq);
		return cvq;
	}
	
	public static Transformation CVQToTransformation(final cartesian_value_quantity cvq)
	{
		return Transformation.ofRad(cvq.x*1000,cvq.y*1000,cvq.z*1000,cvq.a,cvq.b,cvq.c);
	}

	public static Frame CVQToFrame(final cartesian_value_quantity cvq)
	{	
		return new Frame(CVQToTransformation(cvq));
	}
	
	public static void CVQToFrame(final cartesian_value_quantity cvq, Frame frame)
	{	
		frame.setX(cvq.x*1000);
		frame.setY(cvq.y*1000);
		frame.setZ(cvq.z*1000);
		frame.setAlphaRad(cvq.a);
		frame.setBetaRad(cvq.b);
		frame.setGammaRad(cvq.c);
	}
	
	/**
	 * Generates a quaternion from a Matrix.<p>
	 * Mercilessly copied <url>from https://github.com/libgdx/libgdx/blob/master/gdx/src/com/badlogic/gdx/math/Quaternion.java</url>
	 * @param matrix : the starting matrix
	 * @param quaternion : the resulting quaternion
	 */
	public static void MatrixToPoseQuat(final Matrix rot, cartesian_pose pose)
	{
		double xx = rot.getElement00();
		double xy = rot.getElement01();
		double xz = rot.getElement02();
		double yx = rot.getElement10();
		double yy = rot.getElement11();
		double yz = rot.getElement12();
		double zx = rot.getElement20();
		double zy = rot.getElement21();
		double zz = rot.getElement22();

		double x,y,z,w; // return

		final double trace = xx + yy + zz;

		// we protect the division by s by ensuring that s>=1
		if (trace >= 0)
		{ // |w| >= .5
			float s = (float)Math.sqrt(trace + 1); // |s|>=1 ...
			w = 0.5f * s;
			s = 0.5f / s; // so this division isn't bad
			x = (zy - yz) * s;
			y = (xz - zx) * s;
			z = (yx - xy) * s;
		}
		else if ((xx > yy) && (xx > zz))
		{
			float s = (float)Math.sqrt(1.0 + xx - yy - zz); // |s|>=1
			x = s * 0.5f; // |x| >= .5
			s = 0.5f / s;
			y = (yx + xy) * s;
			z = (xz + zx) * s;
			w = (zy - yz) * s;
		}
		else if (yy > zz)
		{
			float s = (float)Math.sqrt(1.0 + yy - xx - zz); // |s|>=1
			y = s * 0.5f; // |y| >= .5
			s = 0.5f / s;
			x = (yx + xy) * s;
			z = (zy + yz) * s;
			w = (xz - zx) * s;
		}
		else
		{
			float s = (float)Math.sqrt(1.0 + zz - xx - yy); // |s|>=1
			z = s * 0.5f; // |z| >= .5
			s = 0.5f / s;
			x = (xz + zx) * s;
			y = (zy + yz) * s;
			w = (yx - xy) * s;
		}
		// Fill in the pose
		pose.wr = w;
		pose.xr = x;
		pose.yr = y;
		pose.zr = z;
	}
	
	public static MatrixRotation QuatToMatrix(double w, double x, double y, double z)
	{
		MatrixBuilder mb = new MatrixBuilder();
		// Assumes input quaternion is already normalized!
		// Main diagonal
		mb.setElement00(1.0 - (2.0 * y * y) - (2.0 * z * z));
		mb.setElement11(1.0 - (2.0 * x * x) - (2.0 * z * z));
		mb.setElement22(1.0 - (2.0 * x * x) - (2.0 * y * y));
		// Others
		mb.setElement10((2.0 * x * y) + (2.0 * z * w));
		mb.setElement01((2.0 * x * y) - (2.0 * z * w));
		//
		mb.setElement20((2.0 * x * z) - (2.0 * y * w));
		mb.setElement02((2.0 * x * z) + (2.0 * y * w));
		//
		mb.setElement21((2.0 * y * z) + (2.0 * x * w));
		mb.setElement12((2.0 * y * z) - (2.0 * x * w));
		// Return matrix
		return MatrixRotation.of(mb.toMatrix());
	}
	
	public static Frame PoseToFrame(final cartesian_pose pose)
	{
		Vector translation = Vector.of(pose.xt * 1000.0, pose.yt * 1000.0, pose.zt * 1000.0);
		MatrixRotation rotMatrix = QuatToMatrix(pose.wr, pose.xr, pose.yr, pose.zr);
		return new Frame(Transformation.of(translation, rotMatrix));	
	}
	
	public static cartesian_pose IdentityPose()
	{
		cartesian_pose pose = new cartesian_pose();
		pose.xt = 0.0;
		pose.yt = 0.0;
		pose.zt = 0.0;
		pose.wr = 1.0;
		pose.xr = 0.0;
		pose.yr = 0.0;
		pose.zr = 0.0;
		return pose;
	}
	
	public static void ForceTorqueToCVQ(final ForceSensorData force_torque, cartesian_value_quantity cvq)
	{
		final Vector force = force_torque.getForce();
		final Vector torque = force_torque.getTorque();
		
		cvq.x = force.getX();
		cvq.y = force.getY();
		cvq.z = force.getZ();
		cvq.a = torque.getZ();
		cvq.b = torque.getY();
		cvq.c = torque.getX();
	}

	public static cartesian_value_quantity ForceTorqueToCVQ(final ForceSensorData force_torque)
	{
		cartesian_value_quantity cvq = new cartesian_value_quantity();
		ForceTorqueToCVQ(force_torque, cvq);
		return cvq;
	}
}