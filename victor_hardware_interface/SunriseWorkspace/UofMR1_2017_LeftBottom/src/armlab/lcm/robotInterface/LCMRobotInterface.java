package armlab.lcm.robotInterface;

import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import armlab.lcm.msgs.cartesian_control_mode_limits;
import armlab.lcm.msgs.cartesian_impedance_parameters;
import armlab.lcm.msgs.cartesian_path_execution_parameters;
import armlab.lcm.msgs.control_mode;
import armlab.lcm.msgs.control_mode_parameters;
import armlab.lcm.msgs.joint_impedance_parameters;
import armlab.lcm.msgs.joint_path_execution_parameters;
import armlab.lcm.msgs.motion_command;
import armlab.lcm.msgs.motion_status;
import armlab.lcm.msgs.robotiq_3finger_actuator_status;
import armlab.lcm.msgs.robotiq_3finger_command;
import armlab.lcm.msgs.robotiq_3finger_object_status;
import armlab.lcm.msgs.robotiq_3finger_status;
import armlab.robotiq.gripper.Robotiq3FingerGripper;
import armlab.robotiq.gripper.Robotiq3FingerGripperCommand;
import armlab.robotiq.gripper.Robotiq3FingerGripperStatus;
import armlab.utils.Utils;

import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.persistenceModel.PersistenceException;

public class LCMRobotInterface extends RoboticsAPIApplication implements LCMSubscriber
{
    static final public String CONTROL_MODE_COMMAND_CHANNEL = "control_mode_command";
    static final public String CONTROL_MODE_STATUS_CHANNEL = "control_mode_status";
    static final public String MOTION_COMMAND_CHANNEL = "motion_command";
    static final public String MOTION_STATUS_CHANNEL = "motion_status";
    static final public String GRIPPER_COMMAND_CHANNEL = "gripper_command";
    static final public String GRIPPER_STATUS_CHANNEL = "gripper_status";
    
    static final public int CONTROL_MODE_FEEDBACK_PERIOD_MS = 1000;
    static final public int MOTION_STATUS_FEEDBACK_PERIOD_MS = 10; 
    static final public int GRIPPER_FEEDBACK_PERIOD_MS = 100;
    static final public int MAIN_LOOP_CONTROL_PERIOD_MS = 10; 
    
    @Inject
    private LBR iiwa7_arm_;
    private Robotiq3FingerGripper gripper_;
    private LCM lcm_subscriber_;
    private LCM lcm_publisher_;
    
    private Timer main_loop_timer_;
    private TimerTask main_loop_task_;
    
    private Object arm_io_lock_;
    private ControlModePublisher control_mode_publisher_;
    private MotionStatusPublisher motion_status_publisher_;
    private Robotiq3FingerGripperPublisher gripper_publisher_;
    
    private ControlModeCommandHandler control_mode_command_handler_;
    private MotionCommandHandler motion_command_handler_;
    private GripperCommandHandler gripper_command_handler_;
    
    
    private Tool tool_;
    private ObjectFrame end_effector_frame_;
    
    private boolean running_;
    private SmartServo joint_smartservo_motion_;
    private SmartServoLIN cartesian_smartservo_motion_;
    public enum ControlMode { JOINT_POSITION, CARTESIAN_POSE, JOINT_IMPEDANCE, CARTESIAN_IMPEDANCE };
    private ControlMode active_control_mode_ = ControlMode.JOINT_POSITION;
    
    private joint_path_execution_parameters joint_path_execution_params_;
    private cartesian_path_execution_parameters cartesian_path_execution_params_;

    
    @Override
    public void initialize()
    {
        // Double check some assertions regarding message definitions and internal representations
        if ((byte)ControlMode.JOINT_POSITION.ordinal() != control_mode.JOINT_POSITION ||
            (byte)ControlMode.JOINT_IMPEDANCE.ordinal() != control_mode.JOINT_IMPEDANCE ||
            (byte)ControlMode.CARTESIAN_POSE.ordinal() != control_mode.CARTESIAN_POSE ||
            (byte)ControlMode.CARTESIAN_IMPEDANCE.ordinal() != control_mode.CARTESIAN_IMPEDANCE)
        {
            throw new NumberFormatException("Control mode enums do not match.");
        }
        
        try
        {
            tool_ = (Tool)getApplicationData().createFromTemplate("Robotiq3FingerGripper");
            tool_.attachTo(iiwa7_arm_.getFlange());
            end_effector_frame_ = tool_.getFrame("PalmSurface");
        }
        catch (PersistenceException ex)
        {
            getLogger().info("Robotiq3FingerGripper tool not available. Using flange.");
            end_effector_frame_ = iiwa7_arm_.getFlange();
        }
        
        if (end_effector_frame_ == null)
        {
            throw new IllegalArgumentException("Something is wrong with tools or frame names");
        }
        
        joint_path_execution_params_ = new joint_path_execution_parameters();
        joint_path_execution_params_.joint_relative_acceleration = 0.1;
        joint_path_execution_params_.joint_relative_velocity = 0.1;
        joint_path_execution_params_.override_joint_acceleration = 0.0;
        cartesian_path_execution_params_ = new cartesian_path_execution_parameters();
        cartesian_path_execution_params_.max_nullspace_velocity = 0.1;
        cartesian_path_execution_params_.max_velocity = Conversions.cvqInitializer(0.1);
        cartesian_path_execution_params_.max_nullspace_acceleration = 0.1;
        cartesian_path_execution_params_.max_acceleration = Conversions.cvqInitializer(0.1);
        
        joint_smartservo_motion_ = createSmartServoMotion(joint_path_execution_params_);
        cartesian_smartservo_motion_ = createSmartServoLINMotion(cartesian_path_execution_params_);
        
        arm_io_lock_ = new Object();
        
        if (tool_ != null)
        {
            getLogger().info("Initializing Gripper");
            gripper_ = new Robotiq3FingerGripper(iiwa7_arm_, getLogger());
            gripper_.InitializeGripper();
        }
        else
        {
            getLogger().info("No Gripper to initialize");
        }
        
        try
        {
            getLogger().info("Initializing LCM Publisher");
            lcm_publisher_ = new LCM(LCMURLs.DEFAULT_DEST_URL);
            control_mode_publisher_ = new ControlModePublisher();
            gripper_publisher_ = new Robotiq3FingerGripperPublisher();
            motion_status_publisher_ = new MotionStatusPublisher();
            
            getLogger().info("Initializing LCM Subscriptions");
            control_mode_command_handler_ = new ControlModeCommandHandler();
            motion_command_handler_ = new MotionCommandHandler();
            gripper_command_handler_ = new GripperCommandHandler();
            lcm_subscriber_ = new LCM(LCMURLs.DEFAULT_SELF_URL);
            lcm_subscriber_.subscribeAll(this);
        }
        catch (IOException ex)
        {
            getLogger().info("LCM Subscription Exception: " + ex);
        }
        
        

        getLogger().info("Creating control loop objects");
        main_loop_timer_ = new Timer ();
        main_loop_task_ = new TimerTask()
        {
            @Override
            public void run ()
            {
                control_mode_parameters control_mode_cmd;
                JointPosition joint_position_target;
                Frame cartesian_pose_target;
                JointPosition joint_impedance_target;
                Frame cartesian_impedance_target;
                synchronized (arm_io_lock_)
                {
                    control_mode_cmd = control_mode_command_handler_.getControlModeCommand();
                    joint_position_target = motion_command_handler_.getJointPositionTarget();
                    cartesian_pose_target = motion_command_handler_.getCartesianPoseTarget();
                    joint_impedance_target = motion_command_handler_.getJointImpedanceTarget();
                    cartesian_impedance_target = motion_command_handler_.getCartesianImpedanceTarget();
                }
                
                if (control_mode_cmd != null)
                {
                    applyControlMode(control_mode_cmd);
                }

                switch (active_control_mode_)
                {
                    case JOINT_POSITION:
                    {
                        if (joint_position_target != null)
                        {
                            // Without isReadyToMove() this code seems to block on setDestination
                            if (iiwa7_arm_.isReadyToMove())// && !old_target.equals(target))
                            {
                                joint_smartservo_motion_.getRuntime().setDestination(joint_position_target);
                            }
                        }
                        break;
                    }
                    case JOINT_IMPEDANCE:
                    {
                        if (joint_impedance_target != null)
                        {
                            // Without isReadyToMove() this code seems to block on setDestination
                            if (iiwa7_arm_.isReadyToMove())// && !old_target.equals(target))
                            {
                                joint_smartservo_motion_.getRuntime().setDestination(joint_impedance_target);
                            }
                        }
                        break;
                    }
                    case CARTESIAN_POSE:
                    {
                        // add Cartesian pose stuff here
                        if (cartesian_pose_target != null)
                        {
                            // Without isReadyToMove() this code seems to block on setDestination
                            if (iiwa7_arm_.isReadyToMove())// && !old_target.equals(target))
                            {
                                try
                                {
                                    cartesian_smartservo_motion_.getRuntime().setDestination(cartesian_pose_target);
                                }
                                catch(Exception e)
                                {
                                    getLogger().error(e.getClass().getName() + ": " + e.getMessage());
                                }
                                
                            }
                        }
                        break;
                    }
                    case CARTESIAN_IMPEDANCE:
                    {
                        // add Cartesian pose stuff here
                        if (cartesian_impedance_target != null)
                        {
                            // Without isReadyToMove() this code seems to block on setDestination
                            if (iiwa7_arm_.isReadyToMove())// && !old_target.equals(target))
                            {
                                try
                                {
                                    cartesian_smartservo_motion_.getRuntime().setDestination(cartesian_impedance_target);
                                }
                                catch(Exception e)
                                {
                                    getLogger().error(e.getClass().getName() + ": " + e.getMessage());
                                }
                                
                            }
                        }
                        break;
                    }
                    default:
                    {
                        getLogger().error("Control mode: " + active_control_mode_ + " not implemented");
                        break;
                    }
                }
            }
        };
    }
    
    @Override
    public void run()
    {
        running_ = true;

        
        
        getLogger().info("Starting SmartServo motion");
        active_control_mode_ = ControlMode.JOINT_POSITION;
        end_effector_frame_.moveAsync(joint_smartservo_motion_);
        
        getLogger().info("Starting feedback threads");
        control_mode_publisher_.start();
        motion_status_publisher_.start();
        gripper_publisher_.start();


        getLogger().info("Starting control loop");
        main_loop_timer_.schedule(main_loop_task_, 0, MAIN_LOOP_CONTROL_PERIOD_MS);

        // Spin; doing nothing until we get interrupted for some reason
        try
        {
            while (running_)
            {
                TimeUnit.SECONDS.sleep(10);
            }
        }
        catch (InterruptedException ex)
        {
            getLogger().info("Main loop interrupted: " + ex);
        }
    }
    
    @Override
    public void dispose()
    {
        getLogger().info("Disposing...");
        running_ = false;
        main_loop_timer_.cancel();
        gripper_publisher_.cancel();
        motion_status_publisher_.cancel();
        control_mode_publisher_.cancel();
        
        lcm_publisher_.close();
        lcm_subscriber_.close();
        super.dispose();
    }
    
    @Override 
    public void onApplicationStateChanged(RoboticsAPIApplicationState state)
    {
        if (state == RoboticsAPIApplicationState.STOPPING)
        {
            running_ = false;
        }
        super.onApplicationStateChanged(state);
    };
    
    
    private SmartServo createSmartServoMotion(final joint_path_execution_parameters params)
    {
        SmartServo motion = new SmartServo(iiwa7_arm_.getCurrentJointPosition());
        motion.setMinimumTrajectoryExecutionTime(20e-3);    // TODO : Parameterize
        motion.setJointVelocityRel(params.joint_relative_velocity);                    
        motion.setJointAccelerationRel(params.joint_relative_acceleration);
        motion.overrideJointAcceleration(params.override_joint_acceleration);
        motion.setTimeoutAfterGoalReach(3600);                // TODO : Parameterize
        return motion;
    }
    
    private SmartServoLIN createSmartServoLINMotion(final cartesian_path_execution_parameters params)
    {
        SmartServoLIN motion = new SmartServoLIN(iiwa7_arm_.getCurrentCartesianPosition(end_effector_frame_));
        motion.setMinimumTrajectoryExecutionTime(20e-3);    // TODO : Parameterize
        motion.setMaxNullSpaceAcceleration(params.max_nullspace_acceleration);
        motion.setMaxNullSpaceVelocity(params.max_nullspace_velocity);
        motion.setMaxOrientationAcceleration(new double[]{params.max_acceleration.a, params.max_acceleration.b, params.max_acceleration.c});
        motion.setMaxOrientationVelocity(new double[]{params.max_velocity.a, params.max_velocity.b, params.max_velocity.c});
        motion.setMaxTranslationAcceleration(new double[]{params.max_acceleration.x, params.max_acceleration.y, params.max_acceleration.z});
        motion.setMaxTranslationVelocity(new double[]{params.max_velocity.x, params.max_velocity.y, params.max_velocity.z});
        motion.setTimeoutAfterGoalReach(3600);                // TODO : Parameterize
        return motion;
    }
    
    @Override
	public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try 
        {
            if (channel.equals(CONTROL_MODE_COMMAND_CHANNEL))
            {
                control_mode_parameters cmd = new control_mode_parameters(ins);
                control_mode_command_handler_.storeControlModeCommand(cmd);
            }
            else if (channel.equals(MOTION_COMMAND_CHANNEL))
            {
                motion_command cmd = new motion_command(ins);
                motion_command_handler_.storeMotionCommand(cmd);
            }            
            else if (channel.equals(GRIPPER_COMMAND_CHANNEL))
            {
                robotiq_3finger_command cmd = new robotiq_3finger_command(ins);
                gripper_command_handler_.executeCommand(cmd);
            }
            else
            {
                getLogger().warn("Unknown LCM channel: " + channel);
            }
        }
        catch (IOException ex)
        {
            getLogger().info("Message received exception: " + ex);
        }
    }
    
    public IMotionControlMode buildMotionControlMode(control_mode_parameters cmd) {
        IMotionControlMode cm = null;
        
        getLogger().info("In buildMotionControlMode");

        switch (cmd.control_mode.mode)
        {
            case control_mode.JOINT_POSITION:
            {
                PositionControlMode pcm = new PositionControlMode(true);
                cm = pcm;
                break;
            }    
            case control_mode.CARTESIAN_POSE:
            {
                PositionControlMode pcm = new PositionControlMode(true);
                cm = pcm;
                break;
            }
            case control_mode.JOINT_IMPEDANCE:
            {
                JointImpedanceControlMode jcm = new JointImpedanceControlMode(iiwa7_arm_.getJointCount());
                jcm.setDamping(Conversions.jvqToVector(cmd.joint_impedance_params.joint_damping));
                jcm.setStiffness(Conversions.jvqToVector(cmd.joint_impedance_params.joint_stiffness));
                cm = jcm;
                break;
                
            }
            case control_mode.CARTESIAN_IMPEDANCE:
            {
                CartesianImpedanceControlMode ccm = new CartesianImpedanceControlMode();
                ccm.parametrize(CartDOF.X).setDamping(cmd.cartesian_impedance_params.cartesian_damping.x);
                ccm.parametrize(CartDOF.Y).setDamping(cmd.cartesian_impedance_params.cartesian_damping.y);
                ccm.parametrize(CartDOF.Z).setDamping(cmd.cartesian_impedance_params.cartesian_damping.z);
                ccm.parametrize(CartDOF.A).setDamping(cmd.cartesian_impedance_params.cartesian_damping.a);
                ccm.parametrize(CartDOF.B).setDamping(cmd.cartesian_impedance_params.cartesian_damping.b);
                ccm.parametrize(CartDOF.C).setDamping(cmd.cartesian_impedance_params.cartesian_damping.c);
                ccm.parametrize(CartDOF.X).setStiffness(cmd.cartesian_impedance_params.cartesian_stiffness.x);
                ccm.parametrize(CartDOF.Y).setStiffness(cmd.cartesian_impedance_params.cartesian_stiffness.y);
                ccm.parametrize(CartDOF.Z).setStiffness(cmd.cartesian_impedance_params.cartesian_stiffness.z);
                ccm.parametrize(CartDOF.A).setStiffness(cmd.cartesian_impedance_params.cartesian_stiffness.a);
                ccm.parametrize(CartDOF.B).setStiffness(cmd.cartesian_impedance_params.cartesian_stiffness.b);
                ccm.parametrize(CartDOF.C).setStiffness(cmd.cartesian_impedance_params.cartesian_stiffness.c);
                ccm.setNullSpaceDamping(cmd.cartesian_impedance_params.nullspace_damping);
                ccm.setNullSpaceStiffness(cmd.cartesian_impedance_params.nullspace_stiffness);
                ccm.setMaxPathDeviation(cmd.cartesian_control_mode_limits.max_path_deviation.x,
                                        cmd.cartesian_control_mode_limits.max_path_deviation.y,
                                        cmd.cartesian_control_mode_limits.max_path_deviation.z,
                                        cmd.cartesian_control_mode_limits.max_path_deviation.a,
                                        cmd.cartesian_control_mode_limits.max_path_deviation.b,
                                        cmd.cartesian_control_mode_limits.max_path_deviation.c);
                ccm.setMaxCartesianVelocity(cmd.cartesian_control_mode_limits.max_cartesian_velocity.x,
                                            cmd.cartesian_control_mode_limits.max_cartesian_velocity.y,
                                            cmd.cartesian_control_mode_limits.max_cartesian_velocity.z,
                                            cmd.cartesian_control_mode_limits.max_cartesian_velocity.a,
                                            cmd.cartesian_control_mode_limits.max_cartesian_velocity.b,
                                            cmd.cartesian_control_mode_limits.max_cartesian_velocity.c);
                ccm.setMaxControlForce(cmd.cartesian_control_mode_limits.max_control_force.x,
                                       cmd.cartesian_control_mode_limits.max_control_force.y,
                                       cmd.cartesian_control_mode_limits.max_control_force.z,
                                       cmd.cartesian_control_mode_limits.max_control_force.a,
                                       cmd.cartesian_control_mode_limits.max_control_force.b,
                                       cmd.cartesian_control_mode_limits.max_control_force.c,
                                       cmd.cartesian_control_mode_limits.stop_on_max_control_force);
                cm = ccm;
                break;
            }
            default:
            {
                getLogger().error("Unknown control mode from LCM: " + cmd.control_mode);
                break;
            }
        }
        getLogger().info("Leaving buildMotionControlMode");
        return cm;

    }

    public void applyControlMode(control_mode_parameters cmd)
    {
        getLogger().info("Changing control mode");
        synchronized (arm_io_lock_)
        {
            ControlMode previous_control_mode = active_control_mode_;
            boolean new_control_mode_recognized = false;
            switch (cmd.control_mode.mode)
            {
                case control_mode.JOINT_POSITION:
                {
                    getLogger().info("Switch to ControlMode:" + ControlMode.JOINT_POSITION);
                    active_control_mode_ = ControlMode.JOINT_POSITION;
                    new_control_mode_recognized = true;
                    break;
                }    
                case control_mode.CARTESIAN_POSE:
                {
                    getLogger().info("Switch to ControlMode:" + ControlMode.CARTESIAN_POSE);
                    active_control_mode_ = ControlMode.CARTESIAN_POSE;
                    new_control_mode_recognized = true;
                    break;
                }
                case control_mode.JOINT_IMPEDANCE:
                {
                    getLogger().info("Switch to ControlMode:" + ControlMode.JOINT_IMPEDANCE);
                    active_control_mode_ = ControlMode.JOINT_IMPEDANCE;
                    new_control_mode_recognized = true;
                    break;
                }
                case control_mode.CARTESIAN_IMPEDANCE:
                {
                    getLogger().info("Switch to ControlMode:" + ControlMode.CARTESIAN_IMPEDANCE);
                    active_control_mode_ = ControlMode.CARTESIAN_IMPEDANCE;
                    new_control_mode_recognized = true;
                    break;
                }
                default:
                {
                    getLogger().error("Unknown control mode from LCM: " + cmd.control_mode);
                    break;
                }
            }
            if (new_control_mode_recognized)
            {
                // Always rebuild the control mode
                // Stop the previous control mode
                if (previous_control_mode == ControlMode.JOINT_POSITION || previous_control_mode == ControlMode.JOINT_IMPEDANCE)
                {
                    getLogger().info("Stopping old Joint control mode");
                    SmartServo oldmotion = joint_smartservo_motion_;
                    boolean stopped = oldmotion.getRuntime().stopMotion();
                    assert(stopped);
                }
                else if (previous_control_mode == ControlMode.CARTESIAN_POSE || previous_control_mode == ControlMode.CARTESIAN_IMPEDANCE)
                {
                    getLogger().info("Stopping old Cartesian control mode");
                    SmartServoLIN oldmotion = cartesian_smartservo_motion_;
                    boolean stopped = oldmotion.getRuntime().stopMotion();
                    assert(stopped);
                }
                else
                {
                    getLogger().info("No old control mode to stop");
                }

                // Build the new control mode
                if (active_control_mode_ == ControlMode.JOINT_POSITION)
                {
                    getLogger().info("Building new Joint control mode");
                    joint_smartservo_motion_ = createSmartServoMotion(cmd.joint_path_execution_params);
                    joint_smartservo_motion_.setMode(buildMotionControlMode(cmd));
                    end_effector_frame_.moveAsync(joint_smartservo_motion_);
                    getLogger().info("Finished building new Joint control mode");
                }
                else if (active_control_mode_ == ControlMode.JOINT_IMPEDANCE)
                {
                    if (tool_ != null)
                    {
                        getLogger().info("Attempting to validate for impedance mode (arm + tool)");
                        boolean validated = SmartServo.validateForImpedanceMode(tool_);
                        assert(validated);
                    }
                    else
                    {
                        getLogger().info("Attempting to validate for impedance mode (arm only)");
                        boolean validated = SmartServo.validateForImpedanceMode(iiwa7_arm_);
                        assert(validated);
                    }
                    getLogger().info("Building new Joint control mode");
                    joint_smartservo_motion_ = createSmartServoMotion(cmd.joint_path_execution_params);
                    joint_smartservo_motion_.setMode(buildMotionControlMode(cmd));
                    end_effector_frame_.moveAsync(joint_smartservo_motion_);
                    getLogger().info("Finished building new Joint control mode");
                }
                else if (active_control_mode_ == ControlMode.CARTESIAN_POSE)
                {
                    getLogger().info("Building new Cartesian control mode");
                    cartesian_smartservo_motion_ = createSmartServoLINMotion(cmd.cartesian_path_execution_params);
                    cartesian_smartservo_motion_.setMode(buildMotionControlMode(cmd));
                    end_effector_frame_.moveAsync(cartesian_smartservo_motion_);
                    getLogger().info("Finished building new Cartesian control mode");
                }
                else if (active_control_mode_ == ControlMode.CARTESIAN_IMPEDANCE)
                {
                    if (tool_ != null)
                    {
                        getLogger().info("Attempting to validate for impedance mode (arm + tool)");
                        boolean validated = SmartServoLIN.validateForImpedanceMode(tool_);
                        assert(validated);
                    }
                    else
                    {
                        getLogger().info("Attempting to validate for impedance mode (arm only)");
                        boolean validated = SmartServoLIN.validateForImpedanceMode(iiwa7_arm_);
                        assert(validated);
                    }
                    getLogger().info("Building new Cartesian control mode");
                    cartesian_smartservo_motion_ = createSmartServoLINMotion(cmd.cartesian_path_execution_params);
                    cartesian_smartservo_motion_.setMode(buildMotionControlMode(cmd));
                    end_effector_frame_.moveAsync(cartesian_smartservo_motion_);
                    getLogger().info("Finished building new Cartesian control mode");
                }
                else
                {
                    getLogger().info("No new control mode to build");
                }
                // Update params
                joint_path_execution_params_ = cmd.joint_path_execution_params;
                cartesian_path_execution_params_ = cmd.cartesian_path_execution_params;
            }
            else
            {
                getLogger().info("New control mode not recognized, ignoring");
            }
        }
    }

    
    private class ControlModePublisher
    {
        private final control_mode_parameters control_mode_status_msg_;
        
        private final Timer timer_;
        private final TimerTask feedback_loop_task_;
        
        public ControlModePublisher()
        {
            final boolean is_daemon = true; 
            timer_ = new Timer(is_daemon);        
        
            // We need to make sure these get initialized into a valid state!
            // The numbers here are meaningless, we are going to get new values as soon as the program starts.
            control_mode_status_msg_ = new control_mode_parameters();
            control_mode_status_msg_.joint_impedance_params = new joint_impedance_parameters();
            control_mode_status_msg_.joint_impedance_params.joint_stiffness = Conversions.jvqInitializer(0.0);
            control_mode_status_msg_.joint_impedance_params.joint_damping = Conversions.jvqInitializer(0.0);
            
            control_mode_status_msg_.cartesian_impedance_params = new cartesian_impedance_parameters();
            control_mode_status_msg_.cartesian_impedance_params.cartesian_stiffness = Conversions.cvqInitializer(0.1);
            control_mode_status_msg_.cartesian_impedance_params.nullspace_damping = 0.3;
            control_mode_status_msg_.cartesian_impedance_params.cartesian_damping = Conversions.cvqInitializer(0.1);
            control_mode_status_msg_.cartesian_impedance_params.nullspace_stiffness = 0.0;
            
            control_mode_status_msg_.cartesian_control_mode_limits = new cartesian_control_mode_limits();
            control_mode_status_msg_.cartesian_control_mode_limits.max_cartesian_velocity = Conversions.cvqInitializer(0.1);
            control_mode_status_msg_.cartesian_control_mode_limits.max_control_force = Conversions.cvqInitializer(0.1);
            control_mode_status_msg_.cartesian_control_mode_limits.max_path_deviation = Conversions.cvqInitializer(0.1);
            control_mode_status_msg_.cartesian_control_mode_limits.stop_on_max_control_force = false;
            
            //These numbers must be set by the encapsulating class.
            control_mode_status_msg_.joint_path_execution_params = joint_path_execution_params_;
            control_mode_status_msg_.cartesian_path_execution_params = cartesian_path_execution_params_;
            
            control_mode_status_msg_.control_mode = new control_mode();
            
            feedback_loop_task_ = new TimerTask()
            {
                @Override
                public void run ()
                {
                    synchronized(arm_io_lock_)
                    {
                        final double now = Utils.getUTCTimeAsDouble();
                        control_mode_status_msg_.timestamp = now;
                        control_mode_status_msg_.control_mode.mode = (byte) active_control_mode_.ordinal();
                        // Populate everything by default
                        control_mode_status_msg_.joint_path_execution_params = joint_path_execution_params_;
                        control_mode_status_msg_.cartesian_path_execution_params = cartesian_path_execution_params_;
                        // Get running path execution & impedance params if possible
                        if (active_control_mode_ == ControlMode.JOINT_POSITION)
                        {
                            // Path execution params
                            control_mode_status_msg_.joint_path_execution_params.joint_relative_acceleration = joint_smartservo_motion_.getJointAccelerationRel();
                            control_mode_status_msg_.joint_path_execution_params.joint_relative_velocity = joint_smartservo_motion_.getJointVelocityRel();
                            control_mode_status_msg_.joint_path_execution_params.override_joint_acceleration = joint_path_execution_params_.override_joint_acceleration;
                        }
                        else if (active_control_mode_ == ControlMode.JOINT_IMPEDANCE)
                        {
                            // Impedance params
                            JointImpedanceControlMode jcm = (JointImpedanceControlMode)joint_smartservo_motion_.getMode();
                            Conversions.vectorToJvq(jcm.getDamping(), control_mode_status_msg_.joint_impedance_params.joint_damping);
                            Conversions.vectorToJvq(jcm.getStiffness(), control_mode_status_msg_.joint_impedance_params.joint_stiffness);
                            // Path execution params
                            control_mode_status_msg_.joint_path_execution_params.joint_relative_acceleration = joint_smartservo_motion_.getJointAccelerationRel();
                            control_mode_status_msg_.joint_path_execution_params.joint_relative_velocity = joint_smartservo_motion_.getJointVelocityRel();
                            control_mode_status_msg_.joint_path_execution_params.override_joint_acceleration = joint_path_execution_params_.override_joint_acceleration;
                        }
                        else if (active_control_mode_ == ControlMode.CARTESIAN_IMPEDANCE)
                        {
                            CartesianImpedanceControlMode ccm = (CartesianImpedanceControlMode)cartesian_smartservo_motion_.getMode();
                            // Impedance params
                            Conversions.vectorToCvq(ccm.getDamping(), control_mode_status_msg_.cartesian_impedance_params.cartesian_damping);
                            Conversions.vectorToCvq(ccm.getStiffness(), control_mode_status_msg_.cartesian_impedance_params.cartesian_stiffness);
                            control_mode_status_msg_.cartesian_impedance_params.nullspace_damping = ccm.getNullSpaceDamping();
                            control_mode_status_msg_.cartesian_impedance_params.nullspace_stiffness = ccm.getNullSpaceStiffness();
                            // Cartesian control mode limits
                            Conversions.vectorToCvq(ccm.getMaxCartesianVelocity(), control_mode_status_msg_.cartesian_control_mode_limits.max_cartesian_velocity);
                            Conversions.vectorToCvq(ccm.getMaxPathDeviation(), control_mode_status_msg_.cartesian_control_mode_limits.max_path_deviation);
                            Conversions.vectorToCvq(ccm.getMaxControlForce(), control_mode_status_msg_.cartesian_control_mode_limits.max_control_force);
                            control_mode_status_msg_.cartesian_control_mode_limits.stop_on_max_control_force = ccm.hasMaxControlForceStopCondition();
                        }
                    }                    
                    lcm_publisher_.publish(CONTROL_MODE_STATUS_CHANNEL, control_mode_status_msg_);
                }
            };
        }
        
        public void start()
        {
            // schedule the task to run now, and then every T milliseconds
            timer_.schedule(feedback_loop_task_, 0, CONTROL_MODE_FEEDBACK_PERIOD_MS);
        }
        
        public void cancel()
        {
            timer_.cancel();
        }
    }
        
    private class MotionStatusPublisher
    {
        private final motion_status motion_status_msg_;
        
        private final Timer timer_;
        private final TimerTask feedback_loop_task_;
        
        public MotionStatusPublisher()
        {
            motion_status_msg_ = new motion_status();
            motion_status_msg_.measured_joint_position = Conversions.jvqInitializer(0.0);
            motion_status_msg_.commanded_joint_position = Conversions.jvqInitializer(0.0);
            motion_status_msg_.measured_joint_velocity = Conversions.jvqInitializer(0.0);
            motion_status_msg_.measured_joint_torque = Conversions.jvqInitializer(0.0);
            motion_status_msg_.estimated_external_torque = Conversions.jvqInitializer(0.0);
            motion_status_msg_.measured_cartesian_pose_abc = Conversions.cvqInitializer(0.0);
            motion_status_msg_.commanded_cartesian_pose_abc = Conversions.cvqInitializer(0.0);
            motion_status_msg_.measured_cartesian_pose = Conversions.identityPose();
            motion_status_msg_.commanded_cartesian_pose = Conversions.identityPose();
            motion_status_msg_.estimated_external_wrench = Conversions.cvqInitializer(0.0);
            motion_status_msg_.active_control_mode = new control_mode();
            
            timer_ = new Timer();
            feedback_loop_task_ = new TimerTask()
            {
                @Override
                public void run ()
                {
                    synchronized (arm_io_lock_)
                    {
                        final double now = Utils.getUTCTimeAsDouble();
                        motion_status_msg_.timestamp = now;
                        motion_status_msg_.active_control_mode.mode = (byte)active_control_mode_.ordinal();
                        Conversions.jointPositionToJvq(iiwa7_arm_.getCurrentJointPosition(), motion_status_msg_.measured_joint_position);
                        Conversions.jointPositionToJvq(iiwa7_arm_.getCommandedJointPosition(), motion_status_msg_.commanded_joint_position);
                        Conversions.jvqInitializer(0.0, motion_status_msg_.measured_joint_velocity); // No joint velocity data exists natively
                        Conversions.vectorToJvq(iiwa7_arm_.getMeasuredTorque().getTorqueValues(), motion_status_msg_.measured_joint_torque);
                        Conversions.vectorToJvq(iiwa7_arm_.getExternalTorque().getTorqueValues(), motion_status_msg_.estimated_external_torque);
                        Transformation commanded_world_ee = iiwa7_arm_.getCommandedCartesianPosition(end_effector_frame_).transformationFromWorld();
                        Transformation measured_world_ee = iiwa7_arm_.getCurrentCartesianPosition(end_effector_frame_).transformationFromWorld();
                        Conversions.transformationToCvq(measured_world_ee, motion_status_msg_.measured_cartesian_pose_abc);
                        Conversions.transformationToCvq(commanded_world_ee, motion_status_msg_.commanded_cartesian_pose_abc);
                        Conversions.transformationToPose(measured_world_ee, motion_status_msg_.measured_cartesian_pose);
                        Conversions.transformationToPose(commanded_world_ee, motion_status_msg_.commanded_cartesian_pose);
                        Conversions.forceTorqueToCvq(iiwa7_arm_.getExternalForceTorque(end_effector_frame_), motion_status_msg_.estimated_external_wrench);
                    }
                    lcm_publisher_.publish(MOTION_STATUS_CHANNEL, motion_status_msg_);
                }
            };
        }
        
        public void start()
        {
            // schedule the task to run now, and then every T milliseconds
            timer_.schedule(feedback_loop_task_, 0, MOTION_STATUS_FEEDBACK_PERIOD_MS);
        }
        
        public void cancel()
        {
            timer_.cancel();
        }
    }
        
    private class Robotiq3FingerGripperPublisher
    {
        private final robotiq_3finger_status gripper_status_msg_;
        
        private final Timer timer_;
        private final TimerTask feedback_loop_task_;
        
        public Robotiq3FingerGripperPublisher()
        {
            // Double check some assertions regarding message definitions and internal representations
            assert((byte) Robotiq3FingerGripperStatus.InitializationStatus.GRIPPER_RESET.ordinal() == robotiq_3finger_status.GRIPPER_RESET);
            assert((byte) Robotiq3FingerGripperStatus.InitializationStatus.GRIPPER_ACTIVATION.ordinal() == robotiq_3finger_status.GRIPPER_ACTIVATION);
            
            assert((byte) Robotiq3FingerGripperStatus.GripperActionStatus.GRIPPER_STOPPED_OR_BUSY.ordinal() == robotiq_3finger_status.GRIPPER_STOPPED_OR_BUSY);
            assert((byte) Robotiq3FingerGripperStatus.GripperActionStatus.GRIPPER_GOTO.ordinal() == robotiq_3finger_status.GRIPPER_GOTO);
    
            assert((byte) Robotiq3FingerGripperStatus.GripperSystemStatus.GRIPPER_RESET_OR_AUTO_RELEASE.ordinal() == robotiq_3finger_status.GRIPPER_RESET_OR_AUTO_RELEASE);
            assert((byte) Robotiq3FingerGripperStatus.GripperSystemStatus.GRIPPER_ACTIVATION_IN_PROGRESS.ordinal() == robotiq_3finger_status.GRIPPER_ACTIVATION_IN_PROGRESS);
            assert((byte) Robotiq3FingerGripperStatus.GripperSystemStatus.GRIPPER_MODE_CHANGE_IN_PROGESS.ordinal() == robotiq_3finger_status.GRIPPER_MODE_CHANGE_IN_PROGRESS);
            assert((byte) Robotiq3FingerGripperStatus.GripperSystemStatus.GRIPPER_ACTIVATION_MODE_CHANGE_COMPLETE.ordinal() == robotiq_3finger_status.GRIPPER_ACTIVATION_MODE_CHANGE_COMPLETE);
            
            assert((byte) Robotiq3FingerGripperStatus.GripperMotionStatus.GRIPPER_STOPPED_UNKNOWN.ordinal() == robotiq_3finger_status.GRIPPER_STOPPED_UNKNOWN);
            assert((byte) Robotiq3FingerGripperStatus.GripperMotionStatus.GRIPPER_IN_MOTION.ordinal() == robotiq_3finger_status.GRIPPER_IN_MOTION);
            assert((byte) Robotiq3FingerGripperStatus.GripperMotionStatus.GRIPPER_ONE_OR_TWO_STOPPED_EARLY.ordinal() == robotiq_3finger_status.GRIPPER_ONE_OR_TWO_STOPPED_EARLY);
            assert((byte) Robotiq3FingerGripperStatus.GripperMotionStatus.GRIPPER_ALL_STOPPED_EARLY.ordinal() == robotiq_3finger_status.GRIPPER_ALL_STOPPED_EARLY);
            assert((byte) Robotiq3FingerGripperStatus.GripperMotionStatus.GRIPPER_ALL_AT_REQUESTED.ordinal() == robotiq_3finger_status.GRIPPER_ALL_AT_REQUESTED);
    
            // TODO: These assertion need to be here, but the enum doesn't match. Need to fix this when we do the gripper status code review.
//            assert((byte) Robotiq3FingerGripperStatus.ObjectStatus.STOPPED.ordinal() == robotiq_3finger_object_status.STOPPED);
//            assert((byte) Robotiq3FingerGripperStatus.ObjectStatus.IN_MOTION.ordinal() == robotiq_3finger_object_status.IN_MOTION);
//            assert((byte) Robotiq3FingerGripperStatus.ObjectStatus.CONTACT_OPENING.ordinal() == robotiq_3finger_object_status.CONTACT_OPENING);
//            assert((byte) Robotiq3FingerGripperStatus.ObjectStatus.CONTACT_CLOSING.ordinal() == robotiq_3finger_object_status.CONTACT_CLOSING);
//            assert((byte) Robotiq3FingerGripperStatus.ObjectStatus.AT_REQUESTED.ordinal() == robotiq_3finger_object_status.AT_REQUESTED);
            // TODO: The assertion for the GripperFaultStatus check also need to be here, but as the current code directly return the values from robotiq_3finger_status, we don't need this now.
            // TODO: The enum of GripperFaultStatus also doesn't match the robotiq_3finger_status.
            gripper_status_msg_ = new robotiq_3finger_status();
            
            gripper_status_msg_.finger_a_status = new robotiq_3finger_actuator_status();
            gripper_status_msg_.finger_b_status = new robotiq_3finger_actuator_status();
            gripper_status_msg_.finger_c_status = new robotiq_3finger_actuator_status();
            gripper_status_msg_.scissor_status = new robotiq_3finger_actuator_status();
            
            gripper_status_msg_.finger_a_object_status = new robotiq_3finger_object_status();
            gripper_status_msg_.finger_b_object_status = new robotiq_3finger_object_status();
            gripper_status_msg_.finger_c_object_status = new robotiq_3finger_object_status();
            gripper_status_msg_.scissor_object_status = new robotiq_3finger_object_status();
            
            final boolean is_daemon = true; 
            timer_ = new Timer(is_daemon);
            
            feedback_loop_task_ = new TimerTask ()
            {
                @Override
                public void run ()
                {
                    if (tool_ != null)
                    {
                        synchronized(gripper_)
                        {
                            gripper_.PopulateLCMStatusMessage(gripper_status_msg_);
                        }
                        lcm_publisher_.publish(GRIPPER_STATUS_CHANNEL, gripper_status_msg_);
                    }
                }
            };
        }
        
        public void start()
        {
            // schedule the task to run now, and then every T milliseconds
            timer_.schedule(feedback_loop_task_, 0, GRIPPER_FEEDBACK_PERIOD_MS);
        }
    
        public void cancel()
        {
            timer_.cancel();
        }
    }

    /**
     * This class acts as a buffer to store control mode command message    
     * @author armlab
     *
     */
    private class ControlModeCommandHandler
    {
        private control_mode_parameters control_mode_command_;
        private Boolean new_control_mode_command_ready_ = new Boolean(false);
        
        public void storeControlModeCommand(control_mode_parameters cmd)
        {
            synchronized (new_control_mode_command_ready_)
            {
                control_mode_command_ = cmd;
                new_control_mode_command_ready_ = true;
            }
        }
        
        public control_mode_parameters getControlModeCommand()
        {
            synchronized (new_control_mode_command_ready_)
            {
                if (new_control_mode_command_ready_)
                {
                    new_control_mode_command_ready_ = false;
                    return control_mode_command_;
                }
                else
                {
                    return null;
                }
            }
        }
    }

    /**
     * This class acts as a buffer to store motion command message
     * @author armlab
     *
     */
    private class MotionCommandHandler
    {
        private ControlMode control_mode_ = null;
        private final JointPosition joint_position_target_ = new JointPosition(iiwa7_arm_.getJointCount());
        private Frame cartesian_pose_target_ = new Frame();
        private Boolean new_motion_command_ready_ = new Boolean(false);
        
        public void storeMotionCommand(motion_command cmd)
        {
            switch (cmd.control_mode.mode)
            {
                case control_mode.JOINT_POSITION:
                {
                    storeJointPositionCommand(cmd, ControlMode.JOINT_POSITION);
                    break;
                }
                case control_mode.JOINT_IMPEDANCE:
                {
                    storeJointPositionCommand(cmd, ControlMode.JOINT_IMPEDANCE);
                    break;
                }
                case control_mode.CARTESIAN_POSE:
                {
                    storeCartesianPoseCommand(cmd, ControlMode.CARTESIAN_POSE);
                    break;
                }
                case control_mode.CARTESIAN_IMPEDANCE:
                {
                    storeCartesianPoseCommand(cmd, ControlMode.CARTESIAN_IMPEDANCE);
                    break;
                }
                default:
                {
                    getLogger().error("MotionMode: " + cmd.control_mode + " is invalid");
                    break;
                }
            }
        }
    
        private void storeJointPositionCommand(motion_command cmd, ControlMode control_mode)
        {
            synchronized (new_motion_command_ready_)
            {
                Conversions.jvqToJointPosition(cmd.joint_position, joint_position_target_);
                control_mode_ = control_mode;
                new_motion_command_ready_ = true;
            }
        }
    
        private void storeCartesianPoseCommand(motion_command cmd, ControlMode control_mode)
        {
            synchronized (new_motion_command_ready_)
            {    
                cartesian_pose_target_ = Conversions.poseToFrame(cmd.cartesian_pose);
                ObjectFrame cartesian_command_frame = iiwa7_arm_.getRootFrame();
                cartesian_pose_target_.setParent(cartesian_command_frame, true);
                control_mode_ = control_mode;
                new_motion_command_ready_ = true;
            }
        }
        
        public JointPosition getJointPositionTarget()
        {
            synchronized (new_motion_command_ready_)
            {
            	if (new_motion_command_ready_ && control_mode_ == ControlMode.JOINT_POSITION)
            	{
            		new_motion_command_ready_ = false;
            		return joint_position_target_;
            	}
            	else
            	{
            		return null;
            	}
            }
        }
        
        public Frame getCartesianPoseTarget()
        {
            synchronized (new_motion_command_ready_)
            {
            	if (new_motion_command_ready_ && control_mode_ == ControlMode.CARTESIAN_POSE)
            	{
            		new_motion_command_ready_ =  false;
            		return cartesian_pose_target_;
            	}
            	else
            	{
            		return null;
            	}
            }
        }
        
        public JointPosition getJointImpedanceTarget()
        {
            synchronized (new_motion_command_ready_)
            {
                if (new_motion_command_ready_ && control_mode_ == ControlMode.JOINT_IMPEDANCE)
                {
                    new_motion_command_ready_ = false;
                    return joint_position_target_;
                }
                else
                {
                    return null;
                }
            }
        }
        
        public Frame getCartesianImpedanceTarget()
        {
            synchronized (new_motion_command_ready_)
            {
                if (new_motion_command_ready_ && control_mode_ == ControlMode.CARTESIAN_IMPEDANCE)
                {
                	new_motion_command_ready_ = false;
                    return cartesian_pose_target_;
                }
                else
                {
                    return null;
                }
            }
        }
    }
    
    /**
     * This class will command the gripper after receiving a gripper command message. 
     * We may need further discussion on this class about what behavior we want for the gripper.
     * @author armlab
     *
     */
    private class GripperCommandHandler
    {
        private void executeCommand(robotiq_3finger_command cmd)
        {
            if (tool_ != null)
            {
                synchronized(gripper_)
                {
                    gripper_.CommandGripper(new Robotiq3FingerGripperCommand(cmd));
                }
            }
        }
    }
}
