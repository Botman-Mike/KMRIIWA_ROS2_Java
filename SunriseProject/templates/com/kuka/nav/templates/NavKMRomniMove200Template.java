package com.kuka.nav.templates;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.nav.OrientationMode;
import com.kuka.nav.Orientations;
import com.kuka.nav.Pose;
import com.kuka.nav.Position;
import com.kuka.nav.fleet.ChangeGraphCommand;
import com.kuka.nav.fleet.FleetManager;
import com.kuka.nav.fleet.GraphMotion;
import com.kuka.nav.fleet.filter.InstanceFilter;
import com.kuka.nav.fleet.graph.TopologyGraph;
import com.kuka.nav.fleet.graph.TopologyNode;
import com.kuka.nav.line.VirtualLine;
import com.kuka.nav.line.VirtualLineMotion;
import com.kuka.nav.recover.OffStartPoseCondition;
import com.kuka.nav.recover.ReturnToStartPoseRecover;
import com.kuka.nav.rel.RelativeMotion;
import com.kuka.nav.robot.MobileRobot;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.task.ITaskLogger;

/**
 * Implementation of a robot application.
 * <p>
 * This {@link RoboticsAPIApplication} provides a {@link #run()} method. This method 
 * will be called in the application lifecycle after the {@link #initialize()}-method 
 * and before the{@link #dispose()}-method (which may both be overriden). The application  
 * will terminate automatically after the {@link #run()} method has finished or after 
 * stopping the task. The {@link #dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link #dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class NavKMRomniMove200Template extends RoboticsAPIApplication
{
	@Inject
	private ITaskLogger log;
	
	/**
	 * Inject LBR.
	 */
	@Inject
	private LBR lbrIiwa14R820;
	
	/**
	 * Inject KMR-resource --- note that the name in the @Named annotation must be the same as the name given
	 * to the Nav KMR omniMove 200 in the topology.
	 */ 
	@Inject
	@Named("Nav_KMR_iiwa_14_R820_1")
	private MobileRobot kmr;
	
	/**
	 * Inject fleet manager --- note that the fleet manager is only available for injection in the single installation case
	 * (any application) or in the fleet installation case (applications running on the server). If in the fleet installation
	 * case an application involving the fleet manager is deployed and started on the robot the application will fail.
	 */
	@Inject
	private FleetManager fleetManager;

	@Override
	public void run() throws Exception
	{
		lbrIiwa14R820.move(ptpHome());
		
		double x = 0.2;
		double y = 0.0;
		double theta = Math.toRadians(5.0);
		
		try
		{
			/*
			 * All navigation motions require the robot to be locked first.
			 */
			kmr.lock();
			
			/*
			 * Performs a relative motion to the given pose (x, y, theta).
			 */
			kmr.execute(new RelativeMotion(x, y, theta));
			
			/*
			 * Performs a motion on a virtual line from the current pose of
			 * the robot to the given goal pose (x, y, theta). The platform
			 * will rotate while moving on the line to reach the given
			 * orientation theta.
			 */
			kmr.execute(new VirtualLineMotion(kmr.getPose(), new Pose(x, y, theta)));
			
			/*
			 * Performs a motion on a virtual line from the current pose of
			 * the robot to the given goal position (x, y). The allowed
			 * orientations of the platform relative to the line are 0°
			 * and 180°. If this is not the case the motion will fail.
			 */
			kmr.execute(new VirtualLineMotion(
					VirtualLine.from(kmr.getPose().toPosition()).to(new Position(x, y)),
					Orientations.LENGTHWISE, OrientationMode.FIXED));
			
			/*
			 * Performs a motion on a virtual line from the current pose of
			 * the robot to the given goal pose (x, y, theta). The allowed
			 * orientations of the platform relative to the line are 0°
			 * and 180°. If this is not the case the platform will rotate
			 * to the closest orientation.
			 */
			kmr.execute(new VirtualLineMotion(
					VirtualLine.from(kmr.getPose().toPosition()).to(new Position(x, y)),
					Orientations.LENGTHWISE, OrientationMode.FIXED)
					.recoverWhen(new OffStartPoseCondition(0.1, Math.toRadians(180)), new ReturnToStartPoseRecover()));
		}
		catch (Exception e)
		{
			log.error("Something went wrong while trying to excecute navigation motions on the robot.", e);
		}
		finally
		{
			/*
			 * Make sure that platform is unlocked in the end.
			 */
			kmr.unlock();
		}
		
		/*
		 * Uses the fleet manager to command a graph motion in graph 1 to node 2. Before executing the motion the platform must be set to the graph.
		 * For this purpose the robot must be at node 1.
		 * 
		 * Note that the robot does not have to be explicitly locked (and should not be, since the graph motion will fail otherwise) before commanding
		 * the graph motion to the fleet manager.
		 * 
		 * Note again that the fleet manager is only available in the single installation case (any application) or in the
		 * fleet installation case (applications running on the server). If in the fleet installation case an application involving 
		 * the fleet manager is deployed and started on a robot the application will fail.
		 */
		try
		{
			kmr.lock();
			kmr.execute(new ChangeGraphCommand(1, 1));
		}
		finally
		{
			kmr.unlock();
		}
		
		fleetManager.execute(new GraphMotion(TopologyGraph.withId(1), TopologyNode.withId(1))
				.setResourceFilter(new InstanceFilter(kmr.getId())));
	}
}
