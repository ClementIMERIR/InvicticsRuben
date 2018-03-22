package programIMERIR;


import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.Workpiece;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class TrainingLegs_point extends RoboticsAPIApplication {
	@Inject
	private LBR robot;
	@Inject
	@Named("LegLift")
	private Tool legLift;//Création d'un objet outil
	@Inject
	@Named("Leg_thomas")
	private Tool leg_thomas;
	
	//variable accessible via process data
	private Integer tempo, 
					nbcycle;
	private Double angle;
	private Double vitesse;
	private String name;

	@Override
	public void initialize() {
		// initialize your application here
		legLift.attachTo(robot.getFlange());//"Fixation" de l'outil à la bride du robot.
	}

	@Override
	public void run() {
		// your application execution starts here
		robot.move(ptpHome().setJointVelocityRel(0.5));
		legLift.getFrame("/Dummy/PNP_parent").move(ptp(getApplicationData().getFrame("/Genoux/P1")));
		//message variable answeranswer=getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "La jambe du patient est elle en place ?", "Oui","Non");
		ThreadUtil.milliSleep(tempo);//10 sec pour détacher sa jambe
		//leg_thomas.getFrame("PNP_enfant").attachTo(legLift.getRootFrame());

				leg_thomas.getFrame("Genoux").move(linRel(0,0,0,Math.toRadians(-angle),0,0).setCartVelocity(vitesse));
				leg_thomas.getFrame("Genoux").move(linRel(0,0,0,Math.toRadians(angle),0,0).setCartVelocity(vitesse));
			
	
		ThreadUtil.milliSleep(tempo);//10 sec pour détacher sa jambe
		leg_thomas.detach();//detache la jambe de l'outil en logiciel 
		robot.move(ptpHome().setJointVelocityRel(0.5));
	}
}