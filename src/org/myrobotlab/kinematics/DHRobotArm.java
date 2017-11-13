package org.myrobotlab.kinematics;

import java.io.Serializable;
import java.util.ArrayList;

import org.myrobotlab.logging.LoggerFactory;
import org.myrobotlab.service.Servo;
import org.slf4j.Logger;

public class DHRobotArm implements Serializable {

  private static final long serialVersionUID = 1L;

  transient public final static Logger log = LoggerFactory.getLogger(DHRobotArm.class);

  private int maxIterations = 1000;

  private ArrayList<DHLink> links;

  public String name;

  public DHRobotArm() {
    super();
    links = new ArrayList<DHLink>();
  }

  public DHRobotArm(DHRobotArm copy) {
    super();
    name = copy.name;
    links = new ArrayList<DHLink>();
    for (DHLink link : copy.links) {
      links.add(new DHLink(link));
    }
  }

  public ArrayList<DHLink> addLink(DHLink link) {
    links.add(link);
    return links;
  }

  public Matrix getJInverse() {
    // something small.
    // double delta = 0.000001;
    double delta = 0.0001;
    int numLinks = this.getNumLinks();
    Matrix jacobian = new Matrix(6, numLinks);
    // compute the gradient of x,y,z based on the joint movement.
    Point basePosition = this.getPalmPosition();
    // log.debug("Base Position : " + basePosition);
    // for each servo, we'll rotate it forward by delta (and back), and get
    // the new positions
    for (int j = 0; j < numLinks; j++) {
      this.getLink(j).incrRotate(delta);
      Point palmPoint = this.getPalmPosition();
      Point deltaPoint = palmPoint.subtract(basePosition);
      this.getLink(j).incrRotate(-delta);
      // delta position / base position gives us the slope / rate of
      // change
      // this is an approx of the gradient of P
      // UHoh,, what about divide by zero?!
      // log.debug("Delta Point" + deltaPoint);
      double dXdj = deltaPoint.getX() / delta;
      double dYdj = deltaPoint.getY() / delta;
      double dZdj = deltaPoint.getZ() / delta;
      double dRXdj = deltaPoint.getYaw() / delta;
      double dRYdj = deltaPoint.getPitch() / delta;
      double dRZdj = deltaPoint.getRoll() / delta;
      jacobian.elements[0][j] = dXdj;
      jacobian.elements[1][j] = dYdj;
      jacobian.elements[2][j] = dZdj;
      jacobian.elements[3][j] = dRXdj;
      jacobian.elements[4][j] = dRYdj;
      jacobian.elements[5][j] = dRZdj;
      
    }
    // log.debug("Jacobian(p)approx");
    // log.info("JACOBIAN\n" +jacobian);
    // This is the MAGIC! the pseudo inverse should map
    // deltaTheta[i] to delta[x,y,z]
    Matrix jInverse = jacobian.pseudoInverse();
    // log.debug("Pseudo inverse Jacobian(p)approx\n" + jInverse);
    if (jInverse == null) {
      jInverse = new Matrix(6, numLinks);
    }
    return jInverse;
  }

  public DHLink getLink(int i) {
    if (links.size() >= i) {
      return links.get(i);
    } else {
      // TODO log a warning or something?
      return null;
    }
  }

  public ArrayList<DHLink> getLinks() {
    return links;
  }

  public int getNumLinks() {
    return links.size();
  }

  public synchronized Point getJointPosition(int index) {
    if (index > this.links.size() || index < 0) {
      // TODO: bound check
      return null;
    }

    Matrix m = new Matrix(4, 4);
    // Identity matrix
    m.elements[0][0] = 1;
    m.elements[1][1] = 1;
    m.elements[2][2] = 1;
    m.elements[3][3] = 1;

    // Get transformation matrix from initial frame to desired frame iterating
    // through each link
    for (int i = 0; i <= index; i++) {
      DHLink link = links.get(i);
      Matrix s = link.resolveMatrix();
      m = m.multiply(s);
    }
    // m contains the transform from initial frame to the desired link, given
    // the arm current position
    double x = m.elements[0][3];
    double y = m.elements[1][3];
    double z = m.elements[2][3];
    Point jointPosition = new Point(x, y, z, 0, 0, 0);
    return jointPosition;
  }

  /**
   * @param lastDHLink
   *          the index of the link that you want the global position at.
   * @return the x,y,z of the palm. roll,pitc, and yaw are not returned/computed
   *         with this function
   */
  public Point getPalmPosition(String lastDHLink) {

    // return getJointPosition(links.size()-1);
    // return the position of the end effector wrt the base frame
    Matrix m = new Matrix(4, 4);
    // Identity matrix
    m.elements[0][0] = 1;
    m.elements[1][1] = 1;
    m.elements[2][2] = 1;
    m.elements[3][3] = 1;

    // Post multiply each transformation matrix climbing frames from base to
    // end effector
    // m will hold the transformation matrix from base to end
    for (int i = 0; i < links.size(); i++) {
      Matrix s = links.get(i).resolveMatrix();
      m = m.multiply(s);
      // log.debug("-------------------------");
      // log.debug(m);
      if (links.get(i).getName() != null && links.get(i).getName().equals(lastDHLink)) {
        break;
      }
    }
    // now m holds the total transformation matrix for the arm
    // given the arms current position
    double x = m.elements[0][3];
    double y = m.elements[1][3];
    double z = m.elements[2][3];
    // Compute RPY angles
    double yaw = Math.atan2(m.elements[2][1], m.elements[2][2]);
    double pitch = Math.atan2(-1 * m.elements[2][0],
        Math.sin(yaw) * m.elements[2][1] + Math.cos(yaw) * m.elements[2][2]);
    double roll = Math.atan2(-1 * Math.cos(yaw) * m.elements[0][1] + Math.sin(yaw) * m.elements[0][2],
        Math.cos(yaw) * m.elements[1][1] - Math.sin(yaw) * m.elements[1][2]);

    Point palm = new Point(x, y, z, pitch * 180 / Math.PI, roll * 180 / Math.PI, yaw * 180 / Math.PI);

    return palm;
  }

  public void zeroAllJoints() {
    for (DHLink link : links) {
      double zero = link.getMin();
      log.info("Zeroing Servo {} to {} degrees", link.getName(), zero * 180.0 / Math.PI);
      link.setTheta(zero);
    }
  }

  public void centerAllJoints() {
    for (DHLink link : links) {
      double center = (link.getMax() + link.getMin()) / 2.0;
      log.info("Centering Servo {} to {} degrees", link.getName(), center * 180.0 / Math.PI);
      link.setTheta(center);
    }
  }
  
  public boolean moveToGoal(Point goal) {
    return moveToGoal(goal, false);
  }

  public boolean moveToGoal(Point goal, boolean useRPY) {
    // we know where we are.. we know where we want to go.
    int numSteps = 0;
    double iterStep = 0.25;
    double errorThreshold = 0.05;
    // what's the current point
    while (true) {
      numSteps++;
      if (numSteps >= maxIterations) {
        log.info("Attempted to iterate there, but didn't make it. giving up.");
        // we shouldn't publish if we don't solve!
        return false;
      }
      // TODO: what if its unreachable!
      Point currentPos = this.getPalmPosition();
      log.debug("Current Position " + currentPos);
      // vector to destination
      Point deltaPoint = goal.subtract(currentPos);
      Matrix dP = new Matrix(6, 1);
      dP.elements[0][0] = deltaPoint.getX();
      dP.elements[1][0] = deltaPoint.getY();
      dP.elements[2][0] = deltaPoint.getZ();
      if (useRPY) {
        dP.elements[4][0] = deltaPoint.getPitch();
      }
      // scale a vector towards the goal by the increment step.
      dP = dP.multiply(iterStep);

      Matrix jInverse = this.getJInverse();
      // why is this zero?
      Matrix dTheta = jInverse.multiply(dP);
      log.debug("delta Theta + " + dTheta);
      for (int i = 0; i < dTheta.getNumRows(); i++) {
        // update joint positions! move towards the goal!
        double d = dTheta.elements[i][0];
        // incr rotate needs to be min/max aware here!
        // TODO Shouldn't this be not clamped, but only clamp the real movement?
        this.getLink(i).incrRotateClamped(d);
      }
      // delta point represents the direction we need to move in order to
      // get there.
      // we should figure out how to scale the steps.

      double error;
      if (useRPY) {
        error = deltaPoint.normXYZP();
      } else {
        error = deltaPoint.magnitude();
      }
      if (error < errorThreshold) {
        break;
      }
    }
    return true;
  }

  public void setLinks(ArrayList<DHLink> links) {
    this.links = links;
  }

  public boolean armMovementEnds() {
    for (DHLink link : links) {
      if (link.getState() != Servo.SERVO_EVENT_STOPPED) {
        return false;
      }
    }
    return true;
  }

  public double[][] createJointPositionMap() {

    double[][] jointPositionMap = new double[getNumLinks() + 1][3];

    // first position is the origin... second is the end of the first link
    jointPositionMap[0][0] = 0;
    jointPositionMap[0][1] = 0;
    jointPositionMap[0][2] = 0;

    for (int i = 1; i <= getNumLinks(); i++) {
      Point jp = getJointPosition(i - 1);
      jointPositionMap[i][0] = jp.getX();
      jointPositionMap[i][1] = jp.getY();
      jointPositionMap[i][2] = jp.getZ();
    }
    return jointPositionMap;
  }

  public Point getVector() {
    Point lastJoint = getJointPosition(links.size() - 1);
    Point previousJoint = getJointPosition(links.size() - 2);
    Point retval = lastJoint.subtract(previousJoint);

    return retval;
  }

  public Point getPalmPosition() {
    return getPalmPosition(null);
  }
}
